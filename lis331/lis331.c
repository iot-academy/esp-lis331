/**
 * @file lis331.c
 * @brief Implementation of the LIS331DLH accelerometer driver.
 *
 * Register addresses, bit fields and sensitivity values follow the LIS331DLH
 * datasheet (see docs/). I2C access uses the native ESP-IDF I2C master driver
 * (`driver/i2c_master.h`). The transport layer is isolated behind small static
 * helpers so the register logic does not depend on a specific I2C
 * implementation.
 *
 * @ingroup lis331
 */

#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lis331.h"

/** @brief WHO_AM_I device identification register. */
#define LIS331_REG_WHO_AM_I  0x0F
/** @brief CTRL_REG1: PM[2:0], DR[2:0], ZEN, YEN, XEN. */
#define LIS331_REG_CTRL1     0x20
/** @brief CTRL_REG4: BDU, BLE, FS[1:0]. */
#define LIS331_REG_CTRL4     0x23
/** @brief STATUS_REG: ZYXDA, ZYXOR and per-axis data/overrun flags. */
#define LIS331_REG_STATUS    0x27
/** @brief First output register (OUT_X_L); the X/Y/Z block is 6 bytes. */
#define LIS331_REG_OUT_X_L   0x28

/** @brief CTRL_REG1 mask for the PM[2:0] and DR[2:0] fields. */
#define LIS331_CTRL1_PM_DR_MASK  0xF8
/** @brief CTRL_REG1 axes-enable bits: ZEN | YEN | XEN (all axes on). */
#define LIS331_CTRL1_AXES         0x07
/** @brief CTRL_REG4.BDU: block data update. */
#define LIS331_CTRL4_BDU          0x80
/** @brief CTRL_REG4.BLE: big/little endian; kept 0 (little endian). */
#define LIS331_CTRL4_BLE          0x40
/** @brief CTRL_REG4 mask for the FS[1:0] full-scale field. */
#define LIS331_CTRL4_FS_MASK      0x30
/** @brief STATUS_REG.ZYXDA: new data available for all axes. */
#define LIS331_STATUS_ZYXDA       0x08
/** @brief Sub-address MSB enabling register auto-increment during bursts. */
#define LIS331_I2C_AUTO_INCREMENT 0x80
/** @brief Timeout for a single I2C transaction, in milliseconds. */
#define LIS331_I2C_TIMEOUT_MS     1000
/** @brief Standard gravity used for the m/s^2 conversion. */
#define LIS331_GRAVITY_M_S2       9.80665f

/**
 * @brief Driver instance.
 *
 * Wraps the transport-layer I2C device handle. Allocated in lis331_create()
 * and released in lis331_delete(). The application owns the I2C bus itself.
 */
struct lis331_dev_t {
    i2c_master_dev_handle_t i2c_dev;
};

/** @brief Logging tag for this component. */
static const char *TAG = "lis331";

/**
 * @brief Read a single register.
 *
 * @param[in]  handle Driver handle.
 * @param[in]  reg    Register address.
 * @param[out] value  Receives the register content.
 *
 * @return ESP_OK or any I2C transport error.
 */
static esp_err_t lis331_read_reg(lis331_handle_t handle, uint8_t reg, uint8_t *value)
{
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, value, 1,
                                       LIS331_I2C_TIMEOUT_MS);
}

/**
 * @brief Read a block of consecutive registers.
 *
 * @param[in]  handle Driver handle.
 * @param[in]  reg    First register address (auto-increment applied by the
 *                    sub-address MSB when required).
 * @param[out] values Buffer receiving `length` bytes.
 * @param[in]  length Number of bytes to read.
 *
 * @return ESP_OK or any I2C transport error.
 */
static esp_err_t lis331_read_regs(lis331_handle_t handle, uint8_t reg, uint8_t *values,
                                  size_t length)
{
    if (handle == NULL || values == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, values, length,
                                       LIS331_I2C_TIMEOUT_MS);
}

/**
 * @brief Write a single register.
 *
 * @param[in] handle Driver handle.
 * @param[in] reg    Register address.
 * @param[in] value  Value to write.
 *
 * @return ESP_OK or any I2C transport error.
 */
static esp_err_t lis331_write_reg(lis331_handle_t handle, uint8_t reg, uint8_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t transaction[] = {reg, value};
    return i2c_master_transmit(handle->i2c_dev, transaction, sizeof(transaction),
                               LIS331_I2C_TIMEOUT_MS);
}

/**
 * @brief Read-modify-write of a bit field in a register.
 *
 * Only the bits selected by `mask` are changed; the rest of the register is
 * preserved.
 *
 * @param[in] handle Driver handle.
 * @param[in] reg    Register address.
 * @param[in] mask   Bit mask of the field to modify.
 * @param[in] value  New field value (only bits in `mask` are applied).
 *
 * @return ESP_OK or any I2C transport error.
 */
static esp_err_t lis331_update_reg_bits(lis331_handle_t handle, uint8_t reg,
                                        uint8_t mask, uint8_t value)
{
    uint8_t current;
    esp_err_t ret = lis331_read_reg(handle, reg, &current);
    if (ret != ESP_OK) {
        return ret;
    }

    current = (current & ~mask) | (value & mask);
    return lis331_write_reg(handle, reg, current);
}

/**
 * @brief Map an ODR/power-mode enum to the CTRL_REG1 PM|DR bit field.
 *
 * @param[in]  odr          Enum value.
 * @param[out] ctrl1_bits   PM[2:0] | DR[2:0] field, zeroed otherwise.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_ARG for an unknown enum or NULL output.
 */
static esp_err_t lis331_odr_to_ctrl1(lis331_odr_t odr, uint8_t *ctrl1_bits)
{
    if (ctrl1_bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (odr) {
    case LIS331_ODR_POWER_DOWN: *ctrl1_bits = 0x00; break;
    case LIS331_ODR_NORMAL_50_HZ: *ctrl1_bits = 0x20; break;
    case LIS331_ODR_NORMAL_100_HZ: *ctrl1_bits = 0x28; break;
    case LIS331_ODR_NORMAL_400_HZ: *ctrl1_bits = 0x30; break;
    case LIS331_ODR_NORMAL_1000_HZ: *ctrl1_bits = 0x38; break;
    case LIS331_ODR_LOW_POWER_0_5_HZ: *ctrl1_bits = 0x40; break;
    case LIS331_ODR_LOW_POWER_1_HZ: *ctrl1_bits = 0x60; break;
    case LIS331_ODR_LOW_POWER_2_HZ: *ctrl1_bits = 0x80; break;
    case LIS331_ODR_LOW_POWER_5_HZ: *ctrl1_bits = 0xA0; break;
    case LIS331_ODR_LOW_POWER_10_HZ: *ctrl1_bits = 0xC0; break;
    default: return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Reverse map of the CTRL_REG1 PM|DR field to an ODR/power-mode enum.
 *
 * @param[in]  ctrl1 CTRL_REG1 content.
 * @param[out] odr   Receives the matching enum value.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG for NULL output, or
 *         ESP_ERR_INVALID_RESPONSE for an unknown PM|DR field.
 */
static esp_err_t lis331_ctrl1_to_odr(uint8_t ctrl1, lis331_odr_t *odr)
{
    if (odr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (ctrl1 & LIS331_CTRL1_PM_DR_MASK) {
    case 0x00: *odr = LIS331_ODR_POWER_DOWN; break;
    case 0x20: *odr = LIS331_ODR_NORMAL_50_HZ; break;
    case 0x28: *odr = LIS331_ODR_NORMAL_100_HZ; break;
    case 0x30: *odr = LIS331_ODR_NORMAL_400_HZ; break;
    case 0x38: *odr = LIS331_ODR_NORMAL_1000_HZ; break;
    case 0x40: *odr = LIS331_ODR_LOW_POWER_0_5_HZ; break;
    case 0x60: *odr = LIS331_ODR_LOW_POWER_1_HZ; break;
    case 0x80: *odr = LIS331_ODR_LOW_POWER_2_HZ; break;
    case 0xA0: *odr = LIS331_ODR_LOW_POWER_5_HZ; break;
    case 0xC0: *odr = LIS331_ODR_LOW_POWER_10_HZ; break;
    default: return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

/**
 * @brief Map a full-scale range enum to the CTRL_REG4 FS[1:0] field.
 *
 * @param[in]  range       Enum value.
 * @param[out] ctrl4_bits  FS[1:0] field, zeroed otherwise.
 *
 * @return ESP_OK, or ESP_ERR_INVALID_ARG for an unknown enum or NULL output.
 */
static esp_err_t lis331_range_to_ctrl4(lis331_range_t range, uint8_t *ctrl4_bits)
{
    if (ctrl4_bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (range) {
    case LIS331_RANGE_2G: *ctrl4_bits = 0x00; break;
    case LIS331_RANGE_4G: *ctrl4_bits = 0x10; break;
    case LIS331_RANGE_8G: *ctrl4_bits = 0x30; break;
    default: return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Reverse map of the CTRL_REG4 FS[1:0] field to a range enum.
 *
 * @param[in]  ctrl4 CTRL_REG4 content.
 * @param[out] range Receives the matching range enum.
 *
 * @return ESP_OK, ESP_ERR_INVALID_ARG for NULL output, or
 *         ESP_ERR_INVALID_RESPONSE for an unknown FS value.
 */
static esp_err_t lis331_ctrl4_to_range(uint8_t ctrl4, lis331_range_t *range)
{
    if (range == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (ctrl4 & LIS331_CTRL4_FS_MASK) {
    case 0x00: *range = LIS331_RANGE_2G; break;
    case 0x10: *range = LIS331_RANGE_4G; break;
    case 0x30: *range = LIS331_RANGE_8G; break;
    default: return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

/**
 * @brief Nominal settling time for a given ODR, in milliseconds.
 *
 * Used to wait until the first valid sample after a configuration change.
 * Returns 0 for power-down, for which no data is expected.
 *
 * @param[in] odr Output data rate / power mode.
 *
 * @return Settling time in ms.
 */
static uint32_t lis331_settling_time_ms(lis331_odr_t odr)
{
    switch (odr) {
    case LIS331_ODR_NORMAL_50_HZ: return 21;
    case LIS331_ODR_NORMAL_100_HZ: return 11;
    case LIS331_ODR_NORMAL_400_HZ: return 4;
    case LIS331_ODR_NORMAL_1000_HZ: return 2;
    case LIS331_ODR_LOW_POWER_0_5_HZ: return 2001;
    case LIS331_ODR_LOW_POWER_1_HZ: return 1001;
    case LIS331_ODR_LOW_POWER_2_HZ: return 501;
    case LIS331_ODR_LOW_POWER_5_HZ: return 201;
    case LIS331_ODR_LOW_POWER_10_HZ: return 101;
    default: return 0;
    }
}

/**
 * @brief Block the calling task until the ODR has settled.
 *
 * @param[in] odr Output data rate / power mode.
 */
static void lis331_wait_for_settling(lis331_odr_t odr)
{
    uint32_t settling_time_ms = lis331_settling_time_ms(odr);
    if (settling_time_ms > 0) {
        /* Add one tick so sub-tick delays never collapse to zero. */
        vTaskDelay(pdMS_TO_TICKS(settling_time_ms) + 1);
    }
}

esp_err_t lis331_create(i2c_master_bus_handle_t bus, uint8_t device_address,
                        const lis331_config_t *config, lis331_handle_t *out_handle)
{
    if (bus == NULL || out_handle == NULL || device_address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_handle = NULL;

    lis331_config_t default_config = {
        .odr = LIS331_ODR_NORMAL_50_HZ,
        .range = LIS331_RANGE_2G,
        .block_data_update = true,
        .i2c_clock_hz = 100000,
    };
    if (config == NULL) {
        config = &default_config;
    }

    if (config->i2c_clock_hz == 0 || config->i2c_clock_hz > 400000) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t odr_bits;
    uint8_t range_bits;
    esp_err_t ret = lis331_odr_to_ctrl1(config->odr, &odr_bits);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = lis331_range_to_ctrl4(config->range, &range_bits);
    if (ret != ESP_OK) {
        return ret;
    }

    lis331_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = config->i2c_clock_hz,
    };
    ret = i2c_master_bus_add_device(bus, &device_config, &handle->i2c_dev);
    if (ret != ESP_OK) {
        free(handle);
        return ret;
    }

    uint8_t who_am_i;
    ret = lis331_get_device_id(handle, &who_am_i);
    if (ret != ESP_OK) {
        goto fail;
    }
    if (who_am_i != LIS331_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "unexpected WHO_AM_I: expected 0x%02X, got 0x%02X",
                 LIS331_WHO_AM_I_VALUE, who_am_i);
        ret = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }

    ret = lis331_write_reg(handle, LIS331_REG_CTRL1, odr_bits | LIS331_CTRL1_AXES);
    if (ret != ESP_OK) {
        goto fail;
    }
    /* Keep output parsing deterministic: BLE=0 maps L then H at ascending addresses. */
    ret = lis331_update_reg_bits(handle, LIS331_REG_CTRL4,
                                 LIS331_CTRL4_FS_MASK | LIS331_CTRL4_BDU | LIS331_CTRL4_BLE,
                                 range_bits | (config->block_data_update ? LIS331_CTRL4_BDU : 0));
    if (ret != ESP_OK) {
        goto fail;
    }

    lis331_wait_for_settling(config->odr);
    *out_handle = handle;
    return ESP_OK;

fail:
    i2c_master_bus_rm_device(handle->i2c_dev);
    free(handle);
    return ret;
}

esp_err_t lis331_delete(lis331_handle_t *handle)
{
    if (handle == NULL || *handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device((*handle)->i2c_dev);
    if (ret != ESP_OK) {
        return ret;
    }
    free(*handle);
    *handle = NULL;
    return ESP_OK;
}

esp_err_t lis331_get_device_id(lis331_handle_t handle, uint8_t *who_am_i)
{
    return lis331_read_reg(handle, LIS331_REG_WHO_AM_I, who_am_i);
}

esp_err_t lis331_set_odr(lis331_handle_t handle, lis331_odr_t odr)
{
    uint8_t ctrl1_bits;
    esp_err_t ret = lis331_odr_to_ctrl1(odr, &ctrl1_bits);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = lis331_update_reg_bits(handle, LIS331_REG_CTRL1,
                                 LIS331_CTRL1_PM_DR_MASK, ctrl1_bits);
    if (ret == ESP_OK) {
        lis331_wait_for_settling(odr);
    }
    return ret;
}

esp_err_t lis331_get_odr(lis331_handle_t handle, lis331_odr_t *odr)
{
    uint8_t ctrl1;
    esp_err_t ret = lis331_read_reg(handle, LIS331_REG_CTRL1, &ctrl1);
    if (ret != ESP_OK) {
        return ret;
    }
    return lis331_ctrl1_to_odr(ctrl1, odr);
}

esp_err_t lis331_set_range(lis331_handle_t handle, lis331_range_t range)
{
    uint8_t ctrl4_bits;
    esp_err_t ret = lis331_range_to_ctrl4(range, &ctrl4_bits);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = lis331_update_reg_bits(handle, LIS331_REG_CTRL4, LIS331_CTRL4_FS_MASK,
                                 ctrl4_bits);
    if (ret != ESP_OK) {
        return ret;
    }

    lis331_odr_t odr;
    ret = lis331_get_odr(handle, &odr);
    if (ret == ESP_OK) {
        lis331_wait_for_settling(odr);
    }
    return ret;
}

esp_err_t lis331_get_range(lis331_handle_t handle, lis331_range_t *range)
{
    uint8_t ctrl4;
    esp_err_t ret = lis331_read_reg(handle, LIS331_REG_CTRL4, &ctrl4);
    if (ret != ESP_OK) {
        return ret;
    }
    return lis331_ctrl4_to_range(ctrl4, range);
}

esp_err_t lis331_set_block_data_update(lis331_handle_t handle, bool enable)
{
    return lis331_update_reg_bits(handle, LIS331_REG_CTRL4, LIS331_CTRL4_BDU,
                                  enable ? LIS331_CTRL4_BDU : 0);
}

esp_err_t lis331_get_status(lis331_handle_t handle, uint8_t *status)
{
    return lis331_read_reg(handle, LIS331_REG_STATUS, status);
}

esp_err_t lis331_data_ready(lis331_handle_t handle, bool *ready)
{
    if (ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status;
    esp_err_t ret = lis331_get_status(handle, &status);
    if (ret != ESP_OK) {
        return ret;
    }
    *ready = (status & LIS331_STATUS_ZYXDA) != 0;
    return ESP_OK;
}

esp_err_t lis331_read_raw(lis331_handle_t handle, lis331_raw_xyz_t *raw)
{
    if (handle == NULL || raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t values[6];
    esp_err_t ret = lis331_read_regs(handle,
                                     LIS331_REG_OUT_X_L | LIS331_I2C_AUTO_INCREMENT,
                                     values, sizeof(values));
    if (ret != ESP_OK) {
        return ret;
    }

    int16_t x_register = (int16_t)((uint16_t)values[0] | ((uint16_t)values[1] << 8));
    int16_t y_register = (int16_t)((uint16_t)values[2] | ((uint16_t)values[3] << 8));
    int16_t z_register = (int16_t)((uint16_t)values[4] | ((uint16_t)values[5] << 8));
    raw->x = x_register / 16;
    raw->y = y_register / 16;
    raw->z = z_register / 16;
    return ESP_OK;
}

esp_err_t lis331_read_acceleration(lis331_handle_t handle,
                                   lis331_acceleration_t *acceleration)
{
    if (handle == NULL || acceleration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    lis331_raw_xyz_t raw;
    esp_err_t ret = lis331_read_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    lis331_range_t range;
    ret = lis331_get_range(handle, &range);
    if (ret != ESP_OK) {
        return ret;
    }

    float sensitivity_mg;
    switch (range) {
    case LIS331_RANGE_2G: sensitivity_mg = 1.0f; break;
    case LIS331_RANGE_4G: sensitivity_mg = 2.0f; break;
    case LIS331_RANGE_8G: sensitivity_mg = 3.9f; break;
    default: return ESP_ERR_INVALID_RESPONSE;
    }

    float scale = sensitivity_mg * LIS331_GRAVITY_M_S2 / 1000.0f;
    acceleration->x = raw.x * scale;
    acceleration->y = raw.y * scale;
    acceleration->z = raw.z * scale;
    return ESP_OK;
}
