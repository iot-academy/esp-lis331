#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lis331.h"

#define LIS331_REG_WHO_AM_I  0x0F
#define LIS331_REG_CTRL1     0x20
#define LIS331_REG_CTRL4     0x23
#define LIS331_REG_STATUS    0x27
#define LIS331_REG_OUT_X_L   0x28

#define LIS331_CTRL1_PM_DR_MASK  0xF8
#define LIS331_CTRL1_AXES         0x07
#define LIS331_CTRL4_BDU          0x80
#define LIS331_CTRL4_BLE          0x40
#define LIS331_CTRL4_FS_MASK      0x30
#define LIS331_STATUS_ZYXDA       0x08
#define LIS331_I2C_AUTO_INCREMENT 0x80
#define LIS331_I2C_TIMEOUT_MS     1000
#define LIS331_GRAVITY_M_S2       9.80665f

struct lis331_dev_t {
    i2c_master_dev_handle_t i2c_dev;
};

static const char *TAG = "lis331";

static esp_err_t lis331_read_reg(lis331_handle_t handle, uint8_t reg, uint8_t *value)
{
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, value, 1,
                                       LIS331_I2C_TIMEOUT_MS);
}

static esp_err_t lis331_read_regs(lis331_handle_t handle, uint8_t reg, uint8_t *values,
                                  size_t length)
{
    if (handle == NULL || values == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(handle->i2c_dev, &reg, 1, values, length,
                                       LIS331_I2C_TIMEOUT_MS);
}

static esp_err_t lis331_write_reg(lis331_handle_t handle, uint8_t reg, uint8_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t transaction[] = {reg, value};
    return i2c_master_transmit(handle->i2c_dev, transaction, sizeof(transaction),
                               LIS331_I2C_TIMEOUT_MS);
}

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
