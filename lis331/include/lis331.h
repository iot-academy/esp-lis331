/**
 * @file lis331.h
 * @brief LIS331DLH three-axis accelerometer driver for ESP-IDF.
 *
 * Production-oriented driver for the ST LIS331DLH MEMS accelerometer using
 * the ESP-IDF I2C interface. All register semantics follow the LIS331DLH
 * datasheet (see docs/); no functionality from similar ST sensors is assumed.
 *
 * Transport selection (selected by the component dependency in
 * idf_component.yml):
 *   - native ESP-IDF I2C master driver (`driver/i2c_master.h`) - this branch
 *     (`main`);
 *   - ESP-IoT-Solution `i2c_bus` wrapper - branch `compat/i2c-bus`.
 *
 * Addresses are 7-bit (no R/W bit). The address is selected by the SA0 pin:
 * LIS331_I2C_ADDR_SA0_GND or LIS331_I2C_ADDR_SA0_VDD.
 *
 * Supported features:
 *   - device identification via WHO_AM_I (0x32) during creation;
 *   - full-scale ranges 2 g / 4 g / 8 g;
 *   - output data rates in normal and low-power modes;
 *   - block data update (BDU);
 *   - raw (normalized signed 12-bit) and SI (m/s^2) acceleration reads;
 *   - data-ready status via STATUS_REG.
 *
 * Not implemented in this revision: interrupts, embedded self-test, high-pass
 * filter, sleep-to-wake, 6D orientation and SPI.
 *
 * @author Eugene A. Kanashev
 * @date 2026
 *
 * @defgroup lis331 LIS331DLH accelerometer driver
 * @{
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 7-bit I2C address when SA0 is tied to GND. */
#define LIS331_I2C_ADDR_SA0_GND  0x18
/** @brief 7-bit I2C address when SA0 is tied to VDD. */
#define LIS331_I2C_ADDR_SA0_VDD  0x19
/** @brief Expected content of the WHO_AM_I register (0x0F). */
#define LIS331_WHO_AM_I_VALUE    0x32

/**
 * @brief Opaque driver handle.
 *
 * Created by lis331_create(), released by lis331_delete(). The underlying
 * I2C bus remains owned by the application.
 */
typedef struct lis331_dev_t *lis331_handle_t;

/**
 * @brief Output data rate / power mode configuration.
 *
 * Maps to the PM[2:0] and DR[2:0] fields of CTRL_REG1.
 */
typedef enum {
    LIS331_ODR_POWER_DOWN,        /**< Power-down mode, axes disabled. */
    LIS331_ODR_NORMAL_50_HZ,      /**< Normal mode, 50 Hz. */
    LIS331_ODR_NORMAL_100_HZ,     /**< Normal mode, 100 Hz. */
    LIS331_ODR_NORMAL_400_HZ,     /**< Normal mode, 400 Hz. */
    LIS331_ODR_NORMAL_1000_HZ,    /**< Normal mode, 1000 Hz. */
    LIS331_ODR_LOW_POWER_0_5_HZ,  /**< Low-power mode, 0.5 Hz. */
    LIS331_ODR_LOW_POWER_1_HZ,    /**< Low-power mode, 1 Hz. */
    LIS331_ODR_LOW_POWER_2_HZ,    /**< Low-power mode, 2 Hz. */
    LIS331_ODR_LOW_POWER_5_HZ,    /**< Low-power mode, 5 Hz. */
    LIS331_ODR_LOW_POWER_10_HZ,   /**< Low-power mode, 10 Hz. */
} lis331_odr_t;

/**
 * @brief Full-scale range selection (CTRL_REG4.FS[1:0]).
 *
 * The acceleration scale is derived from the full scale as 2*FS/4096 g per
 * 12-bit count (AN2847, section 2.4.3; datasheet self-test note). For the
 * three ranges this corresponds to 0.977 / 1.953 / 3.906 mg per count.
 */
typedef enum {
    LIS331_RANGE_2G, /**< +/-2 g full scale; 0.977 mg/digit (12-bit). */
    LIS331_RANGE_4G, /**< +/-4 g full scale; 1.953 mg/digit (12-bit). */
    LIS331_RANGE_8G, /**< +/-8 g full scale; 3.906 mg/digit (12-bit). */
} lis331_range_t;

/**
 * @brief Raw acceleration output.
 *
 * Normalized signed 12-bit counts in the range [-2048, 2047], obtained by
 * right-shifting the left-justified 16-bit register words by 4 bits
 * (register value / 16). The output registers are read in the order
 * OUT_X_L, OUT_X_H, OUT_Y_L, OUT_Y_H, OUT_Z_L, OUT_Z_H with BLE = 0.
 */
typedef struct {
    int16_t x; /**< X axis, normalized signed 12-bit counts. */
    int16_t y; /**< Y axis, normalized signed 12-bit counts. */
    int16_t z; /**< Z axis, normalized signed 12-bit counts. */
} lis331_raw_xyz_t;

/**
 * @brief Acceleration output in SI units.
 *
 * Values are in m/s^2. One raw count corresponds to
 * 2*FS/2048 * 9.80665 m/s^2, where FS is the configured full-scale range
 * in g (2, 4 or 8). At 1 g the output is exactly 9.80665 m/s^2.
 */
typedef struct {
    float x; /**< X axis acceleration, m/s^2. */
    float y; /**< Y axis acceleration, m/s^2. */
    float z; /**< Z axis acceleration, m/s^2. */
} lis331_acceleration_t;

/**
 * @brief Creation-time configuration for lis331_create().
 *
 * All fields are used as-is; there is no hidden configuration. Passing NULL
 * to lis331_create() selects documented defaults instead.
 */
typedef struct {
    lis331_odr_t odr;       /**< Output data rate / power mode. */
    lis331_range_t range;   /**< Full-scale range. */
    bool block_data_update; /**< Enable block data update (BDU). */
    uint32_t i2c_clock_hz;  /**< I2C SCL clock in Hz, 1..400 kHz (datasheet limit). */
} lis331_config_t;

/**
 * @brief Create and initialize a LIS331DLH device.
 *
 * Performs device creation on the bus, WHO_AM_I validation, CTRL_REG1 /
 * CTRL_REG4 configuration and a settling delay for the selected ODR. The bus
 * itself is not modified or taken over.
 *
 * @param[in]  bus             I2C master bus handle, owned by the application.
 *                             The driver only adds a device to it and never
 *                             deletes the bus.
 * @param[in]  device_address  7-bit I2C address, normally
 *                             LIS331_I2C_ADDR_SA0_GND or LIS331_I2C_ADDR_SA0_VDD.
 * @param[in]  config          Optional configuration; NULL selects defaults
 *                             (100 kHz I2C, normal mode 50 Hz, +/-2 g, BDU on).
 * @param[out] out_handle      Receives the driver handle on success.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments, device_address > 0x7F,
 *         invalid ODR/range, or i2c_clock_hz outside 1..400 kHz.
 * @return ESP_ERR_NO_MEM if the driver handle cannot be allocated.
 * @return ESP_ERR_INVALID_RESPONSE if WHO_AM_I does not match 0x32.
 * @return ESP_FAIL if the device cannot be added to the bus.
 * @return any I2C transport error during register access.
 */
esp_err_t lis331_create(i2c_master_bus_handle_t bus,
                        uint8_t device_address,
                        const lis331_config_t *config,
                        lis331_handle_t *out_handle);

/**
 * @brief Delete a device created by lis331_create().
 *
 * Removes the device from the bus and frees the driver handle. The underlying
 * I2C bus is left untouched.
 *
 * @param[in,out] handle Pointer to the driver handle; set to NULL on success.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if handle is NULL or *handle is NULL.
 * @return any I2C transport error.
 */
esp_err_t lis331_delete(lis331_handle_t *handle);

/**
 * @brief Read the WHO_AM_I register.
 *
 * @param[in]  handle    Driver handle.
 * @param[out] who_am_i  Receives the register content (expected 0x32).
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return any I2C transport error.
 */
esp_err_t lis331_get_device_id(lis331_handle_t handle, uint8_t *who_am_i);

/**
 * @brief Set the output data rate / power mode.
 *
 * Applies the PM[2:0] and DR[2:0] fields of CTRL_REG1 and waits for the new
 * data rate to settle before returning. The axes-enable bits are preserved.
 *
 * @param[in] handle Driver handle.
 * @param[in] odr    New ODR / power mode.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL handle or invalid odr.
 * @return any I2C transport error.
 */
esp_err_t lis331_set_odr(lis331_handle_t handle, lis331_odr_t odr);

/**
 * @brief Get the current output data rate / power mode.
 *
 * @param[in]  handle Driver handle.
 * @param[out] odr    Receives the current ODR / power mode.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return ESP_ERR_INVALID_RESPONSE if CTRL_REG1 contains an unknown PM/DR value.
 * @return any I2C transport error.
 */
esp_err_t lis331_get_odr(lis331_handle_t handle, lis331_odr_t *odr);

/**
 * @brief Set the full-scale range.
 *
 * Applies the FS[1:0] field of CTRL_REG4 and waits for the current data rate
 * to settle. BDU and BLE bits are preserved.
 *
 * @param[in] handle Driver handle.
 * @param[in] range  New full-scale range.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL handle or invalid range.
 * @return any I2C transport error.
 */
esp_err_t lis331_set_range(lis331_handle_t handle, lis331_range_t range);

/**
 * @brief Get the current full-scale range.
 *
 * @param[in]  handle Driver handle.
 * @param[out] range  Receives the current range.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return ESP_ERR_INVALID_RESPONSE if CTRL_REG4 contains an unknown FS value.
 * @return any I2C transport error.
 */
esp_err_t lis331_get_range(lis331_handle_t handle, lis331_range_t *range);

/**
 * @brief Enable or disable block data update (BDU).
 *
 * @param[in] handle Driver handle.
 * @param[in] enable true to enable BDU, false to disable.
 *
 * @return ESP_OK on success.
 * @return any I2C transport error.
 */
esp_err_t lis331_set_block_data_update(lis331_handle_t handle, bool enable);

/**
 * @brief Read the raw STATUS_REG content.
 *
 * @param[in]  handle Driver handle.
 * @param[out] status Receives the STATUS_REG value (see the LIS331DLH
 *                    datasheet for the bit definitions).
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return any I2C transport error.
 */
esp_err_t lis331_get_status(lis331_handle_t handle, uint8_t *status);

/**
 * @brief Check whether new data is available for all axes.
 *
 * Reads STATUS_REG and reports the ZYXDA flag (data-ready for X, Y and Z).
 * The flag is set after each data update and cleared when the output
 * registers are read.
 *
 * @param[in]  handle Driver handle.
 * @param[out] ready  Receives true if new data is available.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return any I2C transport error.
 */
esp_err_t lis331_data_ready(lis331_handle_t handle, bool *ready);

/**
 * @brief Read raw acceleration for all axes.
 *
 * Reads OUT_X_L..OUT_Z_H (6 bytes) in a single burst transaction using the
 * auto-increment sub-address and normalizes the left-justified 12-bit words.
 *
 * @param[in]  handle Driver handle.
 * @param[out] raw    Receives normalized signed 12-bit X/Y/Z counts.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return any I2C transport error.
 */
esp_err_t lis331_read_raw(lis331_handle_t handle, lis331_raw_xyz_t *raw);

/**
 * @brief Read acceleration in m/s^2.
 *
 * Reads raw counts and converts them to SI units using the current range:
 * a = raw_count * 2*FS/2048 * 9.80665, where FS is the full-scale range in g.
 *
 * @param[in]  handle       Driver handle.
 * @param[out] acceleration Receives X/Y/Z acceleration in m/s^2.
 *
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG on NULL arguments.
 * @return ESP_ERR_INVALID_RESPONSE if the stored range is invalid.
 * @return any I2C transport error.
 */
esp_err_t lis331_read_acceleration(lis331_handle_t handle,
                                   lis331_acceleration_t *acceleration);

/** @} */

#ifdef __cplusplus
}
#endif
