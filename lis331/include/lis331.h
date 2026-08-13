#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIS331_I2C_ADDR_SA0_GND  0x18
#define LIS331_I2C_ADDR_SA0_VDD  0x19
#define LIS331_WHO_AM_I_VALUE    0x32

typedef struct lis331_dev_t *lis331_handle_t;

typedef enum {
    LIS331_ODR_POWER_DOWN,
    LIS331_ODR_NORMAL_50_HZ,
    LIS331_ODR_NORMAL_100_HZ,
    LIS331_ODR_NORMAL_400_HZ,
    LIS331_ODR_NORMAL_1000_HZ,
    LIS331_ODR_LOW_POWER_0_5_HZ,
    LIS331_ODR_LOW_POWER_1_HZ,
    LIS331_ODR_LOW_POWER_2_HZ,
    LIS331_ODR_LOW_POWER_5_HZ,
    LIS331_ODR_LOW_POWER_10_HZ,
} lis331_odr_t;

typedef enum {
    LIS331_RANGE_2G,
    LIS331_RANGE_4G,
    LIS331_RANGE_8G,
} lis331_range_t;

/* Normalized signed 12-bit output counts, not left-justified register words. */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lis331_raw_xyz_t;

/* Acceleration in m/s2. */
typedef struct {
    float x;
    float y;
    float z;
} lis331_acceleration_t;

typedef struct {
    lis331_odr_t odr;
    lis331_range_t range;
    bool block_data_update;
    uint32_t i2c_clock_hz;
} lis331_config_t;

/*
 * Creates a device on an application-owned I2C master bus.
 * device_address is a 7-bit address, normally LIS331_I2C_ADDR_SA0_GND or
 * LIS331_I2C_ADDR_SA0_VDD. A NULL config selects 100 kHz I2C, normal 50 Hz,
 * +/-2g, and BDU on. i2c_clock_hz must not exceed the LIS331DLH 400 kHz limit.
 */
esp_err_t lis331_create(i2c_master_bus_handle_t bus,
                        uint8_t device_address,
                        const lis331_config_t *config,
                        lis331_handle_t *out_handle);

/* Removes only the device handle; the caller owns the I2C bus lifetime. */
esp_err_t lis331_delete(lis331_handle_t *handle);

esp_err_t lis331_get_device_id(lis331_handle_t handle, uint8_t *who_am_i);

esp_err_t lis331_set_odr(lis331_handle_t handle, lis331_odr_t odr);
esp_err_t lis331_get_odr(lis331_handle_t handle, lis331_odr_t *odr);

esp_err_t lis331_set_range(lis331_handle_t handle, lis331_range_t range);
esp_err_t lis331_get_range(lis331_handle_t handle, lis331_range_t *range);

esp_err_t lis331_set_block_data_update(lis331_handle_t handle, bool enable);
esp_err_t lis331_get_status(lis331_handle_t handle, uint8_t *status);
esp_err_t lis331_data_ready(lis331_handle_t handle, bool *ready);

esp_err_t lis331_read_raw(lis331_handle_t handle, lis331_raw_xyz_t *raw);
esp_err_t lis331_read_acceleration(lis331_handle_t handle,
                                   lis331_acceleration_t *acceleration);

#ifdef __cplusplus
}
#endif
