#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "lis331.h"

#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ  100000

void app_main(void)
{
    i2c_config_t bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus_handle_t bus = i2c_bus_create(I2C_NUM_0, &bus_config);
    if (bus == NULL) {
        ESP_LOGE("lis331_example", "failed to create I2C bus");
        return;
    }

    lis331_config_t config = {
        .odr = LIS331_ODR_NORMAL_50_HZ,
        .range = LIS331_RANGE_2G,
        .block_data_update = true,
        .i2c_clock_hz = I2C_MASTER_FREQ_HZ,
    };
    lis331_handle_t sensor;
    ESP_ERROR_CHECK(lis331_create_i2c_bus(bus, LIS331_I2C_ADDR_SA0_GND, &config, &sensor));

    uint8_t who_am_i;
    ESP_ERROR_CHECK(lis331_get_device_id(sensor, &who_am_i));
    printf("LIS331DLH detected, WHO_AM_I=0x%02X\n", who_am_i);

    while (true) {
        bool ready;
        ESP_ERROR_CHECK(lis331_data_ready(sensor, &ready));
        if (ready) {
            lis331_acceleration_t acceleration;
            ESP_ERROR_CHECK(lis331_read_acceleration(sensor, &acceleration));
            printf("Acceleration [m/s2]: X=% .3f Y=% .3f Z=% .3f\n",
                   acceleration.x, acceleration.y, acceleration.z);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}