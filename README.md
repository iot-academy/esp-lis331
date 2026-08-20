# LIS331DLH ESP-IDF Driver

Production-oriented I2C driver for the STMicroelectronics LIS331DLH three-axis
accelerometer, written for ESP-IDF 5.x/6.x using the new I2C master API
(`driver/i2c_master.h`). No Arduino dependencies.

All register semantics follow the LIS331DLH datasheet. No functionality from
similar ST sensors (LIS3DH, LIS331HH, ...) is assumed.

## Features

- Native ESP-IDF I2C master driver (`lis331_create()`, always available);
- optional ESP-IoT-Solution `i2c_bus` transport (`lis331_create_i2c_bus()`),
  enabled with `CONFIG_LIS331_USE_I2C_BUS=y`;
- device identification via `WHO_AM_I` (0x32) at creation;
- full-scale ranges 2 g / 4 g / 8 g;
- output data rates in normal (50/100/400/1000 Hz) and low-power
  (0.5/1/2/5/10 Hz) modes;
- block data update (BDU);
- raw (normalized signed 12-bit) and SI (m/s^2) acceleration reads;
- data-ready status via `STATUS_REG`.

Not implemented in this revision: interrupts, embedded self-test, high-pass
filter, sleep-to-wake, 6D orientation, SPI, click/tap.

## Hardware connection

- SDA -> GPIO21, SCL -> GPIO22 (defaults in the examples).
- ESP32 internal pull-ups are disabled by the examples, so external pull-up
  resistors are required on both I2C lines.
- The 7-bit I2C address is selected by the SA0 pin:
  `0x18` (SA0 tied to GND) or `0x19` (SA0 tied to VDD).
- SCL clock must stay within 1..400 kHz.

## Example usage

```c
#include "driver/i2c_master.h"
#include "lis331.h"

i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};
i2c_master_bus_handle_t bus;
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

lis331_config_t config = {
    .odr = LIS331_ODR_NORMAL_50_HZ,
    .range = LIS331_RANGE_2G,
    .block_data_update = true,
    .i2c_clock_hz = 100000,
};
lis331_handle_t sensor;
ESP_ERROR_CHECK(lis331_create(bus, LIS331_I2C_ADDR_SA0_GND, &config, &sensor));

lis331_acceleration_t a;
ESP_ERROR_CHECK(lis331_read_acceleration(sensor, &a));
```

## Data interpretation

- `lis331_read_raw()` returns normalized signed 12-bit counts in
  `[-2048, 2047]`: the left-justified 16-bit register words are right-shifted
  by 4 bits (register value / 16).
- `lis331_read_acceleration()` returns X/Y/Z in m/s^2. One count equals
  `2*FS/2048 * 9.80665` m/s^2, where FS is the configured range in g
  (2, 4 or 8). At 1 g the output is exactly 9.80665 m/s^2.

## API overview

| Function                    | Description                                       |
| --------------------------- | ------------------------------------------------- |
| `lis331_create`             | Create device on a native I2C master bus          |
| `lis331_create_i2c_bus`     | Create device on an ESP-IoT-Solution i2c_bus      |
| `lis331_delete`             | Remove device and free the handle                 |
| `lis331_get_device_id`      | Read `WHO_AM_I`                                   |
| `lis331_set_odr`            | Set output data rate / power mode                 |
| `lis331_get_odr`            | Get current ODR / power mode                      |
| `lis331_set_range`          | Set full-scale range (2/4/8 g)                    |
| `lis331_get_range`          | Get current range                                 |
| `lis331_set_block_data_update` | Enable/disable BDU                             |
| `lis331_get_status`         | Read raw `STATUS_REG`                             |
| `lis331_data_ready`         | Check the ZYXDA data-ready flag                   |
| `lis331_read_raw`           | Read normalized signed 12-bit X/Y/Z               |
| `lis331_read_acceleration`  | Read X/Y/Z acceleration in m/s^2                  |

Passing `NULL` as `config` to `lis331_create()`/`lis331_create_i2c_bus()`
selects documented defaults: 100 kHz I2C, normal mode 50 Hz, +/-2 g, BDU on.

## Examples

Both examples build with `idf.py build` from their directories after
exporting an ESP-IDF environment:

- `examples/basic` — native ESP-IDF I2C master driver;
- `examples/i2c_bus` — ESP-IoT-Solution `i2c_bus` transport
  (requires `CONFIG_LIS331_USE_I2C_BUS=y`, the dependency is pulled
  automatically from the component registry).

## Dependencies

- `espressif/cmake_utilities` (always);
- `espressif/i2c_bus` (conditional on `CONFIG_LIS331_USE_I2C_BUS`, pulled
  from the component registry when enabled).

## License

This project is licensed under the MIT License — see the `LICENSE` file.