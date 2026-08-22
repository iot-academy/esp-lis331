# LIS331DLH — ESP-IDF Driver Component

ESP-IDF component driver for the STMicroelectronics LIS331DLH three-axis MEMS
accelerometer. Uses the new I2C master API (`driver/i2c_master.h`). No Arduino
dependencies.

All register semantics follow the official LIS331DLH datasheet.

## Quick start

```c
#include "lis331.h"

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

## Transport backends

- **Native** (`lis331_create`) — always available, uses `driver/i2c_master.h`.
- **ESP-IoT-Solution i2c_bus** (`lis331_create_i2c_bus`) — enabled by
  `CONFIG_LIS331_USE_I2C_BUS=y`. Pulls `espressif/i2c_bus` from the registry.

## API

| Function | Description |
|----------|-------------|
| `lis331_create` | Create device on native I2C master bus |
| `lis331_create_i2c_bus` | Create device on i2c_bus handle |
| `lis331_delete` | Remove device, free handle |
| `lis331_get_device_id` | Read WHO_AM_I |
| `lis331_set_odr` / `lis331_get_odr` | Set/get output data rate |
| `lis331_set_range` / `lis331_get_range` | Set/get full-scale (2/4/8 g) |
| `lis331_set_block_data_update` | Enable/disable BDU |
| `lis331_get_status` | Read raw STATUS_REG |
| `lis331_data_ready` | Check ZYXDA flag |
| `lis331_read_raw` | Normalised signed 12-bit X/Y/Z |
| `lis331_read_acceleration` | X/Y/Z in m/s² |

## Dependencies

- `idf` >= 5.3.0
- `espressif/cmake_utilities` 0.*
- `espressif/i2c_bus` (conditional, see above)

## License

MIT — see `LICENSE` file.