# Changelog

## 1.0.0 — 2026-08-22

Stable release.

- Component is fully standalone: drop the `cmake_utilities` dependency and
  the `cu_pkg_define_version()` macro (generated `LIS331_VER_*` macros were
  unused). `cmake_utilities` is still pulled transitively by
  `espressif/i2c_bus` when enabled.
- Pin the optional `espressif/i2c_bus` dependency to `^1.0.0` instead of `*`.
- Declare all ESP-IDF targets.
- Add SPDX-License-Identifier and copyright notice to source files.
- Remove Arduino-related phrasing from documentation.

## 0.0.2 — 2026-08-22

- Declare all ESP-IDF targets (esp32, esp32c2, esp32c3, esp32c5, esp32c61,
  esp32c6, esp32h2, esp32p4, esp32s2, esp32s3).
- Add SPDX-License-Identifier and copyright notice to source files.

## 0.0.1 — 2026-08-22

Initial release.

- Native ESP-IDF I2C master driver transport (`lis331_create`)
- Optional ESP-IoT-Solution i2c_bus transport (`lis331_create_i2c_bus`)
- Full-scale ranges 2 g / 4 g / 8 g
- Output data rates in normal and low-power modes
- Block data update (BDU)
- Raw (signed 12-bit) and SI (m/s²) acceleration reads
- Data-ready status via STATUS_REG (ZYXDA flag)
- WHO_AM_I validation at device creation (expected 0x32)