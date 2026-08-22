# Changelog

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