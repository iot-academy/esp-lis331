# LIS331DLH Basic Example

The example creates an ESP-IDF I2C master bus, verifies the LIS331DLH identity,
and prints coherent XYZ acceleration samples in m/s2.

The default wiring is SDA=GPIO21 and SCL=GPIO22. The example disables ESP32
internal pull-ups, so the I2C bus requires external pull-up resistors. The
sensor address defaults to SA0 tied low (`0x18`).

Build it from this directory after exporting an ESP-IDF environment:

```sh
. ~/.espressif/v6.0.2/esp-idf/export.sh
idf.py set-target esp32
idf.py build
```
