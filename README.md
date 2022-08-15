# HTS221 I2C Driver Implementation for the ESP32

An I2C driver for the HTS221 Humidity Sensor used on the Raspberry Pi Sense-HAT boards.

This implementation is for connecting the HTS221 on the Sense-HAT board to an ESP-32-C3 microcontroller.

The pins used were as follows:

---------------

GPIO0 => SDA

GPIO1 => SCL

GND => GND

3V3 => 3v3

---------------

NOTE: The 5v pins were not used as the temperature, pressure, humidity, accelerometer, and magnetometer sensors did not use 3v.

The GPIO/I2C pin configuration can be found in hts221_i2c_main.c

All I2C registers are documented in hts221_i2c.h

  

## Requirements
 1. ESP32 (I specifically tested the ESP32-C3 board. For other ESP32 boards, YMMV)
 2. [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html) with your favorite code editor/IDE (I used the current master. YMMV on older versions)
 3. Raspberry Pi Sense-HAT (YMMV on other boards using the HTS221)
 4. No-solder breadboard (Optional, but highly useful)
 5. Male-to-Male Wires
## Build/Flash/Monitor
The following snippets assume that you have already  setup the ESP-IDF, opened a terminal/command prompt that has the ESP-IDF environment variables set, and configured the project to use the ESP32-C3. `<ESP32 PORT>` is a device that looks like `/dev/ttyUSB0` in Linux, `/dev/cu.0` in MacOS, and `COM0` in Windows.
### To build the project <ins>without</ins> flashing the ESP32:
    $ idf.py build
### To build the project <ins>and</ins> flash the ESP32 <ins>without</ins> monitoring the ESP32's serial output:
    $ idf.py -p <ESP32 PORT> flash
To see the serial output on the board at a later point:

    $ idf.py -p <ESP32 PORT> monitor

### To build, flash, <ins>and</ins> monitor the program's serial output on the ESP32:

    $ idf.py -p <ESP32 PORT> flash monitor

For more information, please visit the ESP-IDF's [Getting Started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html) page.
