# UVc Sensor Data Logger

Code for a custom UVc (ultra-violet c wavelength) data logger constructed from
an [ESP32][1] and a [Mikroe UVc-Click module][2]. Sensor readings are broadcast
via an [MQTT][3] broker.

## Hardware
The reference hardware consists of:
 * An [Espressif ESP32-S2-Saola-1][4] development board
 * A [Mikroe UVc-Click][2] module
 * A [DFRobot LCD1602][5] LCD display module

The devices are connected on the I2C bus.

## Dependencies
 * [Adafruit MQTT Library][6]
 * [DF Robot LCD master][7]

## Compiling
Either the [Arduino IDE][8] or [Arduino CLI][9] can be used for compiling and
uploading to the ESP32. `uvc-sensor/config.h.example` should be copied to
`uvc-sensor/config.h` and updated with the required details before compilation.

## Acknowledgements
This work was developed as part of the the [Trustworthy Human Robot Teams project][10],
 supported by the [UK Engineering and Physical Sciences Research Council][11]
(EPSRC) through the [Trustworthy Autonomous Systems Hub][12] ([EP/V00784X/1][13]).

[1]: https://www.espressif.com/en/products/socs/esp32
[2]: https://www.mikroe.com/uvc-click
[3]: https://mqtt.org/
[4]: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/hw-reference/esp32s2/user-guide-saola-1-v1.2.html
[5]: https://www.dfrobot.com/product-1724.html
[6]: https://github.com/adafruit/Adafruit_MQTT_Library
[7]: https://github.com/bearwaterfall/DFRobot_LCD-master
[8]: https://www.arduino.cc/en/software
[9]: https://www.arduino.cc/pro/cli
[10]: https://www.tas.ac.uk/current-research-projects/trustworthy-human-robot-teams/
[11]: https://www.ukri.org/councils/epsrc/
[12]: https://www.tas.ac.uk/
[13]: https://gow.epsrc.ukri.org/NGBOViewGrant.aspx?GrantRef=EP/V00784X/1
