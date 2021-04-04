# flatpack2s2

[![esp-idf-build](https://github.com/neg2led/flatpack2s2/actions/workflows/esp-idf-build.yml/badge.svg?branch=master)](https://github.com/neg2led/flatpack2s2/actions/workflows/esp-idf-build.yml)

application code for my [esp32-cantroller](https://github.com/neg2led/esp32-cantroller) board, to control an Eltek Flatpack2 rectifier/power supply module.

more documentation to come later on, once the code is something vaguely resembling stable/feature-complete.

uses [ESP-IDF](https://github.com/espressif/esp-idf) master (currently v4.4-not-even-a-beta-yet), because I'm using the ESP32-S2's USB-CDC console mode & support for that is *very* new and fairly buggy even in v4.3-beta.

### many thanks to these projects:
- [lvgl](https://github.com/lvgl/lvgl) and [lv_port_esp32](https://github.com/lvgl/lv_port_esp32)

my own code is licensed under the GNU GPL v3 as per [LICENSE](LICENSE.txt) - others' code is licensed as detailed in their source files and/or repositories.