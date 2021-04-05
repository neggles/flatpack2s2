
# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(PROJECT_VER "0.1.0")
set(EXCLUDE_COMPONENTS "coap" "asio" "freemodbus" "bt" "sdmmc" "ulp" "esp_serial_slave_link" "esp_eth" "esp_hid")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flatpack2s2)
