idf_component_register(SRCS "lvgl_gpiodev.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver"
                    REQUIRES "lvgl" "lvgl_esp32_drivers")
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLV_LVGL_H_INCLUDE_SIMPLE")
