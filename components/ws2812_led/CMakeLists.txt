set(srcs "ws2812_led.c" "led_strip_rmt_ws2812.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES "driver")
