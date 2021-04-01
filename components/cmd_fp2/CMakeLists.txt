idf_component_register(SRCS "cmd_fp2.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver"
                    REQUIRES console "nvs_flash")
