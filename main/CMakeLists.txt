idf_component_register(
    SRCS "src/helper_funcs.c" "main.c"
    REQUIRED_IDF_TARGETS "esp32s2"
    INCLUDE_DIRS "src"
)
