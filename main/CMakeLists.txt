idf_component_register(
    SRCS "src/helper_funcs.c" "main.c"
    REQUIRED_IDF_TARGETS "esp32s2"
    INCLUDE_DIRS "src"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -Wno-unused-function)
