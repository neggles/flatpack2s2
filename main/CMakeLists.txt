set(project_srcs "src/helpers.c" "main.c")

idf_component_register(
    SRCS "${project_srcs}"
    INCLUDE_DIRS "src"
    REQUIRED_IDF_TARGETS "esp32s2"
    )

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -Wno-unused-function)
