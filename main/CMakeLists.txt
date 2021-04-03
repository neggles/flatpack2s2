set(project_srcs "src/helpers.c" "src/flatpack2.c" "main.c")

#set(extra_compile_args "-Wno-unused-variable" "-Wno-unused-function" "-D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")
set(extra_compile_args "-Wno-unused-variable" "-Wno-unused-function")


idf_component_register(
    SRCS "${project_srcs}"
    INCLUDE_DIRS "src"
    REQUIRED_IDF_TARGETS "esp32s2"
    )

target_compile_options(${COMPONENT_LIB} PRIVATE ${extra_compile_args})
