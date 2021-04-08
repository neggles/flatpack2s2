set(project_srcs "src/flatpack2.c" "main.c")
set(extra_compile_args "-Wno-unused-variable" "-Wno-unused-function")

# debug!
#list(APPEND project_srcs "src/debug.c")

# gcc8 doesn't know about #pragma region
list(APPEND extra_compile_args "-Wno-unknown-pragmas")

# comment out to set local log level to config level
list(APPEND extra_compile_args "-D LOG_LOCAL_LEVEL=ESP_LOG_DEBUG")

idf_component_register(
    SRCS "${project_srcs}"
    INCLUDE_DIRS "src"
    REQUIRED_IDF_TARGETS "esp32s2"
    )

target_compile_options(${COMPONENT_LIB} PRIVATE ${extra_compile_args})
