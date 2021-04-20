set(project_srcs "src/flatpack2.c" "main.c")
set(extra_compile_args "-Wno-unused-variable" "-Wno-unused-function")

# custom theme...
list(APPEND project_srcs "src/lv_theme_fp2.c")

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

if(CONFIG_WEB_DEPLOY_FLASH)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../web")
    if(EXISTS ${WEB_SRC_DIR}/dist)
        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
    else()
        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
    endif()
endif()