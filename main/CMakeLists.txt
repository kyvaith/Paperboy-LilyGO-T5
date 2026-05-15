idf_component_register(
    SRCS "main.c" "msg/msg.c" "peanut_gb/paperboy_gb.c"
    INCLUDE_DIRS "." "msg" "peanut_gb"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd
)

if ((IDF_VERSION_MAJOR GREATER_EQUAL 5) AND (${IDF_TARGET} MATCHES "^esp32$|esp32s2|esp32s3"))
    target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables -ftree-switch-conversion)
endif ()
