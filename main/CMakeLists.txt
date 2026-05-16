idf_component_register(
    SRCS "main.c" "ui.c" "msg/msg.c" "gbemu.c" "audio.c" "minigb_apu/minigb_apu.c" "touch.c"
    INCLUDE_DIRS "." "msg" "minigb_apu"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd esp_driver_ledc esp_driver_i2s esp_driver_i2c
)

# Audio APU format: 16-bit signed integer
target_compile_definitions(${COMPONENT_LIB} PRIVATE MINIGB_APU_AUDIO_FORMAT_S16SYS)

if ((IDF_VERSION_MAJOR GREATER_EQUAL 5) AND (${IDF_TARGET} MATCHES "^esp32$|esp32s2|esp32s3"))
    target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables -ftree-switch-conversion)
endif ()
