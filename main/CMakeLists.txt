idf_component_register(
    SRCS "main.c" "ui.c" "msg/msg.c" "gbemu.c" "audio.c" "minigb_apu/minigb_apu.c" "touch.c" "profiler.c"
    INCLUDE_DIRS "." "msg" "minigb_apu"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd esp_driver_ledc esp_driver_i2s esp_driver_i2c
)

# Audio APU format: 16-bit signed integer
target_compile_definitions(${COMPONENT_LIB} PRIVATE MINIGB_APU_AUDIO_FORMAT_S16SYS)

if ((IDF_VERSION_MAJOR GREATER_EQUAL 5) AND (${IDF_TARGET} MATCHES "^esp32$|esp32s2|esp32s3"))
    target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables -ftree-switch-conversion)
endif ()

# Compile the emulator translation unit at -O3.
# walnut_cgb.h is included here and contains the entire SM83 CPU decoder;
# the extra optimisation (better register allocation, unrolling) typically
# saves 5–15% on the hot loop.
set_source_files_properties(gbemu.c PROPERTIES COMPILE_FLAGS "-O3")
