# Select the PCM audio backend by changing PAPERBOY_AUDIO_PCM_IMPL:
#   audio_pdm.c  — I2S PDM TX       (default; accurate PCM, quietest)
#   audio_pwm.c  — LEDC PCM via ISR (accurate PCM, louder)
#
# The polyphonic backend (audio_poly_pwm.c) is always compiled in.
# Runtime selection between PCM, Polyphony, and Mute is done in the game
# selection menu via the "Snd:" item.
#
# Usage:  cmake -DPAPERBOY_AUDIO_PCM_IMPL=audio_pwm.c  (or edit below)
set(PAPERBOY_AUDIO_PCM_IMPL "audio_pdm.c" CACHE STRING
    "PCM audio backend source file (audio_pdm.c or audio_pwm.c)")

idf_component_register(
    SRCS "main.c" "ui.c" "msg/msg.c" "gbemu.c" "audio.c" "${PAPERBOY_AUDIO_PCM_IMPL}" "audio_poly_pwm.c" "minigb_apu/minigb_apu.c" "touch.c" "serial_input.c" "sample_rom.c" "profiler.c" "battery.c" "lilygo_epd/ed047tc1.c" "lilygo_epd/i2s_data_bus.c" "lilygo_epd/rmt_pulse.c"
    INCLUDE_DIRS "." "msg" "crankboy_core" "minigb_apu" "lilygo_epd"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd esp_driver_ledc esp_driver_i2s esp_driver_i2c esp_driver_usb_serial_jtag esp_driver_rmt esp_adc
)

# Audio APU format: 16-bit signed integer
target_compile_definitions(${COMPONENT_LIB} PRIVATE MINIGB_APU_AUDIO_FORMAT_S16SYS)

if ((IDF_VERSION_MAJOR GREATER_EQUAL 5) AND (${IDF_TARGET} MATCHES "^esp32$|esp32s2|esp32s3"))
    target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables -ftree-switch-conversion)
endif ()

# Compile the emulator translation unit at -O3.
# The CrankBoy core is embedded through gbemu.c as a single translation unit.
# Keep it highly optimised, and relax a couple of warnings that come from the
# template-expanded interpreter labels in the vendored core.
set_source_files_properties(gbemu.c PROPERTIES COMPILE_FLAGS "-O3 -Wno-error=unused-label")
