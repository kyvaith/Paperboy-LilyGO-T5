idf_component_register(
    SRCS "blink_example_main.c" "peanut_gb/paperboy_gb.c"
    INCLUDE_DIRS "." "fastepd" "peanut_gb"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd
)
