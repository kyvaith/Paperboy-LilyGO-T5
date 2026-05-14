idf_component_register(
    SRCS "blink_example_main.c"
    INCLUDE_DIRS "." "fastepd"
    REQUIRES driver esp_timer esp_lcd
)
