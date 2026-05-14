#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fastepd/FastEPD.h"
#include "peanut_gb/paperboy_gb.h"

static const char *TAG = "papers3_demo";
static FASTEPDSTATE s_epd;
static const uint8_t s_stub_rom[0x150] = {
    [0x147] = 0x00,
    [0x148] = 0x00,
    [0x149] = 0x00,
    [0x14D] = 0xE7,
};

static void draw_initial_scene(void)
{
    char title[] = "FastEPD PaperS3";
    char subtitle[] = "Core port running in ESP-IDF";

    bbepFillScreen(&s_epd, BBEP_WHITE);
    bbepRectangle(&s_epd, 10, 10, s_epd.width - 11, s_epd.height - 11, BBEP_BLACK, 0);
    bbepDrawLine(&s_epd, 10, 10, s_epd.width - 11, s_epd.height - 11, BBEP_BLACK);
    bbepDrawLine(&s_epd, s_epd.width - 11, 10, 10, s_epd.height - 11, BBEP_BLACK);
    bbepWriteString(&s_epd, 24, 30, title, FONT_16x16, BBEP_BLACK);
    bbepWriteString(&s_epd, 24, 58, subtitle, FONT_8x8, BBEP_BLACK);
}

static void draw_partial_overlay(void)
{
    char partial_msg[] = "Partial update OK";
    int y0 = s_epd.height - 90;
    int y1 = s_epd.height - 20;

    bbepRectangle(&s_epd, 20, y0, s_epd.width - 20, y1, BBEP_WHITE, 1);
    bbepRectangle(&s_epd, 20, y0, s_epd.width - 20, y1, BBEP_BLACK, 0);
    bbepWriteString(&s_epd, 32, y0 + 22, partial_msg, FONT_16x16, BBEP_BLACK);
}

static void draw_gb_preview(void)
{
    const uint8_t *fb = paperboy_gb_get_framebuffer();
    const int scale = 2;
    const int origin_x = 24;
    const int origin_y = 100;
    int x;
    int y;

    if (fb == NULL || s_epd.pfnSetPixelFast == NULL) {
        return;
    }

    bbepRectangle(&s_epd,
                  origin_x - 3,
                  origin_y - 3,
                  origin_x + (PAPERBOY_GB_LCD_WIDTH * scale) + 2,
                  origin_y + (PAPERBOY_GB_LCD_HEIGHT * scale) + 2,
                  BBEP_BLACK,
                  0);

    for (y = 0; y < PAPERBOY_GB_LCD_HEIGHT; y++) {
        for (x = 0; x < PAPERBOY_GB_LCD_WIDTH; x++) {
            uint8_t shade = fb[(y * PAPERBOY_GB_LCD_WIDTH) + x];
            uint8_t color = (shade >= 2) ? BBEP_WHITE : BBEP_BLACK;
            int px = origin_x + (x * scale);
            int py = origin_y + (y * scale);

            s_epd.pfnSetPixelFast(&s_epd, px, py, color);
            s_epd.pfnSetPixelFast(&s_epd, px + 1, py, color);
            s_epd.pfnSetPixelFast(&s_epd, px, py + 1, color);
            s_epd.pfnSetPixelFast(&s_epd, px + 1, py + 1, color);
        }
    }
}

void app_main(void)
{
    int rc;
    bool gb_ok;
    ESP_LOGI(TAG, "Initializing FastEPD for M5PaperS3...");

    rc = bbepInitPanel(&s_epd, BB_PANEL_M5PAPERS3, 20000000);
    if (rc != BBEP_SUCCESS) {
        ESP_LOGE(TAG, "bbepInitPanel failed: %d", rc);
        return;
    }

    bbepSetRotation(&s_epd, 0);
    draw_initial_scene();

    rc = bbepFullUpdate(&s_epd, CLEAR_SLOW, false, NULL);
    if (rc != BBEP_SUCCESS) {
        ESP_LOGE(TAG, "bbepFullUpdate failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Full update done, waiting before partial update...");
    vTaskDelay(pdMS_TO_TICKS(1500));

    gb_ok = paperboy_gb_init(s_stub_rom, sizeof(s_stub_rom));
    if (!gb_ok) {
        ESP_LOGW(TAG, "Peanut-GB init skipped: %s", paperboy_gb_last_error());
    } else {
        int frame;

        paperboy_gb_set_buttons(0);
        for (frame = 0; frame < 2; frame++) {
            paperboy_gb_run_frame();
        }
        draw_gb_preview();
    }

    draw_partial_overlay();
    if (gb_ok) {
        char gb_msg[] = "Peanut-GB wired (stub ROM)";
        bbepWriteString(&s_epd, 26, s_epd.height - 54, gb_msg, FONT_8x8, BBEP_BLACK);
    } else {
        char gb_msg[] = "Peanut-GB wired, ROM pending";
        bbepWriteString(&s_epd, 26, s_epd.height - 54, gb_msg, FONT_8x8, BBEP_BLACK);
    }

    rc = bbepPartialUpdate(&s_epd, false, s_epd.height - 96, s_epd.height - 14);
    if (rc != BBEP_SUCCESS) {
        ESP_LOGE(TAG, "bbepPartialUpdate failed: %d", rc);
        return;
    }

    if (gb_ok) {
        rc = bbepPartialUpdate(&s_epd, false, 96, 100 + (PAPERBOY_GB_LCD_HEIGHT * 2) + 6);
        if (rc != BBEP_SUCCESS) {
            ESP_LOGE(TAG, "bbepPartialUpdate for GB preview failed: %d", rc);
            return;
        }
    }

    ESP_LOGI(TAG, "Demo complete. Entering idle loop.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
