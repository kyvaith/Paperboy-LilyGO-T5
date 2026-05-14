#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fastepd/FastEPD.h"

static const char *TAG = "papers3_demo";
static FASTEPDSTATE s_epd;

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

void app_main(void)
{
    int rc;
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

    draw_partial_overlay();
    rc = bbepPartialUpdate(&s_epd, false, s_epd.height - 96, s_epd.height - 14);
    if (rc != BBEP_SUCCESS) {
        ESP_LOGE(TAG, "bbepPartialUpdate failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Demo complete. Entering idle loop.");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
