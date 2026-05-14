#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "fastepd/FastEPD.h"
#include "peanut_gb/paperboy_gb.h"

static const char *TAG = "papers3_demo";
static FASTEPDSTATE s_epd;
static sdmmc_card_t *s_sd_card;
static uint8_t *s_rom_data;
static size_t s_rom_size;
static volatile bool s_emu_task_run;
static volatile uint32_t s_emu_frames;
static volatile uint32_t s_emu_errors;
static uint8_t s_render_framebuffer[PAPERBOY_GB_LCD_WIDTH * PAPERBOY_GB_LCD_HEIGHT];
static const uint8_t s_stub_rom[0x150] = {
    [0x147] = 0x00,
    [0x148] = 0x00,
    [0x149] = 0x00,
    [0x14D] = 0xE7,
};

#define PAPERBOY_GB_ROM_MAX_SIZE (8 * 1024 * 1024)
#define SD_MOUNT_POINT "/sdcard"

/* M5PaperS3 SD SPI pins */
#define SD_PIN_CLK 39
#define SD_PIN_MOSI 38
#define SD_PIN_MISO 40
#define SD_PIN_CS 47

#define GB_SCALE 2
#define GB_ORIGIN_X 24
#define GB_ORIGIN_Y 100
#define GB_TOP_LINE (GB_ORIGIN_Y - 3)
#define GB_BOTTOM_LINE (GB_ORIGIN_Y + (PAPERBOY_GB_LCD_HEIGHT * GB_SCALE) + 2)
#define GB_RENDER_INTERVAL_MS 120
#define GB_EMU_TARGET_HZ 30
#define GB_EMU_FRAME_PERIOD_US (1000000 / GB_EMU_TARGET_HZ)
#define GB_EMU_TASK_PRIORITY 1
#define STATUS_TOP_LINE (s_epd.height - 96)
#define STATUS_BOTTOM_LINE (s_epd.height - 14)

static esp_err_t sdcard_mount(void)
{
#if CONFIG_PAPERBOY_SD_ENABLE
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_MOSI,
        .miso_io_num = SD_PIN_MISO,
        .sclk_io_num = SD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t err;

    host.slot = SPI2_HOST;
    slot_config.host_id = host.slot;
    slot_config.gpio_cs = SD_PIN_CS;

    /* Enable internal pull-ups for better signal integrity */
    gpio_set_pull_mode((gpio_num_t)SD_PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)SD_PIN_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)SD_PIN_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)SD_PIN_CS, GPIO_PULLUP_ONLY);

    err = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT,
                                  &host,
                                  &slot_config,
                                  &mount_config,
                                  &s_sd_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(err));
        spi_bus_free(host.slot);
        return err;
    }

    sdmmc_card_print_info(stdout, s_sd_card);
    ESP_LOGI(TAG, "Mounted SD card on %s", SD_MOUNT_POINT);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static bool load_gb_rom_from_file(const char *path)
{
    FILE *f;
    long file_len;
    size_t read_len;

    f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "ROM file not found: %s", path);
        return false;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        ESP_LOGW(TAG, "Failed to seek ROM file");
        return false;
    }

    file_len = ftell(f);
    if (file_len <= 0 || file_len > PAPERBOY_GB_ROM_MAX_SIZE) {
        fclose(f);
        ESP_LOGW(TAG, "Invalid ROM size: %ld", file_len);
        return false;
    }

    rewind(f);
    s_rom_data = (uint8_t *)malloc((size_t)file_len);
    if (s_rom_data == NULL) {
        fclose(f);
        ESP_LOGE(TAG, "Failed to allocate ROM buffer (%ld bytes)", file_len);
        return false;
    }

    read_len = fread(s_rom_data, 1, (size_t)file_len, f);
    fclose(f);
    if (read_len != (size_t)file_len) {
        ESP_LOGE(TAG, "Short ROM read: %u/%ld", (unsigned)read_len, file_len);
        free(s_rom_data);
        s_rom_data = NULL;
        return false;
    }

    s_rom_size = (size_t)file_len;
    ESP_LOGI(TAG, "Loaded ROM from SD: %s (%u bytes)", path, (unsigned)s_rom_size);
    return true;
}

static void probe_sd_file(const char *path)
{
    FILE *f;
    uint8_t probe[16] = {0};
    size_t n;
    char hex[(sizeof(probe) * 3) + 1];
    size_t pos = 0;
    size_t i;

    f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "Cannot open SD file: %s", path);
        return;
    }

    n = fread(probe, 1, sizeof(probe), f);
    fclose(f);

    for (i = 0; i < n; i++) {
        pos += (size_t)snprintf(&hex[pos], sizeof(hex) - pos, "%02X ", probe[i]);
    }
    if (pos > 0) {
        hex[pos - 1] = '\0';
    }
    ESP_LOGI(TAG, "Read %u bytes from %s: %s", (unsigned)n, path, (n > 0) ? hex : "<empty>");
}

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

static void draw_status_box(const char *status)
{
    int y0 = s_epd.height - 90;
    int y1 = s_epd.height - 20;

    bbepRectangle(&s_epd, 20, y0, s_epd.width - 20, y1, BBEP_WHITE, 1);
    bbepRectangle(&s_epd, 20, y0, s_epd.width - 20, y1, BBEP_BLACK, 0);
    bbepWriteString(&s_epd, 26, y0 + 18, (char *)status, FONT_8x8, BBEP_BLACK);
}

static void draw_gb_frame(const uint8_t *fb)
{
    int x;
    int y;

    if (fb == NULL || s_epd.pfnSetPixelFast == NULL) {
        return;
    }

    bbepRectangle(&s_epd,
                  GB_ORIGIN_X - 3,
                  GB_ORIGIN_Y - 3,
                  GB_ORIGIN_X + (PAPERBOY_GB_LCD_WIDTH * GB_SCALE) + 2,
                  GB_ORIGIN_Y + (PAPERBOY_GB_LCD_HEIGHT * GB_SCALE) + 2,
                  BBEP_BLACK,
                  0);

    for (y = 0; y < PAPERBOY_GB_LCD_HEIGHT; y++) {
        for (x = 0; x < PAPERBOY_GB_LCD_WIDTH; x++) {
            uint8_t shade = fb[(y * PAPERBOY_GB_LCD_WIDTH) + x];
            uint8_t color = (shade >= 2) ? BBEP_WHITE : BBEP_BLACK;
            int px = GB_ORIGIN_X + (x * GB_SCALE);
            int py = GB_ORIGIN_Y + (y * GB_SCALE);

            s_epd.pfnSetPixelFast(&s_epd, px, py, color);
            s_epd.pfnSetPixelFast(&s_epd, px + 1, py, color);
            s_epd.pfnSetPixelFast(&s_epd, px, py + 1, color);
            s_epd.pfnSetPixelFast(&s_epd, px + 1, py + 1, color);
        }
    }
}

static void gb_emu_task(void *arg)
{
    int64_t next_deadline_us;

    (void)arg;

    ESP_LOGI(TAG, "GB emulator task started");
    next_deadline_us = esp_timer_get_time();

    while (s_emu_task_run) {
        int64_t now_us;
        int64_t sleep_us;

        if (!paperboy_gb_run_frame()) {
            ESP_LOGE(TAG, "paperboy_gb_run_frame failed");
            s_emu_errors++;
            break;
        }

        s_emu_frames++;

        next_deadline_us += GB_EMU_FRAME_PERIOD_US;
        now_us = esp_timer_get_time();
        sleep_us = next_deadline_us - now_us;

        if (sleep_us > 0) {
            vTaskDelay(pdMS_TO_TICKS((sleep_us + 999) / 1000));
        } else {
            /* If we miss budget, avoid catch-up bursts and still yield. */
            next_deadline_us = now_us;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    s_emu_task_run = false;
    ESP_LOGW(TAG, "GB emulator task exited");
    vTaskDelete(NULL);
}

void app_main(void)
{
    int rc;
    esp_err_t sd_err;
    bool gb_ok;
    BaseType_t task_ok;
    const uint8_t *rom_data = s_stub_rom;
    size_t rom_size = sizeof(s_stub_rom);
    char gb_msg[48] = "Peanut-GB wired, ROM pending";
    char status_line[64] = {0};
    uint32_t last_frame_id = UINT32_MAX;
    uint32_t render_frames = 0;
    int64_t last_new_frame_us = 0;
    int64_t last_status_us = 0;

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

    sd_err = sdcard_mount();
    if (sd_err == ESP_OK) {
        const char *rom_path = SD_MOUNT_POINT "/game/tetris.gb";
        probe_sd_file(rom_path);
        if (load_gb_rom_from_file(rom_path)) {
            rom_data = s_rom_data;
            rom_size = s_rom_size;
            snprintf(gb_msg, sizeof(gb_msg), "Peanut-GB ROM loaded from SD");
        }
    } else {
        ESP_LOGW(TAG, "SD init skipped, falling back to built-in stub ROM");
    }

    gb_ok = paperboy_gb_init(rom_data, rom_size);
    if (!gb_ok) {
        ESP_LOGW(TAG, "Peanut-GB init skipped: %s", paperboy_gb_last_error());
    } else {
        paperboy_gb_set_buttons(0);
        s_emu_frames = 0;
        s_emu_errors = 0;

        s_emu_task_run = true;
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
        task_ok = xTaskCreatePinnedToCore(gb_emu_task,
                                          "gb_emu",
                                          8192,
                                          NULL,
                          GB_EMU_TASK_PRIORITY,
                                          NULL,
                                          1);
#else
        task_ok = xTaskCreate(gb_emu_task,
                              "gb_emu",
                              8192,
                              NULL,
                      GB_EMU_TASK_PRIORITY,
                              NULL);
#endif
        if (task_ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create emulator task");
            s_emu_task_run = false;
            gb_ok = false;
            snprintf(gb_msg, sizeof(gb_msg), "Peanut-GB task creation failed");
        }
    }

    draw_status_box(gb_ok ? gb_msg : "Peanut-GB init failed");
    if (gb_ok) {
        bbepRectangle(&s_epd,
                      GB_ORIGIN_X - 3,
                      GB_ORIGIN_Y - 3,
                      GB_ORIGIN_X + (PAPERBOY_GB_LCD_WIDTH * GB_SCALE) + 2,
                      GB_ORIGIN_Y + (PAPERBOY_GB_LCD_HEIGHT * GB_SCALE) + 2,
                      BBEP_BLACK,
                      0);
    }

    rc = bbepPartialUpdate(&s_epd, false, STATUS_TOP_LINE, STATUS_BOTTOM_LINE);
    if (rc != BBEP_SUCCESS) {
        ESP_LOGE(TAG, "bbepPartialUpdate failed: %d", rc);
        return;
    }

    if (gb_ok) {
        rc = bbepPartialUpdate(&s_epd, false, GB_TOP_LINE, GB_BOTTOM_LINE);
        if (rc != BBEP_SUCCESS) {
            ESP_LOGE(TAG, "bbepPartialUpdate for GB preview failed: %d", rc);
            return;
        }
    }

    ESP_LOGI(TAG, "Entering GB render loop");

    last_new_frame_us = esp_timer_get_time();
    last_status_us = last_new_frame_us;

    while (1) {
        uint32_t frame_id = 0;
        int64_t now_us = esp_timer_get_time();
        uint32_t emu_frames = s_emu_frames;
        bool stalled = false;

        if (!gb_ok || !s_emu_task_run) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!paperboy_gb_copy_framebuffer(s_render_framebuffer,
                                          sizeof(s_render_framebuffer),
                                          &frame_id)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (frame_id == 0 || frame_id == last_frame_id) {
            stalled = ((now_us - last_new_frame_us) > 2000000);
        } else {
            last_frame_id = frame_id;
            last_new_frame_us = now_us;
            draw_gb_frame(s_render_framebuffer);

            rc = bbepPartialUpdate(&s_epd, false, GB_TOP_LINE, GB_BOTTOM_LINE);
            if (rc != BBEP_SUCCESS) {
                ESP_LOGE(TAG, "bbepPartialUpdate render loop failed: %d", rc);
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            render_frames++;
        }

        if ((now_us - last_status_us) > 1000000) {
            if (stalled) {
                snprintf(status_line,
                         sizeof(status_line),
                         "STALL emu:%u err:%u render:%u",
                         (unsigned)emu_frames,
                         (unsigned)s_emu_errors,
                         (unsigned)render_frames);
            } else {
                snprintf(status_line,
                         sizeof(status_line),
                         "RUN emu:%u render:%u fid:%u",
                         (unsigned)emu_frames,
                         (unsigned)render_frames,
                         (unsigned)last_frame_id);
            }

            draw_status_box(status_line);
            rc = bbepPartialUpdate(&s_epd, false, STATUS_TOP_LINE, STATUS_BOTTOM_LINE);
            if (rc != BBEP_SUCCESS) {
                ESP_LOGE(TAG, "status bbepPartialUpdate failed: %d", rc);
            }

            ESP_LOGI(TAG, "%s", status_line);
            last_status_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(GB_RENDER_INTERVAL_MS));
    }
}
