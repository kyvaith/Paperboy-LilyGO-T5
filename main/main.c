#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include <esp_timer.h>
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "msg/msg.h"
#include "gbemu.h"
#include "background.h"
#include "touch.h"

static const char *TAG = "paperboy";
static sdmmc_card_t *s_sd_card;
static uint8_t *s_rom_data;
static size_t s_rom_size;

#define PAPERBOY_GB_ROM_MAX_SIZE (4 * 1024 * 1024)
#define SD_MOUNT_POINT "/sdcard"

/* M5PaperS3 SD SPI pins */
#define SD_PIN_CLK 39
#define SD_PIN_MOSI 38
#define SD_PIN_MISO 40
#define SD_PIN_CS 47

/*
 * GB rendering: 3× scale, Bayer-like dithering, 90° CW rotation.
 * GB_ORIGIN_X_R90 MUST be a multiple of 8 – the renderer writes whole bytes.
 * After CW 90° and 3× scale the image occupies:
 *   EPD x: [GB_ORIGIN_X_R90 .. GB_ORIGIN_X_R90 + GB_EPD_COL_SPAN - 1]  (432 px)
 *   EPD y: [GB_ORIGIN_Y_R90 .. GB_ORIGIN_Y_R90 + GB_EPD_ROW_SPAN - 1]  (480 px)
 */
#define GB_ORIGIN_X_R90    432  /* must be ≡ 0 (mod 8) for byte-aligned writes */
#define GB_ORIGIN_Y_R90     30
#define GB_EPD_COL_SPAN    (PAPERBOY_GB_LCD_HEIGHT * 3)  /* 432 EPD columns */
#define GB_EPD_ROW_SPAN    (PAPERBOY_GB_LCD_WIDTH  * 3)  /* 480 EPD rows    */
#define GB_EPD_PITCH_BYTES (GB_EPD_COL_SPAN / 8)        /* 54 bytes / row  */
#define GB_TOP_LINE         GB_ORIGIN_Y_R90
#define GB_BOTTOM_LINE     (GB_ORIGIN_Y_R90 + GB_EPD_ROW_SPAN - 1)
static esp_err_t sdcard_mount(void)
{
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
    s_rom_data = (uint8_t *)(uint8_t *)heap_caps_calloc(1, file_len, MALLOC_CAP_SPIRAM);
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

// Our FB is a 432x160 buffer
static void ui_put_pixel_raw(uint8_t *fb, int x, int y, bool c) {
    uint8_t *p = &(fb[y * 54 + x / 8]);
    uint8_t mask = 0x80 >> (x % 8);
    if (c)
        *p |= mask;
    else
        *p &= ~mask;
}

// Input X,Y ranges from (0,0) to (159,143)
void ui_put_pixel(uint8_t *fb, int x, int y, int c) {
    for (int i = 0; i < 3; i++) {
        ui_put_pixel_raw(fb, (143 - y) * 3 + i, x, (i < c));
    }
}

void ui_put_rect(uint8_t *fb, int x0, int y0, int x1, int y1, int c) {
    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            ui_put_pixel(fb, x, y, c);
        }
    }
}

void app_main(void)
{
    esp_err_t sd_err;
    bool gb_ok;
    const uint8_t *rom_data = NULL;
    size_t rom_size = 0;

    ESP_LOGI(TAG, "Initializing MSG");
    msg_init();
    msg_start();

    tp_init();  /* GT911 touch — non-fatal if absent */

    uint8_t *fb = (uint8_t *)heap_caps_calloc(1, EPD_FB_SIZE, MALLOC_CAP_SPIRAM);
    memset(fb, 0xff, EPD_FB_SIZE);
    // Clear screen
    msg_display_image(fb, false);
    vTaskDelay(pdMS_TO_TICKS(500));
    msg_display_image(fb, true);
    vTaskDelay(pdMS_TO_TICKS(500));
    msg_display_image(fb, false);
    vTaskDelay(pdMS_TO_TICKS(500));
    msg_display_image(fb, true);
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < sizeof(image); i++) {
        fb[i] = ~image[i];
    }
    //memcpy(fb, image, sizeof(image));
    msg_display_image(fb, false);

    vTaskDelay(pdMS_TO_TICKS(500));
    msg_enable_video(true);

    sd_err = sdcard_mount();
    if (sd_err == ESP_OK) {
        const char *rom_path = SD_MOUNT_POINT "/game/PokemonBlue.gb";
        probe_sd_file(rom_path);
        if (load_gb_rom_from_file(rom_path)) {
            rom_data = s_rom_data;
            rom_size = s_rom_size;
        }
    } else {
        ESP_LOGW(TAG, "SD init skipped, falling back to built-in stub ROM");
    }

    gb_ok = paperboy_gb_init(rom_data, rom_size);
    if (!gb_ok) {
        ESP_LOGW(TAG, "Peanut-GB init skipped: %s", paperboy_gb_last_error());
    } else {
        paperboy_gb_set_buttons(0);
    }

    ESP_LOGI(TAG, "Entering GB render loop");

    /* Maximum number of consecutive rendered frames that may be skipped
     * (video output suppressed) when the emulator falls behind VSYNC. */
    #define MAX_FRAME_SKIPS 2

    uint32_t last_print = 0;
    int skip_count = 0;
    int total_skip_count = 0;

    /* Obtain the first back-buffer and record the current VSYNC epoch. */
    uint8_t *video_fb = msg_flip();
    uint32_t vsync_ref = msg_get_vsync_count();

    while (1) {
        if (!gb_ok) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* Poll touchscreen and update GB button state every emulated frame. */
        paperboy_gb_set_buttons(tp_read_buttons());

        bool skip_render = (skip_count > 0);

        uint32_t start = esp_timer_get_time();
        if (!paperboy_gb_run_frame(video_fb, skip_render)) {
            ESP_LOGE(TAG, "paperboy_gb_run_frame failed: %s", paperboy_gb_last_error());
            gb_ok = false;
            continue;
        }
        uint32_t end = esp_timer_get_time();

        if ((end - last_print) > 1000000) {
            ESP_LOGI(TAG, "Last frame time: %lu us (skip_count=%d)", end - start, total_skip_count);
            last_print = end;
            total_skip_count = 0;
        }

        /* Check whether at least one VSYNC fired while we were rendering. */
        bool missed = (msg_get_vsync_count() != vsync_ref);

        if (missed && skip_count < MAX_FRAME_SKIPS) {
            /* We missed VSYNC.  Don't swap buffers — the EPD keeps showing
             * the last submitted frame.  Run another emulation pass without
             * rendering to try to catch back up. */
            skip_count++;
            total_skip_count++;
            video_fb  = msg_flip_nowait();
            vsync_ref = msg_get_vsync_count();
        } else {
            /* Either on time, or we've hit the skip cap: submit the last
             * rendered frame and wait for the next VSYNC. */
            skip_count = 0;
            video_fb  = msg_flip();
            vsync_ref = msg_get_vsync_count();
        }
    }
}
