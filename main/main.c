#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include <esp_timer.h>
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "msg/msg.h"
#include "gbemu.h"
#include "audio.h"
#include "background.h"
#include "touch.h"
#include "ui.h"
#include "battery.h"
#include "profiler.h"

static const char *TAG = "paperboy";
static sdmmc_card_t *s_sd_card;
static uint8_t *s_rom_data;
static size_t s_rom_size;
static char s_rom_path[256];

typedef struct {
    char           last_rom[256];
    audio_engine_t audio_engine;
} paperboy_cfg_t;

static paperboy_cfg_t s_cfg;

#define PAPERBOY_PERSIST_EXTENSION ".sav"
#define PAPERBOY_STATE_EXTENSION   ".state"
#define PAPERBOY_CFG_FILE          SD_MOUNT_POINT "/paperboy.cfg"

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
    free(s_rom_data);
    s_rom_data = NULL;
    s_rom_size = 0;

    /* Prefer internal DRAM: 1–3 cycle access vs PSRAM cache-miss (~20–40 cy).
     * Falls back to PSRAM when the ROM is too large for available SRAM. */
    s_rom_data = (uint8_t *)heap_caps_malloc(file_len,
                                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_rom_data == NULL) {
        ESP_LOGW(TAG, "ROM (%ld B) does not fit in internal SRAM, using PSRAM", file_len);
        s_rom_data = (uint8_t *)heap_caps_malloc(file_len, MALLOC_CAP_SPIRAM);
    } else {
        ESP_LOGI(TAG, "ROM (%ld B) allocated in internal SRAM", file_len);
    }
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
    strlcpy(s_rom_path, path, sizeof(s_rom_path));
    ESP_LOGI(TAG, "Loaded ROM from SD: %s (%u bytes)", path, (unsigned)s_rom_size);
    return true;
}

static bool build_save_path(const char *rom_path, char *save_path, size_t save_path_size)
{
    const char *extension = PAPERBOY_PERSIST_EXTENSION;
    const char *slash;
    const char *dot;
    size_t base_len;

    if (rom_path == NULL || rom_path[0] == '\0' || save_path == NULL || save_path_size == 0) {
        return false;
    }

    slash = strrchr(rom_path, '/');
    dot = strrchr(rom_path, '.');
    if (dot == NULL || (slash != NULL && dot < slash)) {
        dot = rom_path + strlen(rom_path);
    }

    base_len = (size_t)(dot - rom_path);
    if (base_len + strlen(extension) + 1 > save_path_size) {
        return false;
    }

    memcpy(save_path, rom_path, base_len);
    memcpy(save_path + base_len, extension, strlen(extension) + 1);
    return true;
}

static bool build_state_path(const char *rom_path, char *state_path, size_t state_path_size)
{
    const char *extension = PAPERBOY_STATE_EXTENSION;
    const char *slash;
    const char *dot;
    size_t base_len;

    if (rom_path == NULL || rom_path[0] == '\0' || state_path == NULL || state_path_size == 0) {
        return false;
    }

    slash = strrchr(rom_path, '/');
    dot = strrchr(rom_path, '.');
    if (dot == NULL || (slash != NULL && dot < slash)) {
        dot = rom_path + strlen(rom_path);
    }

    base_len = (size_t)(dot - rom_path);
    if (base_len + strlen(extension) + 1 > state_path_size) {
        return false;
    }

    memcpy(state_path, rom_path, base_len);
    memcpy(state_path + base_len, extension, strlen(extension) + 1);
    return true;
}

static void cfg_read(paperboy_cfg_t *cfg)
{
    cfg->last_rom[0]  = '\0';
    cfg->audio_engine = AUDIO_ENGINE_PCM;

    FILE *f = fopen(PAPERBOY_CFG_FILE, "r");
    if (!f) {
        ESP_LOGI(TAG, "No config file: %s", PAPERBOY_CFG_FILE);
        return;
    }

    char line[320];
    while (fgets(line, sizeof(line), f)) {
        line[strcspn(line, "\r\n")] = '\0';
        if (strncmp(line, "last_rom=", 9) == 0) {
            strlcpy(cfg->last_rom, line + 9, sizeof(cfg->last_rom));
        } else if (strncmp(line, "audio_engine=", 13) == 0) {
            int val = atoi(line + 13);
            if (val >= 0 && val < (int)AUDIO_ENGINE_COUNT) {
                cfg->audio_engine = (audio_engine_t)val;
            }
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "Config loaded: last_rom=%s audio=%d",
             cfg->last_rom[0] ? cfg->last_rom : "(none)", (int)cfg->audio_engine);
}

static bool cfg_write(const paperboy_cfg_t *cfg)
{
    FILE *f = fopen(PAPERBOY_CFG_FILE, "w");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open config for writing: %s", PAPERBOY_CFG_FILE);
        return false;
    }

    if (cfg->last_rom[0] != '\0') {
        fprintf(f, "last_rom=%s\n", cfg->last_rom);
    }
    fprintf(f, "audio_engine=%d\n", (int)cfg->audio_engine);

    if (fclose(f) != 0) {
        ESP_LOGE(TAG, "Failed to close config: %s", PAPERBOY_CFG_FILE);
        return false;
    }

    ESP_LOGI(TAG, "Config saved: last_rom=%s audio=%d",
             cfg->last_rom[0] ? cfg->last_rom : "(none)", (int)cfg->audio_engine);
    return true;
}

static bool save_game_persist(const char *rom_path)
{
    bool ok = false;
    char save_path[256];
    uint8_t *persist_data = NULL;
    size_t persist_size;
    FILE *save_file = NULL;
    size_t written;
    uint32_t timestamp;

    if (!paperboy_gb_has_persist()) {
        ESP_LOGW(TAG, "Save requested, but current ROM has no persistent storage");
        return false;
    }

    if (!build_save_path(rom_path, save_path, sizeof(save_path))) {
        ESP_LOGE(TAG, "Failed to build save path for ROM: %s", rom_path ? rom_path : "<null>");
        return false;
    }

    persist_size = paperboy_gb_persist_size();
    if (persist_size == 0) {
        ESP_LOGW(TAG, "Save requested, but persist payload is empty");
        return false;
    }

    persist_data = (uint8_t *)heap_caps_malloc(persist_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (persist_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u-byte save buffer in SPIRAM", (unsigned)persist_size);
        return false;
    }

    timestamp = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (!paperboy_gb_persist_export(persist_data, persist_size, timestamp)) {
        ESP_LOGE(TAG, "Persist export failed: %s", paperboy_gb_last_error());
        goto out;
    }

    save_file = fopen(save_path, "wb");
    if (save_file == NULL) {
        ESP_LOGE(TAG, "Failed to open save file: %s", save_path);
        goto out;
    }

    written = fwrite(persist_data, 1, persist_size, save_file);
    if (written != persist_size) {
        ESP_LOGE(TAG, "Short save write: %u/%u", (unsigned)written, (unsigned)persist_size);
        goto out;
    }

    if (fclose(save_file) != 0) {
        save_file = NULL;
        ESP_LOGE(TAG, "Failed to close save file: %s", save_path);
        goto out;
    }
    save_file = NULL;

    paperboy_gb_persist_mark_clean();
    ESP_LOGI(TAG, "Saved persist data to %s (%u bytes)", save_path, (unsigned)persist_size);
    ok = true;

out:
    if (save_file != NULL) {
        fclose(save_file);
    }
    heap_caps_free(persist_data);
    return ok;
}

static bool load_game_persist(const char *rom_path)
{
    bool ok = false;
    char save_path[256];
    uint8_t *persist_data = NULL;
    FILE *save_file = NULL;
    long file_len;
    size_t read_len;
    uint32_t timestamp;

    if (!paperboy_gb_has_persist()) {
        return false;
    }

    if (!build_save_path(rom_path, save_path, sizeof(save_path))) {
        ESP_LOGW(TAG, "Skipping persist load: invalid ROM path");
        return false;
    }

    save_file = fopen(save_path, "rb");
    if (save_file == NULL) {
        ESP_LOGI(TAG, "No save file present for ROM: %s", save_path);
        return false;
    }

    if (fseek(save_file, 0, SEEK_END) != 0) {
        ESP_LOGE(TAG, "Failed to seek save file: %s", save_path);
        goto out;
    }

    file_len = ftell(save_file);
    if (file_len <= 0) {
        ESP_LOGE(TAG, "Invalid save file size: %ld", file_len);
        goto out;
    }

    if ((size_t)file_len != paperboy_gb_persist_size()) {
        ESP_LOGW(TAG, "Save file size mismatch for %s: %ld != %u",
                 save_path,
                 file_len,
                 (unsigned)paperboy_gb_persist_size());
        goto out;
    }

    rewind(save_file);

    persist_data = (uint8_t *)heap_caps_malloc((size_t)file_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (persist_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %ld-byte load buffer in SPIRAM", file_len);
        goto out;
    }

    read_len = fread(persist_data, 1, (size_t)file_len, save_file);
    if (read_len != (size_t)file_len) {
        ESP_LOGE(TAG, "Short save read: %u/%ld", (unsigned)read_len, file_len);
        goto out;
    }

    timestamp = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (!paperboy_gb_persist_import(persist_data, (size_t)file_len, timestamp)) {
        ESP_LOGE(TAG, "Persist import failed: %s", paperboy_gb_last_error());
        goto out;
    }

    ESP_LOGI(TAG, "Loaded persist data from %s", save_path);
    ok = true;

out:
    if (save_file != NULL) {
        fclose(save_file);
    }
    heap_caps_free(persist_data);
    return ok;
}

static bool save_game_state(const char *rom_path)
{
    bool ok = false;
    char state_path[256];
    uint8_t *state_data = NULL;
    size_t state_size;
    FILE *state_file = NULL;
    size_t written;

    if (!build_state_path(rom_path, state_path, sizeof(state_path))) {
        ESP_LOGE(TAG, "Failed to build state path for ROM: %s", rom_path ? rom_path : "<null>");
        return false;
    }

    state_size = paperboy_gb_state_size();
    if (state_size == 0) {
        ESP_LOGE(TAG, "Failed to determine snapshot size");
        return false;
    }

    state_data = (uint8_t *)heap_caps_malloc(state_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (state_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %u-byte state buffer in SPIRAM", (unsigned)state_size);
        return false;
    }

    if (!paperboy_gb_state_export(state_data, state_size)) {
        ESP_LOGE(TAG, "State export failed: %s", paperboy_gb_last_error());
        goto out;
    }

    state_file = fopen(state_path, "wb");
    if (state_file == NULL) {
        ESP_LOGE(TAG, "Failed to open state file: %s", state_path);
        goto out;
    }

    written = fwrite(state_data, 1, state_size, state_file);
    if (written != state_size) {
        ESP_LOGE(TAG, "Short state write: %u/%u", (unsigned)written, (unsigned)state_size);
        goto out;
    }

    if (fclose(state_file) != 0) {
        state_file = NULL;
        ESP_LOGE(TAG, "Failed to close state file: %s", state_path);
        goto out;
    }
    state_file = NULL;

    strlcpy(s_cfg.last_rom, rom_path, sizeof(s_cfg.last_rom));
    s_cfg.audio_engine = audio_get_engine();
    if (!cfg_write(&s_cfg)) {
        goto out;
    }

    ESP_LOGI(TAG, "Saved snapshot to %s (%u bytes)", state_path, (unsigned)state_size);
    ok = true;

out:
    if (state_file != NULL) {
        fclose(state_file);
    }
    heap_caps_free(state_data);
    return ok;
}

static bool load_game_state(const char *rom_path)
{
    bool ok = false;
    char state_path[256];
    uint8_t *state_data = NULL;
    FILE *state_file = NULL;
    long file_len;
    size_t read_len;

    if (!build_state_path(rom_path, state_path, sizeof(state_path))) {
        ESP_LOGW(TAG, "Skipping snapshot load: invalid ROM path");
        return false;
    }

    state_file = fopen(state_path, "rb");
    if (state_file == NULL) {
        ESP_LOGI(TAG, "No snapshot present for ROM: %s", state_path);
        return false;
    }

    if (fseek(state_file, 0, SEEK_END) != 0) {
        ESP_LOGE(TAG, "Failed to seek state file: %s", state_path);
        goto out;
    }

    file_len = ftell(state_file);
    if (file_len <= 0) {
        ESP_LOGE(TAG, "Invalid state file size: %ld", file_len);
        goto out;
    }

    rewind(state_file);

    state_data = (uint8_t *)heap_caps_malloc((size_t)file_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (state_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate %ld-byte state buffer in SPIRAM", file_len);
        goto out;
    }

    read_len = fread(state_data, 1, (size_t)file_len, state_file);
    if (read_len != (size_t)file_len) {
        ESP_LOGE(TAG, "Short state read: %u/%ld", (unsigned)read_len, file_len);
        goto out;
    }

    if (!paperboy_gb_state_import(state_data, (size_t)file_len)) {
        ESP_LOGE(TAG, "State import failed: %s", paperboy_gb_last_error());
        goto out;
    }

    ESP_LOGI(TAG, "Loaded snapshot from %s", state_path);
    ok = true;

out:
    if (state_file != NULL) {
        fclose(state_file);
    }
    heap_caps_free(state_data);
    return ok;
}

static bool save_game_session(const char *rom_path)
{
    bool persist_ok = true;

    if (paperboy_gb_has_persist()) {
        persist_ok = save_game_persist(rom_path);
    }

    if (!persist_ok) {
        return false;
    }

    return save_game_state(rom_path);
}

static bool load_game_session(const char *rom_path, bool include_snapshot)
{
    bool state_loaded = true;

    if (paperboy_gb_has_persist()) {
        load_game_persist(rom_path);
    }

    if (include_snapshot) {
        state_loaded = load_game_state(rom_path);
    }

    return state_loaded;
}

/* ui_put_pixel / ui_put_rect are now defined in ui.c (see ui.h). */

void app_main(void)
{
    esp_err_t sd_err;
    bool gb_ok;
    bool load_snapshot = false;
    uint8_t prev_actions = 0;
    const uint8_t *rom_data = NULL;
    size_t rom_size = 0;

    ESP_LOGI(TAG, "Initializing MSG");
    msg_init();
    msg_start();

    tp_init();  /* GT911 touch — non-fatal if absent */
    battery_init();

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
        char rom_path[256] = {0};

        cfg_read(&s_cfg);
        audio_set_engine(s_cfg.audio_engine);

        while (1) {
            const ui_rom_pick_result_t pick = ui_rom_picker(SD_MOUNT_POINT, rom_path, sizeof(rom_path));

            if (pick == UI_ROM_PICK_NONE) {
                break;
            }

            if (pick == UI_ROM_PICK_LOAD_LAST) {
                if (s_cfg.last_rom[0] == '\0') {
                    ui_show_notice("LOAD", "No snapshot", 750);
                    continue;
                }
                strlcpy(rom_path, s_cfg.last_rom, sizeof(rom_path));
                load_snapshot = true;
            } else {
                load_snapshot = false;
            }

            ui_show_notice("LOAD", "Loading", 0);

            if (!load_gb_rom_from_file(rom_path)) {
                if (load_snapshot) {
                    ui_show_notice("LOAD", "ROM missing", 750);
                    continue;
                }
                continue;
            }

            rom_data = s_rom_data;
            rom_size = s_rom_size;

            gb_ok = paperboy_gb_init(rom_data, rom_size);
            if (!gb_ok) {
                ESP_LOGW(TAG, "Peanut-GB init skipped: %s", paperboy_gb_last_error());
                rom_data = NULL;
                rom_size = 0;
                continue;
            }

            if (load_snapshot && !load_game_session(s_rom_path, true)) {
                ui_show_notice("LOAD", "No snapshot", 750);
                rom_data = NULL;
                rom_size = 0;
                continue;
            }

            if (!load_snapshot) {
                load_game_session(s_rom_path, false);
            }

            /* Persist chosen ROM and audio engine to config. */
            strlcpy(s_cfg.last_rom, s_rom_path, sizeof(s_cfg.last_rom));
            s_cfg.audio_engine = audio_get_engine();
            cfg_write(&s_cfg);

            paperboy_gb_set_buttons(0);
            break;
        }
    } else {
        ESP_LOGW(TAG, "SD init skipped, falling back to built-in stub ROM");
    }

    if (rom_data == NULL) {
        gb_ok = paperboy_gb_init(rom_data, rom_size);
        if (!gb_ok) {
            ESP_LOGW(TAG, "Peanut-GB init skipped: %s", paperboy_gb_last_error());
        } else {
            paperboy_gb_set_buttons(0);
        }
    }

    ESP_LOGI(TAG, "Entering GB render loop");

    /* Maximum number of consecutive rendered frames that may be skipped
     * (video output suppressed) when the emulator falls behind VSYNC. */
    #define MAX_FRAME_SKIPS 2

    int skip_count = 0;

    /* Obtain the first back-buffer and record the current VSYNC epoch. */
    uint8_t *video_fb = msg_flip();
    uint32_t vsync_ref = msg_get_vsync_count();

    while (1) {
        tp_state_t touch_state;
        uint8_t action_edges;

        if (!gb_ok) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* Poll touchscreen and update GB button state every emulated frame. */
        PROF_BEGIN(PROF_TOUCH);
        touch_state = tp_read_state();
        paperboy_gb_set_buttons(touch_state.gb_buttons);
        PROF_END(PROF_TOUCH);

        action_edges = touch_state.actions & (uint8_t)~prev_actions;
        prev_actions = touch_state.actions;

        if (action_edges & TP_ACTION_SAVE) {
            paperboy_gb_set_buttons(0);
            if (save_game_session(s_rom_path)) {
                ui_show_notice("SAVE", "Saved", 500);
            }
            video_fb = msg_flip();
            vsync_ref = msg_get_vsync_count();
            skip_count = 0;
            continue;
        }

        if (action_edges & TP_ACTION_LOAD) {
            paperboy_gb_set_buttons(0);
            if (load_game_state(s_rom_path)) {
                ui_show_notice("LOAD", "Loaded", 500);
            }
            video_fb = msg_flip();
            vsync_ref = msg_get_vsync_count();
            skip_count = 0;
            continue;
        }

        if (action_edges & TP_ACTION_CLEAR_SCREEN) {
            paperboy_gb_set_buttons(0);
            ui_clear_ghosting();
            video_fb = msg_flip();
            vsync_ref = msg_get_vsync_count();
            skip_count = 0;
            continue;
        }

        bool skip_render = (skip_count > 0);

        PROF_BEGIN(PROF_FRAME);
        if (!paperboy_gb_run_frame(video_fb, skip_render)) {
            ESP_LOGE(TAG, "paperboy_gb_run_frame failed: %s", paperboy_gb_last_error());
            gb_ok = false;
            continue;
        }
        PROF_END(PROF_FRAME);

        /* Battery-low overlay: overwrite the top row of the game frame. */
        if (!skip_render && battery_is_low()) {
            ui_draw_bat_low_overlay(video_fb);
        }

        /* Check whether at least one VSYNC fired while we were rendering. */
        bool missed = (msg_get_vsync_count() != vsync_ref);

        if (missed && skip_count < MAX_FRAME_SKIPS) {
            /* We missed VSYNC.  Don't swap buffers — the EPD keeps showing
             * the last submitted frame.  Run another emulation pass without
             * rendering to try to catch back up. */
            skip_count++;
            video_fb  = msg_flip_nowait();
            vsync_ref = msg_get_vsync_count();
        } else {
            /* Either on time, or we've hit the skip cap: submit the last
             * rendered frame and wait for the next VSYNC. */
            skip_count = 0;
            PROF_BEGIN(PROF_FLIP);
            video_fb  = msg_flip();
            PROF_END(PROF_FLIP);
            vsync_ref = msg_get_vsync_count();
        }

        profiler_frame_end(skip_render);
    }
}
