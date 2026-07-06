#include "ui.h"
#include "battery.h"
#include "msg/msg.h"
#include "touch.h"
#include "serial_input.h"
#include "gbemu.h"   /* GB_BTN_* masks */
#include "audio.h"   /* audio_engine_t, audio_set/get_engine */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

/* ── 5×7 bitmap font ──────────────────────────────────────────────────────
 *
 * 95 glyphs covering ASCII 0x20 (SPACE) … 0x7E (~).
 * Each glyph is 5 bytes (columns 0–4).  Within each byte bit-0 is the
 * topmost row and bit-6 is the bottommost row (bit-7 is unused).
 *
 * This is the classic Nokia-5110 / Arduino LCD font, which is in the public
 * domain (derived from work by Limor Fried / Adafruit Industries).
 */
static const uint8_t s_font5x7[95][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /* 0x20 ' '  */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 }, /* 0x21 '!'  */
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, /* 0x22 '"'  */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, /* 0x23 '#'  */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, /* 0x24 '$'  */
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, /* 0x25 '%'  */
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, /* 0x26 '&'  */
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, /* 0x27 '\'' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 }, /* 0x28 '('  */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 }, /* 0x29 ')'  */
    { 0x08, 0x2A, 0x1C, 0x2A, 0x08 }, /* 0x2A '*'  */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* 0x2B '+'  */
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, /* 0x2C ','  */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* 0x2D '-'  */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* 0x2E '.'  */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* 0x2F '/'  */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0x30 '0'  */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 0x31 '1'  */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 0x32 '2'  */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 0x33 '3'  */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 0x34 '4'  */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 0x35 '5'  */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 0x36 '6'  */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 0x37 '7'  */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 0x38 '8'  */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 0x39 '9'  */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* 0x3A ':'  */
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, /* 0x3B ';'  */
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, /* 0x3C '<'  */
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, /* 0x3D '='  */
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, /* 0x3E '>'  */
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, /* 0x3F '?'  */
    { 0x32, 0x49, 0x79, 0x41, 0x3E }, /* 0x40 '@'  */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* 0x41 'A'  */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* 0x42 'B'  */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* 0x43 'C'  */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* 0x44 'D'  */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* 0x45 'E'  */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 }, /* 0x46 'F'  */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A }, /* 0x47 'G'  */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* 0x48 'H'  */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* 0x49 'I'  */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* 0x4A 'J'  */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* 0x4B 'K'  */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* 0x4C 'L'  */
    { 0x7F, 0x02, 0x04, 0x02, 0x7F }, /* 0x4D 'M'  */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* 0x4E 'N'  */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* 0x4F 'O'  */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* 0x50 'P'  */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* 0x51 'Q'  */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* 0x52 'R'  */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* 0x53 'S'  */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* 0x54 'T'  */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* 0x55 'U'  */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* 0x56 'V'  */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F }, /* 0x57 'W'  */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* 0x58 'X'  */
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, /* 0x59 'Y'  */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* 0x5A 'Z'  */
    { 0x00, 0x7F, 0x41, 0x41, 0x00 }, /* 0x5B '['  */
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, /* 0x5C '\\' */
    { 0x00, 0x41, 0x41, 0x7F, 0x00 }, /* 0x5D ']'  */
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, /* 0x5E '^'  */
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, /* 0x5F '_'  */
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, /* 0x60 '`'  */
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, /* 0x61 'a'  */
    { 0x7F, 0x48, 0x44, 0x44, 0x38 }, /* 0x62 'b'  */
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, /* 0x63 'c'  */
    { 0x38, 0x44, 0x44, 0x48, 0x7F }, /* 0x64 'd'  */
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, /* 0x65 'e'  */
    { 0x08, 0x7E, 0x09, 0x01, 0x02 }, /* 0x66 'f'  */
    { 0x08, 0x54, 0x54, 0x54, 0x3C }, /* 0x67 'g'  */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 }, /* 0x68 'h'  */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 }, /* 0x69 'i'  */
    { 0x20, 0x40, 0x44, 0x3D, 0x00 }, /* 0x6A 'j'  */
    { 0x7F, 0x10, 0x28, 0x44, 0x00 }, /* 0x6B 'k'  */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 }, /* 0x6C 'l'  */
    { 0x7C, 0x04, 0x18, 0x04, 0x7C }, /* 0x6D 'm'  */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 }, /* 0x6E 'n'  */
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, /* 0x6F 'o'  */
    { 0x7C, 0x14, 0x14, 0x14, 0x08 }, /* 0x70 'p'  */
    { 0x08, 0x14, 0x14, 0x18, 0x7C }, /* 0x71 'q'  */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 }, /* 0x72 'r'  */
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, /* 0x73 's'  */
    { 0x04, 0x3F, 0x44, 0x40, 0x20 }, /* 0x74 't'  */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C }, /* 0x75 'u'  */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C }, /* 0x76 'v'  */
    { 0x3C, 0x40, 0x20, 0x40, 0x3C }, /* 0x77 'w'  */
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, /* 0x78 'x'  */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C }, /* 0x79 'y'  */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 }, /* 0x7A 'z'  */
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, /* 0x7B '{'  */
    { 0x00, 0x00, 0x7F, 0x00, 0x00 }, /* 0x7C '|'  */
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, /* 0x7D '}'  */
    { 0x08, 0x04, 0x08, 0x10, 0x08 }, /* 0x7E '~'  */
};

/* ── Drawing primitives ───────────────────────────────────────────────────
 *
 * These replace the equivalents that were previously defined in main.c.
 * The pitch of the video framebuffer is EPD_VIDEO_WIDTH/8 bytes per raw row.
 */

#define UI_FB_PITCH  (EPD_VIDEO_WIDTH / 8)
#define UI_SCREEN_W  160                      /* logical pixel width  */
#define UI_SCREEN_H  144                      /* logical pixel height */

static void ui_put_pixel_raw(uint8_t *fb, int x, int y, bool c)
{
    uint8_t *p    = &fb[y * UI_FB_PITCH + x / 8];
    uint8_t  mask = 0x80u >> (x % 8);
    if (c) *p |= mask;
    else   *p &= ~mask;
}

void ui_put_pixel(uint8_t *fb, int x, int y, int c)
{
    for (int i = 0; i < 3; i++)
        ui_put_pixel_raw(fb, x * 3 + i, y, i < c);
}

void ui_put_rect(uint8_t *fb, int x0, int y0, int x1, int y1, int c)
{
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            ui_put_pixel(fb, x, y, c);
}

/* ── Text rendering ───────────────────────────────────────────────────────
 *
 * Each character cell is CHAR_W × CHAR_H pixels (5-pixel glyph + 1 gap,
 * 7-pixel glyph + 1 gap row).  The gap pixels are filled with the
 * background colour so callers need not pre-clear the target area.
 */

#define CHAR_W  6   /* glyph columns (5) + 1 gap  */
#define CHAR_H  8   /* glyph rows    (7) + 1 gap  */

static void ui_draw_char(uint8_t *fb, int cx, int cy, char ch, int fg, int bg)
{
    uint8_t idx = (uint8_t)ch;
    if (idx < 0x20u || idx > 0x7Eu) idx = '?';
    const uint8_t *g = s_font5x7[idx - 0x20u];

    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++)
            ui_put_pixel(fb, cx + col, cy + row, (bits >> row) & 1 ? fg : bg);
    }
    /* 6th column: spacing gap */
    for (int row = 0; row < 7; row++)
        ui_put_pixel(fb, cx + 5, cy + row, bg);
}

/* Draw a NUL-terminated string; returns the x position after the last char. */
static int ui_draw_string(uint8_t *fb, int x, int y,
                           const char *s, int fg, int bg)
{
    while (*s && (x + CHAR_W) <= UI_SCREEN_W) {
        ui_draw_char(fb, x, y, *s++, fg, bg);
        x += CHAR_W;
    }
    return x;
}

/* ── Menu layout ──────────────────────────────────────────────────────────
 *
 * The 144-pixel screen is divided into 18 rows of 8 pixels each:
 *
 *   Row  0        : title bar  (inverted: white text on black bg)
 *   Row  1        : "^ N above" scroll indicator (or blank)
 *   Rows 2 … 15   : 14 list entries
 *   Row 16        : "v N below" scroll indicator (or blank)
 *   Row 17        : footer hint (light grey bg)
 */

#define MENU_ROWS         18
#define MENU_TITLE_ROW     0
#define MENU_SCROLL_UP     1
#define MENU_LIST_FIRST    2
#define MENU_LIST_LAST    15
#define MENU_SCROLL_DOWN  16
#define MENU_HINT_ROW     17
#define MENU_VISIBLE      (MENU_LIST_LAST - MENU_LIST_FIRST + 1)  /* 14 */

/* Fill an entire menu row, then draw text left-aligned with a 1-px indent. */
static void ui_menu_row(uint8_t *fb, int row,
                         const char *text, int fg, int bg)
{
    int y = row * CHAR_H;
    ui_put_rect(fb, 0, y, UI_SCREEN_W, y + CHAR_H, bg);
    if (text && *text)
        ui_draw_string(fb, 1, y, text, fg, bg);
}

/* ── File scanning ────────────────────────────────────────────────────────
 *
 * Scans a directory and appends regular-file paths to files[].
 * Recurses into subdirectories up to `depth` levels deep.
 * Returns the updated count.
 */

#define UI_MAX_FILES  64
#define UI_PATH_MAX  128

typedef char ui_path_t[UI_PATH_MAX];

static bool has_rom_extension(const char *path)
{
    const char *dot = strrchr(path, '.');

    if (dot == NULL) {
        return false;
    }

    return strcasecmp(dot, ".gb") == 0 || strcasecmp(dot, ".gbc") == 0;
}

static int scan_dir(const char *dir_path, ui_path_t *files,
                    int max, int count, int depth)
{
    DIR *dir = opendir(dir_path);
    if (!dir) return count;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && count < max) {
        /* Skip . and hidden entries (includes .. on FAT). */
        if (ent->d_name[0] == '.') continue;

        /* Use a generously-sized local buffer for the path being probed;
         * stored paths are then truncated to UI_PATH_MAX when saved. */
        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dir_path, ent->d_name);

        /* Resolve type: FAT VFS usually sets d_type, fall back to stat(). */
        uint8_t dtype = ent->d_type;
        if (dtype == DT_UNKNOWN) {
            struct stat st;
            if (stat(full, &st) == 0)
                dtype = S_ISDIR(st.st_mode) ? DT_DIR : DT_REG;
            else
                dtype = DT_REG;  /* treat unknowable entries as files */
        }

        if (dtype == DT_DIR) {
            if (depth > 0)
                count = scan_dir(full, files, max, count, depth - 1);
        } else {
            if (!has_rom_extension(ent->d_name)) {
                continue;
            }
            strlcpy(files[count], full, UI_PATH_MAX - 1);
            files[count][UI_PATH_MAX - 1] = '\0';
            count++;
        }
    }
    closedir(dir);
    return count;
}

/* ── Input handling ───────────────────────────────────────────────────────
 *
 * tp_read_buttons() returns the instantaneous held-down mask.
 * We track rising edges and add auto-repeat for UP / DOWN navigation.
 */

#define BTN_REPEAT_DELAY_MS  500
#define BTN_REPEAT_RATE_MS   150

static uint8_t s_prev_btns;
static uint8_t s_prev_actions;
static uint8_t s_repeat_btn;
static int64_t s_repeat_next_ms;

typedef struct {
    uint8_t buttons;
    uint8_t actions;
} ui_input_t;

static void input_reset(void)
{
    s_prev_btns      = 0;
    s_prev_actions   = 0;
    s_repeat_btn     = 0;
    s_repeat_next_ms = 0;
}

/* Returns a bitmask of GB_BTN_* events that should be acted upon this frame.
 * Handles rising-edge detection and auto-repeat for UP/DOWN. */
static ui_input_t input_poll(void)
{
    tp_state_t touch = tp_read_state();
    tp_state_t serial = serial_input_read_state();
    ui_input_t input = {0};
    touch.gb_buttons |= serial.gb_buttons;
    touch.actions    |= serial.actions;

    uint8_t cur      = touch.gb_buttons;
    uint8_t edge     = cur & (uint8_t)~s_prev_btns;          /* rising edges */
    uint8_t actions  = touch.actions & (uint8_t)~s_prev_actions;
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000);

    /* Auto-repeat: arm on first contact, fire at REPEAT_RATE after DELAY. */
    uint8_t nav = cur & (GB_BTN_UP | GB_BTN_DOWN);
    if (nav) {
        if (s_repeat_btn == 0) {
            s_repeat_btn     = nav;
            s_repeat_next_ms = now_ms + BTN_REPEAT_DELAY_MS;
        } else if (now_ms >= s_repeat_next_ms) {
            edge            |= s_repeat_btn;
            s_repeat_next_ms = now_ms + BTN_REPEAT_RATE_MS;
        }
    } else {
        s_repeat_btn = 0;
    }

    s_prev_btns = cur;
    input.buttons = edge;
    input.actions = actions;
    s_prev_actions = touch.actions;
    return input;
}

/* ── Public: ROM / file picker ────────────────────────────────────────────
 *
 * Allocates a file list in SPIRAM, scans the SD card, and runs the
 * interactive selection loop.  Uses msg_flip() for display pacing.
 */

ui_rom_pick_result_t ui_rom_picker(const char *mount_pt, char *out_path, size_t path_size)
{
    /* Allocate file list from SPIRAM to avoid stack pressure. */
    ui_path_t *files = (ui_path_t *)heap_caps_malloc(
                            UI_MAX_FILES * sizeof(ui_path_t), MALLOC_CAP_SPIRAM);
    if (!files) {
        ESP_LOGE(TAG, "Cannot allocate file list");
        return UI_ROM_PICK_NONE;
    }

    int file_count = scan_dir(mount_pt, files, UI_MAX_FILES, 0, 1 /* depth */);
    ESP_LOGI(TAG, "Found %d file(s) on SD card under %s", file_count, mount_pt);

    /* Acquire initial back-buffer from the video pipeline. */
    uint8_t *video_fb = msg_flip();

    /* ── No files: show an error message for 3 s, then return. ── */
    if (file_count == 0) {
        memset(video_fb, 0xFF, EPD_VIDEO_FB_SIZE);
        ui_menu_row(video_fb, 0,  "SD CARD",              3, 0);
        ui_menu_row(video_fb, 2,  "No files found.",      0, 3);
        ui_menu_row(video_fb, 3,  "Place .gb/.gbc files", 0, 3);
        ui_menu_row(video_fb, 4,  "on the SD card.",      0, 3);
        msg_flip();
        vTaskDelay(pdMS_TO_TICKS(3000));
        heap_caps_free(files);
        return UI_ROM_PICK_NONE;
    }

    /*
     * Fixed header items (always before ROM files):
     *   IDX 0 — "SETTINGS"        non-selectable section label
     *   IDX 1 — "Audio Engine: X" selectable
     *   IDX 2 — separator         non-selectable grey bar
     *   IDX 3 .. 3+file_count-1   ROM files
     */
#define PICKER_IDX_SETTINGS  0
#define PICKER_IDX_AUDIO     1
#define PICKER_IDX_SEP       2
#define PICKER_IDX_ROMS      3

    int total_items = PICKER_IDX_ROMS + file_count;
    int selection   = PICKER_IDX_ROMS;    /* default: first ROM file */
    int scroll_off  = 0;

    input_reset();

    while (1) {
        /* ── Render ─────────────────────────────────────────────── */
        memset(video_fb, 0xFF, EPD_VIDEO_FB_SIZE);

        /* Title bar — show battery voltage */
        char title[UI_SCREEN_W / CHAR_W + 2];
        uint32_t bat_mv = battery_read_mv();
        snprintf(title, sizeof(title), "Paperboy  BAT:%lu.%02luV",
                 bat_mv / 1000, (bat_mv % 1000) / 10);
        ui_menu_row(video_fb, MENU_TITLE_ROW, title, 3, 0);

        /* Scroll-up indicator */
        if (scroll_off > 0) {
            char ind[UI_SCREEN_W / CHAR_W + 2];
            snprintf(ind, sizeof(ind), "^ %d above", scroll_off);
            ui_menu_row(video_fb, MENU_SCROLL_UP, ind, 0, 2);
        } else {
            ui_menu_row(video_fb, MENU_SCROLL_UP, NULL, 0, 3);
        }

        /* List entries */
        for (int i = 0; i < MENU_VISIBLE; i++) {
            int list_idx = scroll_off + i;
            int row      = MENU_LIST_FIRST + i;

            if (list_idx >= total_items) {
                ui_menu_row(video_fb, row, NULL, 0, 3);
                continue;
            }

            bool sel = (list_idx == selection);
            char line[UI_SCREEN_W / CHAR_W + 2];

            if (list_idx == PICKER_IDX_SETTINGS) {
                /* Section label — non-selectable, grey background */
                ui_menu_row(video_fb, row, "SETTINGS", 0, 2);
            } else if (list_idx == PICKER_IDX_AUDIO) {
                /* Audio engine selector — selectable */
                snprintf(line, sizeof(line), "%sAudio Engine: %s",
                         sel ? "> " : "  ",
                         audio_engine_name(audio_get_engine()));
                ui_menu_row(video_fb, row, line,
                            sel ? 3 : 0,
                            sel ? 0 : 3);
            } else if (list_idx == PICKER_IDX_SEP) {
                /* "GAMES" section label — non-selectable, grey background */
                ui_menu_row(video_fb, row, "GAMES", 0, 2);
            } else {
                /* ROM file */
                const char *slash = strrchr(files[list_idx - PICKER_IDX_ROMS], '/');
                const char *fname = slash ? slash + 1 : files[list_idx - PICKER_IDX_ROMS];
                snprintf(line, sizeof(line), "%s%s",
                         sel ? "> " : "  ", fname);
                ui_menu_row(video_fb, row, line,
                            sel ? 3 : 0,
                            sel ? 0 : 3);
            }
        }

        /* Scroll-down indicator */
        int remaining = total_items - scroll_off - MENU_VISIBLE;
        if (remaining > 0) {
            char ind[UI_SCREEN_W / CHAR_W + 2];
            snprintf(ind, sizeof(ind), "v %d below", remaining);
            ui_menu_row(video_fb, MENU_SCROLL_DOWN, ind, 0, 2);
        } else {
            ui_menu_row(video_fb, MENU_SCROLL_DOWN, NULL, 0, 3);
        }

        /* Footer — context-sensitive hint */
        if (selection == PICKER_IDX_AUDIO) {
            ui_menu_row(video_fb, MENU_HINT_ROW,
                        "A:cycle sound  UP/DN:nav", 0, 2);
        } else {
            ui_menu_row(video_fb, MENU_HINT_ROW,
                        "UP/DN:scroll  A/ST:select", 0, 2);
        }

        /* Submit frame, get next back-buffer. */
        video_fb = msg_flip();

        /* ── Input ──────────────────────────────────────────────── */
        ui_input_t ev = input_poll();

        if (ev.buttons & GB_BTN_DOWN) {
            /* Advance, skipping non-selectable items. */
            int next = selection;
            do { next++; } while (next < total_items &&
                                   next != PICKER_IDX_AUDIO &&
                                   next < PICKER_IDX_ROMS);
            if (next < total_items) {
                selection = next;
                if (selection >= scroll_off + MENU_VISIBLE)
                    scroll_off = selection - MENU_VISIBLE + 1;
            }
        }

        if (ev.buttons & GB_BTN_UP) {
            /* Retreat, skipping non-selectable items. */
            int next = selection;
            do { next--; } while (next >= 0 &&
                                   next != PICKER_IDX_AUDIO &&
                                   next < PICKER_IDX_ROMS);
            if (next >= 0) {
                selection = next;
                if (selection < scroll_off)
                    scroll_off = selection;
            }
        }

        if (ev.buttons & (GB_BTN_A | GB_BTN_START)) {
            if (selection == PICKER_IDX_AUDIO) {
                /* Cycle sound engine: PCM → Poly → Mute → PCM … */
                audio_engine_t next = (audio_engine_t)
                    ((audio_get_engine() + 1) % AUDIO_ENGINE_COUNT);
                audio_set_engine(next);
            } else if (selection >= PICKER_IDX_ROMS) {
                /* A ROM file was confirmed — return it. */
                snprintf(out_path, path_size, "%s",
                         files[selection - PICKER_IDX_ROMS]);
                heap_caps_free(files);
                return UI_ROM_PICK_SELECTED;
            }
        }

        if (ev.actions & TP_ACTION_LOAD) {
            heap_caps_free(files);
            return UI_ROM_PICK_LOAD_LAST;
        }
    }

#undef PICKER_IDX_SETTINGS
#undef PICKER_IDX_AUDIO
#undef PICKER_IDX_SEP
#undef PICKER_IDX_ROMS
}

void ui_show_notice(const char *title, const char *message, uint32_t duration_ms)
{
    uint8_t *video_fb = msg_flip();

    memset(video_fb, 0xFF, EPD_VIDEO_FB_SIZE);
    ui_menu_row(video_fb, MENU_TITLE_ROW, title, 3, 0);
    ui_menu_row(video_fb, 4, NULL, 0, 3);
    ui_menu_row(video_fb, 5, message, 0, 3);
    ui_menu_row(video_fb, 6, NULL, 0, 3);
    msg_flip();

    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

void ui_clear_ghosting(void)
{
    uint8_t *fb = msg_flip();   /* submit current frame, get fresh back-buffer */

    /* 5 frames of black */
    for (int i = 0; i < 5; i++) {
        memset(fb, 0x00, EPD_VIDEO_FB_SIZE);
        fb = msg_flip();
    }

    /* 5 frames of white */
    for (int i = 0; i < 5; i++) {
        memset(fb, 0xFF, EPD_VIDEO_FB_SIZE);
        fb = msg_flip();
    }
}

void ui_draw_bat_low_overlay(uint8_t *fb)
{
    ui_menu_row(fb, MENU_TITLE_ROW, "!! BATTERY LOW !!", 3, 0);
}
