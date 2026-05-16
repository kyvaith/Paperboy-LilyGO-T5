#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include <esp_attr.h>

#include "gbemu.h"
#include "audio.h"
#include "msg/msg.h"
#include "profiler.h"

#define PGB_CGB                 0
#define ENABLE_SOUND            1
#define ENABLE_LCD              1
#define PGB_IMPL

uint8_t audio_read(void *audio, const uint16_t addr);
void audio_write(void *audio, const uint16_t addr, const uint8_t val);

#include "crankboy_core/peanut_gb.h"

static const char *TAG = "gbemu";

int preferences_cgb_speed;
int preferences_ppu_timing;
int audio_enabled;

static gb_s s_gb;
static uint8_t *s_rom;
static size_t s_rom_size;
static uint8_t *s_cart_ram;
static size_t s_cart_ram_size;
static uint8_t s_lcd[LCD_BUFFER_BYTES] __attribute__((aligned(32)));
static uint8_t s_previous_lcd[LCD_BUFFER_BYTES] __attribute__((aligned(32)));
static uint8_t s_wram[WRAM_SIZE_CGB] __attribute__((aligned(32)));
static uint8_t s_vram[VRAM_SIZE_CGB] __attribute__((aligned(32)));
static uint8_t s_epd_shadow[EPD_VIDEO_FB_SIZE] __attribute__((aligned(32)));
static uint8_t *s_framebuffer;
static bool s_ready;
static char s_last_error[128];

struct persist_header {
    uint8_t magic[4];
    uint8_t version;
    uint8_t has_cart_ram;
    uint8_t has_rtc;
    uint8_t reserved;
    uint32_t cart_ram_size;
    uint32_t timestamp;
};

#define PERSIST_VERSION 1u
static const uint8_t PERSIST_MAGIC[4] = {'P', 'B', 'S', 'V'};

uint8_t audio_read(void *audio, const uint16_t addr)
{
    (void)audio;
    return audio_apu_read(addr);
}

void audio_write(void *audio, const uint16_t addr, const uint8_t val)
{
    (void)audio;
    audio_apu_write(addr, val);
}

void __gb_on_breakpoint(gb_s *gb, int breakpoint_number)
{
    (void)gb;
    (void)breakpoint_number;
}

static void set_last_error(const char *msg)
{
    strlcpy(s_last_error, msg, sizeof(s_last_error));
}

static bool gb_has_persist_data(void)
{
    return s_cart_ram_size > 0 || s_gb.cart_battery;
}

static size_t gb_persist_rtc_size(void)
{
    return s_gb.cart_battery ? sizeof(s_gb.cart_rtc) : 0u;
}

static inline uint8_t gb_lcd_get_pixel(const uint8_t *lcd, unsigned x, unsigned y)
{
    const uint8_t packed = lcd[(y * LCD_WIDTH_PACKED) + (x >> 2)];
    const unsigned shift = (x & 0x3u) << 1;

    return (packed >> shift) & 0x03u;
}

static bool lcd_line_changed(unsigned line)
{
    const size_t row_offset = line * LCD_WIDTH_PACKED;

    return memcmp(&s_lcd[row_offset], &s_previous_lcd[row_offset], LCD_WIDTH_PACKED) != 0;
}

static void copy_lcd_line(unsigned line)
{
    const size_t row_offset = line * LCD_WIDTH_PACKED;

    memcpy(&s_previous_lcd[row_offset], &s_lcd[row_offset], LCD_WIDTH_PACKED);
}

static void blit_lcd_to_epd(uint8_t *fb, const uint16_t *dirty_lines)
{
    const uint16_t dithermap[4] = {0x0000, 0x2000, 0x6000, 0xe000};

    for (unsigned line = 0; line < GB_LCD_HEIGHT; line++) {
        if (dirty_lines != NULL && ((dirty_lines[line >> 4] >> (line & 0xFu)) & 1u) == 0u) {
            continue;
        }

        const unsigned target_x = (GB_LCD_HEIGHT - 1u - line) * 3u;
        uint8_t *wrptr = &(fb[target_x / 8u]);
        const unsigned offset_x = target_x % 8u;
        const bool crossing = offset_x >= 6u;
        const uint32_t stride = crossing ? 53u : 54u;
        const uint16_t clear_tmp = 0xe000u >> offset_x;
        const uint8_t clrmask = (uint8_t)~(clear_tmp >> 8);
        const uint8_t clrmask_crossing = (uint8_t)~(clear_tmp & 0xffu);
        uint8_t setmask[4];
        uint8_t setmask_crossing[4];

        for (int shade = 0; shade < 4; shade++) {
            const uint16_t tmp = dithermap[shade] >> offset_x;
            setmask[shade] = (uint8_t)(tmp >> 8);
            setmask_crossing[shade] = (uint8_t)(tmp & 0xffu);
        }

        if (!crossing) {
            for (unsigned x = 0; x < GB_LCD_WIDTH; x++) {
                uint8_t value = *wrptr;
                const uint8_t pixel = gb_lcd_get_pixel(s_lcd, x, line);

                value &= clrmask;
                value |= setmask[pixel];
                *wrptr = value;
                wrptr += stride;
            }
            continue;
        }

        for (unsigned x = 0; x < GB_LCD_WIDTH; x++) {
            uint8_t value = *wrptr;
            const uint8_t pixel = gb_lcd_get_pixel(s_lcd, x, line);

            value &= clrmask;
            value |= setmask[pixel];
            *wrptr++ = value;

            value = *wrptr;
            value &= clrmask_crossing;
            value |= setmask_crossing[pixel];
            *wrptr = value;
            wrptr += stride;
        }
    }
}

static void gb_error_cb(gb_s *gb, const enum gb_error_e err, const uint16_t addr)
{
    (void)gb;

    ESP_LOGE(TAG, "Emulator error=%d at addr=0x%04x", (int)err, addr);
    set_last_error("runtime core error");
}

bool paperboy_gb_init(const uint8_t *rom, size_t rom_size)
{
    enum gb_init_error_e init_err;
    size_t save_size;
    char rom_name[17];

    s_ready = false;
    set_last_error("not initialized");

    if (rom == NULL || rom_size < 0x150) {
        set_last_error("rom buffer missing or too small");
        return false;
    }

    memset(&s_gb, 0, sizeof(s_gb));
    memset(s_lcd, 0, sizeof(s_lcd));
    memset(s_previous_lcd, 0, sizeof(s_previous_lcd));
    memset(s_wram, 0, sizeof(s_wram));
    memset(s_vram, 0, sizeof(s_vram));
    memset(s_epd_shadow, 0, sizeof(s_epd_shadow));

    s_rom = (uint8_t *)rom;
    s_rom_size = rom_size;

    free(s_cart_ram);
    s_cart_ram = NULL;
    s_cart_ram_size = 0;

    init_err = gb_init(&s_gb,
                       s_wram,
                       s_vram,
                       s_lcd,
                       s_rom,
                       s_rom_size,
                       gb_error_cb,
                       NULL,
                       false);

    if (init_err != GB_INIT_NO_ERROR && init_err != GB_INIT_NO_ERROR_BUT_REQUIRES_CGB) {
        ESP_LOGE(TAG, "gb_init failed: %d", (int)init_err);
        set_last_error("gb_init failed");
        return false;
    }

    audio_init();
    gb_reset(&s_gb, false);

    save_size = gb_get_save_size(&s_gb);

    if (save_size > 0) {
        s_cart_ram = calloc(1, save_size);
        if (s_cart_ram == NULL) {
            set_last_error("cart ram alloc failed");
            return false;
        }
        s_cart_ram_size = save_size;
        s_gb.gb_cart_ram = s_cart_ram;
        s_gb.gb_cart_ram_size = save_size;
    }

    gb_init_lcd(&s_gb);
    s_gb.direct.joypad = 0xFF;
    audio_enabled = 1;
    s_gb.direct.sram_updated = 0;
    s_gb.direct.sram_dirty = 0;

    ESP_LOGI(TAG, "CrankBoy core initialized. ROM title: %s", gb_get_rom_name(s_rom, rom_name));

    s_ready = true;
    set_last_error("ok");
    return true;
}

void paperboy_gb_set_buttons(uint8_t pressed_mask)
{
    if (!s_ready) {
        return;
    }

    s_gb.direct.joypad = (uint8_t)(~pressed_mask);
}

bool paperboy_gb_run_frame(uint8_t *fb, bool skip_render)
{
    void (*run_frame)(gb_s *) = gb_run_frame__dmg;
    uint16_t dirty_lines[GB_LCD_HEIGHT / 16] = {0};
    bool any_dirty = false;

    if (!s_ready) {
        return false;
    }

    s_gb.direct.frame_skip = skip_render;

    if (!skip_render) {
        s_framebuffer = fb;
    }

    run_frame(&s_gb);

    s_gb.direct.frame_skip = false;

    if (!skip_render) {
        PROF_BEGIN(PROF_LCD);
        for (unsigned line = 0; line < GB_LCD_HEIGHT; line++) {
            if (!lcd_line_changed(line)) {
                continue;
            }

            dirty_lines[line >> 4] |= (uint16_t)(1u << (line & 0xFu));
            copy_lcd_line(line);
            any_dirty = true;
        }

        if (any_dirty) {
            blit_lcd_to_epd(s_epd_shadow, dirty_lines);
        }
        memcpy(s_framebuffer, s_epd_shadow, sizeof(s_epd_shadow));
        PROF_END(PROF_LCD);
    }

    return true;
}

bool paperboy_gb_has_persist(void)
{
    if (!s_ready) {
        return false;
    }

    return gb_has_persist_data();
}

size_t paperboy_gb_persist_size(void)
{
    if (!s_ready || !gb_has_persist_data()) {
        return 0;
    }

    return sizeof(struct persist_header) + s_cart_ram_size + gb_persist_rtc_size();
}

bool paperboy_gb_persist_is_dirty(void)
{
    if (!s_ready || !gb_has_persist_data()) {
        return false;
    }

    return s_gb.direct.sram_updated != 0;
}

bool paperboy_gb_persist_export(uint8_t *dst, size_t dst_size, uint32_t timestamp)
{
    struct persist_header header;
    uint8_t *cursor;

    if (!s_ready || !gb_has_persist_data() || dst == NULL) {
        set_last_error("persist export unavailable");
        return false;
    }

    if (dst_size < paperboy_gb_persist_size()) {
        set_last_error("persist export buffer too small");
        return false;
    }

    memcpy(header.magic, PERSIST_MAGIC, sizeof(header.magic));
    header.version = PERSIST_VERSION;
    header.has_cart_ram = (uint8_t)(s_cart_ram_size > 0);
    header.has_rtc = (uint8_t)(s_gb.cart_battery != 0);
    header.reserved = 0;
    header.cart_ram_size = (uint32_t)s_cart_ram_size;
    header.timestamp = timestamp;

    memcpy(dst, &header, sizeof(header));
    cursor = dst + sizeof(header);

    if (s_cart_ram_size > 0) {
        memcpy(cursor, s_cart_ram, s_cart_ram_size);
        cursor += s_cart_ram_size;
    }

    if (s_gb.cart_battery) {
        memcpy(cursor, s_gb.cart_rtc, sizeof(s_gb.cart_rtc));
    }

    set_last_error("ok");
    return true;
}

bool paperboy_gb_persist_import(const uint8_t *src, size_t src_size, uint32_t current_timestamp)
{
    struct persist_header header;
    const uint8_t *cursor;

    if (!s_ready || !gb_has_persist_data() || src == NULL) {
        set_last_error("persist import unavailable");
        return false;
    }

    if (src_size < sizeof(header)) {
        set_last_error("persist blob too small");
        return false;
    }

    memcpy(&header, src, sizeof(header));
    if (memcmp(header.magic, PERSIST_MAGIC, sizeof(header.magic)) != 0 ||
        header.version != PERSIST_VERSION) {
        set_last_error("persist blob format mismatch");
        return false;
    }

    if ((header.has_cart_ram != 0) != (s_cart_ram_size > 0) || header.cart_ram_size != s_cart_ram_size) {
        set_last_error("persist cart ram mismatch");
        return false;
    }

    if ((header.has_rtc != 0) != (s_gb.cart_battery != 0)) {
        set_last_error("persist rtc mismatch");
        return false;
    }

    if (src_size != sizeof(header) + header.cart_ram_size + (header.has_rtc ? sizeof(s_gb.cart_rtc) : 0u)) {
        set_last_error("persist blob size mismatch");
        return false;
    }

    cursor = src + sizeof(header);
    if (header.cart_ram_size > 0) {
        memcpy(s_cart_ram, cursor, header.cart_ram_size);
        cursor += header.cart_ram_size;
    }

    if (header.has_rtc) {
        memcpy(s_gb.cart_rtc, cursor, sizeof(s_gb.cart_rtc));
        if (header.timestamp > 0 && current_timestamp >= header.timestamp) {
            gb_catch_up_rtc_direct(&s_gb, current_timestamp - header.timestamp);
        }
        memcpy(s_gb.latched_rtc, s_gb.cart_rtc, sizeof(s_gb.latched_rtc));
    }

    s_gb.direct.sram_updated = 0;
    s_gb.direct.sram_dirty = 0;
    set_last_error("ok");
    return true;
}

void paperboy_gb_persist_mark_clean(void)
{
    if (!s_ready) {
        return;
    }

    s_gb.direct.sram_updated = 0;
    s_gb.direct.sram_dirty = 0;
}

const uint8_t *paperboy_gb_get_framebuffer(void)
{
    if (!s_ready) {
        return NULL;
    }

    return s_framebuffer;
}

bool paperboy_gb_is_ready(void)
{
    return s_ready;
}

const char *paperboy_gb_last_error(void)
{
    return s_last_error;
}
