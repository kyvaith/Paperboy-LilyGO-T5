#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include <esp_attr.h>

#include "gbemu.h"
#include "audio.h"

/* Walnut-CGB feature flags – must be set before including walnut_cgb.h */
#define WALNUT_FULL_GBC_SUPPORT 0   /* DMG-only, no CGB colour mode */
#define WALNUT_GB_12_COLOUR     0   /* must match WALNUT_FULL_GBC_SUPPORT */
#define WALNUT_GB_32BIT_DMA     1   /* ESP32-S3 handles unaligned 32-bit fine */
#define ENABLE_SOUND            1   /* Enable minigb_apu audio */
#define ENABLE_LCD              1

/* Forward declarations for audio callbacks used by walnut_cgb.h */
uint8_t audio_read(uint16_t addr);
void audio_write(uint16_t addr, uint8_t val);

#include "walnut_cgb.h"

static const char *TAG = "gbemu";

static struct gb_s s_gb;
static const uint8_t *s_rom;
static size_t s_rom_size;
static uint8_t *s_cart_ram;
static size_t s_cart_ram_size;
static uint8_t *s_framebuffer;
static bool s_ready;
static char s_last_error[128];

static void set_last_error(const char *msg)
{
    strlcpy(s_last_error, msg, sizeof(s_last_error));
}

/* Audio I/O callbacks – called by Walnut-CGB when accessing APU registers.
 * The APU state lives in audio.c; route through its public API. */
uint8_t audio_read(uint16_t addr)
{
    return audio_apu_read(addr);
}

void audio_write(uint16_t addr, uint8_t val)
{
    audio_apu_write(addr, val);
}

static uint8_t gb_rom_read_cb(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;

    if (addr < s_rom_size) {
        return s_rom[addr];
    }

    return 0xFF;
}

static uint16_t gb_rom_read16_cb(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;

    const uint8_t *src = &s_rom[addr];
    // Alignment check, not required for all platforms. ESP32 series mcu flash memory and psram sources *require* this
    if ((uintptr_t)src & 1) {
        // fallback to safe 8-bit reads when not aligned
        return ((uint16_t)src[0]) | ((uint16_t)src[1] << 8);          
    } 
    return *(uint16_t *)src;
}

static uint32_t gb_rom_read32_cb(struct gb_s *gb, const uint_fast32_t addr)
{
    const uint8_t *src = &s_rom[addr];

    // Alignment check: ESP32 flash / PSRAM require 32-bit alignment
    if ((uintptr_t)src & 3) {
        // fallback to safe 8-bit reads when not aligned
        return ((uint32_t)src[0]) |
               ((uint32_t)src[1] << 8) |
               ((uint32_t)src[2] << 16) |
               ((uint32_t)src[3] << 24);
    }

    return *(uint32_t *)src;
}

static uint8_t gb_cart_ram_read_cb(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;

    if (s_cart_ram == NULL || s_cart_ram_size == 0 || addr >= s_cart_ram_size) {
        return 0xFF;
    }

    return s_cart_ram[addr];
}

static void gb_cart_ram_write_cb(struct gb_s *gb, const uint_fast32_t addr, const uint8_t val)
{
    (void)gb;

    if (s_cart_ram == NULL || s_cart_ram_size == 0 || addr >= s_cart_ram_size) {
        return;
    }

    s_cart_ram[addr] = val;
}

static void gb_error_cb(struct gb_s *gb, const enum gb_error_e err, const uint16_t addr)
{
    (void)gb;

    ESP_LOGE(TAG, "Emulator error=%d at addr=0x%04x", (int)err, addr);
    set_last_error("runtime core error");
}

static void lcd_draw_line_cb(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line)
{
    (void)gb;

    if (line >= GB_LCD_HEIGHT) {
        return;
    }

    int target_x = (143 - line) * 3;
    uint8_t *wrptr = &(s_framebuffer[target_x / 8]);
    // We need to write 3 pixels, which might cross the byte boundary (each byte holds 8 pixels)
    // This depends on the line being drawn
    int offset_x = target_x % 8;
    bool crossing = offset_x >= 6;
    uint32_t stride = crossing ? 53 : 54;
    uint8_t clrmask;
    uint8_t setmask[4];
    uint8_t clrmask_crossing;
    uint8_t setmask_crossing[4];

    const uint16_t dithermap[4] = {0x0000, 0x2000, 0x6000, 0xe000};
    uint16_t tmp;
    tmp = 0xe000 >> offset_x;
    clrmask = ~(tmp >> 8);
    clrmask_crossing = ~(tmp & 0xff);
    for (int i = 0; i < 4; i++) {
        tmp = dithermap[i] >> offset_x;
        setmask[i] = tmp >> 8;
        setmask_crossing[i] = tmp & 0xff;
    }
    if (!crossing) {
        for (int i = 0; i < 160; i++) {
            uint8_t b = *wrptr;
            b &= clrmask;
            b |= setmask[pixels[i]];
            *wrptr = b;
            wrptr += stride;
        }
    }
    else {
        for (int i = 0; i < 160; i++) {
            uint8_t b = *wrptr;
            b &= clrmask;
            b |= setmask[pixels[i]];
            *wrptr++ = b;
            b = *wrptr;
            b &= clrmask_crossing;
            b |= setmask_crossing[pixels[i]];
            *wrptr = b;
            wrptr += stride;
        }
    }
}

bool paperboy_gb_init(const uint8_t *rom, size_t rom_size)
{
    enum gb_init_error_e init_err;
    size_t save_size = 0;
    char rom_name[17];

    s_ready = false;
    set_last_error("not initialized");

    if (rom == NULL || rom_size < 0x150) {
        set_last_error("rom buffer missing or too small");
        return false;
    }

    memset(&s_gb, 0, sizeof(s_gb));

    s_rom = rom;
    s_rom_size = rom_size;

    free(s_cart_ram);
    s_cart_ram = NULL;
    s_cart_ram_size = 0;

    init_err = gb_init(&s_gb,
                       gb_rom_read_cb,
                       gb_rom_read16_cb,
                       gb_rom_read32_cb,
                       gb_cart_ram_read_cb,
                       gb_cart_ram_write_cb,
                       gb_error_cb,
                       NULL);

    if (init_err != GB_INIT_NO_ERROR) {
        ESP_LOGE(TAG, "gb_init failed: %d", (int)init_err);
        set_last_error("gb_init failed");
        return false;
    }

    if (gb_get_save_size_s(&s_gb, &save_size) == 0 && save_size > 0) {
        s_cart_ram = calloc(1, save_size);
        if (s_cart_ram == NULL) {
            set_last_error("cart ram alloc failed");
            return false;
        }
        s_cart_ram_size = save_size;
    }

    #if ENABLE_LCD
    gb_init_lcd(&s_gb, lcd_draw_line_cb);
    #endif
    s_gb.direct.joypad = 0xFF;

    /* Initialize audio system (also inits the APU) */
    audio_init();

    ESP_LOGI(TAG, "Walnut-CGB initialized. ROM title: %s", gb_get_rom_name(&s_gb, rom_name));

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
    if (!s_ready) {
        return false;
    }

    if (skip_render) {
        /* Tell the emulator core to skip all pixel rendering this frame.
         * __gb_draw_line() returns immediately when frame_skip is true and
         * frame_skip_count is 0, skipping all sprite/background work. */
        s_gb.direct.frame_skip = true;
        s_gb.display.frame_skip_count = 0;
    } else {
        s_framebuffer = fb;
        memset(s_framebuffer, 0, 160*144/8); // debug
    }

    gb_run_frame_dualfetch(&s_gb);

    /* Restore frame_skip to off so normal rendering resumes next frame. */
    s_gb.direct.frame_skip = false;
    s_gb.display.frame_skip_count = 0;

    /* Audio synthesis is now driven by the audio task at the I2S hardware
     * clock rate, decoupled from emulation speed.  Nothing to do here. */

    return true;
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
