/*
 * gbemu.c — PICboy core integration for Paperboy / M5PaperS3
 *
 * Wraps PICboy (a fast GB/GBC emulator for microcontrollers) behind the same
 * paperboy_gb_* interface that main.c depends on.  The WalnutGB back-end has
 * been replaced; see gbemu_walnut.c.bak for the old implementation.
 *
 * Unity-build: PICboy.c is #included directly so the compiler sees the whole
 * emulator as one translation unit, enabling maximum inlining and register
 * allocation across the hot loop.
 */

#define PICBOY_EMBEDDED  /* suppress desktop (GL/AL/GLFW) code in PICboy.c */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_attr.h"

#include "gbemu.h"
#include "audio.h"
#include "profiler.h"

/* ---- PICboy unity build ---- */
#include "PICboy.c"

/* ------------------------------------------------------------------ */

static const char *TAG = "gbemu";
static bool        s_ready        = false;
static char        s_last_error[128];

/*
 * Grayscale DMG palette in RGB555 format (PICboy native).
 *   Index 0 = white (lightest), index 3 = black (darkest)
 * These values map exactly to EPD shades 0-3 via the luminance formula:
 *   shade = 3 - (((r+g+b)/3) >> 3)
 */
static const uint16_t s_dmg_gray[4] = {
    0x7FFF,  /* R=G=B=31  lum=31  shade=0 (white)       */
    0x56B5,  /* R=G=B=21  lum=21  shade=1 (light grey)  */
    0x294A,  /* R=G=B=10  lum=10  shade=2 (dark grey)   */
    0x0000,  /* R=G=B=0   lum=0   shade=3 (black)       */
};

/* ------------------------------------------------------------------ */

bool paperboy_gb_init(const uint8_t *rom, size_t rom_size)
{
    s_ready = false;
    strlcpy(s_last_error, "not initialized", sizeof(s_last_error));

    if (rom == NULL || rom_size < 0x150) {
        strlcpy(s_last_error, "rom buffer missing or too small", sizeof(s_last_error));
        return false;
    }

    /* Point the PICboy ROM "array" at the caller's buffer (already in SRAM
     * or PSRAM, allocated by main.c).  PICboy.c declares this as a plain
     * pointer in PICBOY_EMBEDDED mode so array accesses go through the MMU. */
    gb_mem_rom = (unsigned char *)rom;

    /* Reset emulator mode so gb_initialize() auto-detects GBC/DMG from the
     * ROM header byte 0x0143. */
    gb_mode = UNK;

    /* Build the DMG colour-palette look-up table (needed before init). */
    gb_palette_arrange();

    /* Zero cart RAM so saves don't contain garbage. */
    memset(gb_mem_eram, 0, sizeof(gb_mem_eram));

    /* Zero the screen buffer. */
    memset(gb_game_screen_buffer, 0, sizeof(gb_game_screen_buffer));

    /* Initialise the emulator core (sets gb_mode, MBC type, registers …).
     * Returns 0 for unsupported configurations but still populates gb_cart_mbc. */
    gb_initialize();

    if (gb_cart_mbc == 0xFF) {
        strlcpy(s_last_error, "unsupported cart / MBC type", sizeof(s_last_error));
        return false;
    }

    /* For DMG mode, override the default pea-soup palette with a clean
     * greyscale ramp so the EPD luminance conversion gives correct shades. */
    if (gb_mode == DMG) {
        for (int i = 0; i < 4; i++) {
            gb_palette_defined[i]     = s_dmg_gray[i]; /* BGP  */
            gb_palette_defined[i + 4] = s_dmg_gray[i]; /* OBP0 */
            gb_palette_defined[i + 8] = s_dmg_gray[i]; /* OBP1 */
        }
    }

    /* Initialise I2S audio output; PICboy fills its own audio buffer and
     * we feed samples to the ring buffer once per frame. */
    audio_init();

    ESP_LOGI(TAG, "PICboy initialised: mode=%s  MBC=%lu  ROM=%u B",
             gb_mode == GBC ? "GBC" : "DMG",
             gb_cart_mbc,
             (unsigned)rom_size);

    s_ready = true;
    strlcpy(s_last_error, "ok", sizeof(s_last_error));
    return true;
}

/* ------------------------------------------------------------------ */

void paperboy_gb_set_buttons(uint8_t pressed_mask)
{
    if (!s_ready) return;

    /* PICboy button byte: 0 = pressed, 1 = released.
     * GB_BTN_* in gbemu.h uses 1 = pressed — so we invert.
     * Bit layout is identical between the two conventions. */
    gb_game_buttons_previous = gb_game_buttons_current;
    gb_game_buttons_current  = (uint8_t)(~pressed_mask);
}

/* ------------------------------------------------------------------ */

/*
 * EPD rendering: convert the PICboy RGB555 frame-buffer to 1-bpp EPD video FB.
 *
 * gb_game_screen_buffer layout:  [y * GB_LCD_WIDTH + x], y=0..143, x=0..159
 * EPD video FB:                  432 columns × 160 rows, 1 bpp, pitch = 54 B
 *
 *   • 90° CW rotation: GB (x, y) → EPD col = (143-y)*3,  EPD row = x
 *   • 3× horizontal scale via the dither pattern
 *   • Luminance → 4-level shade → 3-pixel Bayer pattern
 */
static IRAM_ATTR void picboy_render_epd(uint8_t *fb)
{
    /* 3-bit left-aligned dither pattern per shade (0=white … 3=black). */
    static const uint16_t dithermap[4] = { 0x0000u, 0x2000u, 0x6000u, 0xe000u };

    for (int line = 0; line < GB_LCD_HEIGHT; line++) {

        const int      target_x = (GB_LCD_HEIGHT - 1 - line) * 3;
        uint8_t *const row_base = fb + (target_x >> 3);
        const int      offset_x = target_x & 7;
        const bool     crossing = (offset_x >= 6);
        const uint32_t stride   = crossing ? 53u : 54u;

        /* Pre-compute masks once per column (= per GB scanline). */
        uint8_t clrmask, clrmask_x;
        uint8_t setmask[4], setmask_x[4];

        uint16_t tmp = (uint16_t)(0xe000u >> offset_x);
        clrmask   = ~(uint8_t)(tmp >> 8);
        clrmask_x = ~(uint8_t)(tmp & 0xffu);

        for (int k = 0; k < 4; k++) {
            tmp          = (uint16_t)(dithermap[k] >> offset_x);
            setmask[k]   = (uint8_t)(tmp >> 8);
            setmask_x[k] = (uint8_t)(tmp & 0xffu);
        }

        const uint16_t *srcrow = &gb_game_screen_buffer[line * GB_LCD_WIDTH];
        uint8_t        *wrptr  = row_base;

        if (!crossing) {
            for (int i = 0; i < GB_LCD_WIDTH; i++) {
                const uint16_t px    = srcrow[i];
                const unsigned r     = (px >> 10) & 0x1fu;
                const unsigned g     = (px >>  5) & 0x1fu;
                const unsigned b     =  px        & 0x1fu;
                const unsigned lum   = (r + g + b) / 3u;
                const unsigned shade = 3u - (lum >> 3);

                uint8_t bv = *wrptr;
                bv = (uint8_t)((bv & clrmask) | setmask[shade]);
                *wrptr = bv;
                wrptr += stride;
            }
        } else {
            for (int i = 0; i < GB_LCD_WIDTH; i++) {
                const uint16_t px    = srcrow[i];
                const unsigned r     = (px >> 10) & 0x1fu;
                const unsigned g     = (px >>  5) & 0x1fu;
                const unsigned b     =  px        & 0x1fu;
                const unsigned lum   = (r + g + b) / 3u;
                const unsigned shade = 3u - (lum >> 3);

                uint8_t bv = *wrptr;
                bv = (uint8_t)((bv & clrmask) | setmask[shade]);
                *wrptr++ = bv;
                bv = *wrptr;
                bv = (uint8_t)((bv & clrmask_x) | setmask_x[shade]);
                *wrptr = bv;
                wrptr += stride;
            }
        }
    }
}

/* ------------------------------------------------------------------ */

bool paperboy_gb_run_frame(uint8_t *fb, bool skip_render)
{
    if (!s_ready) return false;

    /* Run PICboy until it sets gb_game_draw (= VBlank reached). */
    gb_game_draw = 0;

    while (!gb_game_draw) {
        gb_run();
        gb_updates();
        gb_interrupts();
    }

    /* RTC: tick once per emulated second (every 60 frames). */
    gb_ext_rtc_counter++;
    if (gb_ext_rtc_counter >= 60) {
        gb_ext_rtc_counter = 0;
        gb_clock();
    }

    /* Feed PICboy's mono audio buffer to the I2S ring as stereo pairs.
     *
     * In embedded mode gb_game_audio_section never toggles (that happens
     * inside openal_play() which is compiled out).  The audio buffer is a
     * circular ring of AUDIO_LEN unsigned-16 samples; gb_game_audio_write
     * is the next-write index.  We track the last position we read from and
     * push only the newly-produced samples each frame (~546 per frame). */
    {
        static int16_t     s_stereo[AUDIO_LEN * 2];
        static unsigned int s_audio_read = 0;

        const unsigned int w = gb_game_audio_write;
        unsigned int count;

        if (w >= s_audio_read)
            count = w - s_audio_read;
        else
            count = AUDIO_LEN - s_audio_read + w;

        if (count > AUDIO_LEN) count = AUDIO_LEN; /* sanity */

        for (unsigned int i = 0; i < count; i++) {
            const unsigned int idx = (s_audio_read + i) % AUDIO_LEN;
            const int16_t s = (int16_t)((int32_t)gb_game_audio_buffer[idx] - 0x8000);
            s_stereo[i * 2]     = s;
            s_stereo[i * 2 + 1] = s;
        }
        s_audio_read = w;

        if (count > 0)
            audio_push_samples(s_stereo, count);
    }

    /* Render the completed frame to the EPD video framebuffer. */
    if (!skip_render && fb != NULL) {
        PROF_BEGIN(PROF_LCD);
        picboy_render_epd(fb);
        PROF_END(PROF_LCD);
    }

    return true;
}

/* ------------------------------------------------------------------ */

bool paperboy_gb_is_ready(void)
{
    return s_ready;
}

const char *paperboy_gb_last_error(void)
{
    return s_last_error;
}
