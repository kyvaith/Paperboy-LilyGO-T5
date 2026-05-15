#include "paperboy_gb.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "peanut_gb.h"

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

static uint8_t gb_rom_read_cb(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;

    if (addr < s_rom_size) {
        return s_rom[addr];
    }

    return 0xFF;
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

    gb_init_lcd(&s_gb, lcd_draw_line_cb);
    s_gb.direct.joypad = 0xFF;

    ESP_LOGI(TAG, "Peanut-GB initialized. ROM title: %s", gb_get_rom_name(&s_gb, rom_name));

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

bool paperboy_gb_run_frame(uint8_t *fb)
{
    if (!s_ready) {
        return false;
    }

    s_framebuffer = fb;
    memset(s_framebuffer, 0, 160*144/8); // debug

    gb_run_frame(&s_gb);

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
