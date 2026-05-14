#ifndef PAPERBOY_GB_H
#define PAPERBOY_GB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PAPERBOY_GB_LCD_WIDTH 160
#define PAPERBOY_GB_LCD_HEIGHT 144

#define PAPERBOY_GB_BTN_A 0x01
#define PAPERBOY_GB_BTN_B 0x02
#define PAPERBOY_GB_BTN_SELECT 0x04
#define PAPERBOY_GB_BTN_START 0x08
#define PAPERBOY_GB_BTN_RIGHT 0x10
#define PAPERBOY_GB_BTN_LEFT 0x20
#define PAPERBOY_GB_BTN_UP 0x40
#define PAPERBOY_GB_BTN_DOWN 0x80

bool paperboy_gb_init(const uint8_t *rom, size_t rom_size);
void paperboy_gb_set_buttons(uint8_t pressed_mask);
bool paperboy_gb_run_frame(void);
const uint8_t *paperboy_gb_get_framebuffer(void);
bool paperboy_gb_copy_framebuffer(uint8_t *out, size_t out_size, uint32_t *out_frame_id);
bool paperboy_gb_is_ready(void);
const char *paperboy_gb_last_error(void);

#endif
