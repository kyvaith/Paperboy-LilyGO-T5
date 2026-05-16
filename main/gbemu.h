#ifndef PAPERBOY_GB_H
#define PAPERBOY_GB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GB_LCD_WIDTH    160
#define GB_LCD_HEIGHT   144

#define GB_BTN_A        0x01
#define GB_BTN_B        0x02
#define GB_BTN_SELECT   0x04
#define GB_BTN_START    0x08
#define GB_BTN_RIGHT    0x10
#define GB_BTN_LEFT     0x20
#define GB_BTN_UP       0x40
#define GB_BTN_DOWN     0x80

bool paperboy_gb_init(const uint8_t *rom, size_t rom_size);
void paperboy_gb_set_buttons(uint8_t pressed_mask);
// Run one GB frame.  If skip_render is true, lcd_draw_line callbacks are
// suppressed (saving ~10-15 ms); fb may be NULL in that case.
bool paperboy_gb_run_frame(uint8_t *fb, bool skip_render);
bool paperboy_gb_has_persist(void);
size_t paperboy_gb_persist_size(void);
bool paperboy_gb_persist_is_dirty(void);
bool paperboy_gb_persist_export(uint8_t *dst, size_t dst_size, uint32_t timestamp);
bool paperboy_gb_persist_import(const uint8_t *src, size_t src_size, uint32_t current_timestamp);
void paperboy_gb_persist_mark_clean(void);
bool paperboy_gb_is_ready(void);
const char *paperboy_gb_last_error(void);

#endif
