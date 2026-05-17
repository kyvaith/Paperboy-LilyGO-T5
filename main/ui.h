#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Drawing primitives ──────────────────────────────────────────────────── *
 *
 * Coordinate system (logical pixels in the video framebuffer):
 *   x : 0 .. 159   (horizontal, left → right)
 *   y : 0 .. 143   (vertical,   top  → bottom)
 *
 * Pixel intensity c:
 *   0 = black, 1 = dark-grey, 2 = light-grey, 3 = white
 *
 * The EPD video region is 432 raw pixels wide × 160 raw rows tall.
 * Each logical pixel maps to 3 sub-pixels in the horizontal direction,
 * creating four perceived brightness levels.
 */
void ui_put_pixel(uint8_t *fb, int x, int y, int c);
void ui_put_rect(uint8_t *fb, int x0, int y0, int x1, int y1, int c);

/* ── ROM / file picker ───────────────────────────────────────────────────── *
 *
 * Displays a full-screen file browser in the video framebuffer.
 * Scans mount_pt and every first-level subdirectory for regular files.
 * The user navigates with the GB D-pad (UP / DOWN) and confirms with
 * A or START.
 *
 * Returns true  and writes the selected path into out_path on success.
 * Returns false if no files were found on the SD card.
 *
 * Call msg_flip() after this returns to obtain a fresh back-buffer before
 * entering the main game render loop.
 */
typedef enum {
	UI_ROM_PICK_NONE = 0,
	UI_ROM_PICK_SELECTED,
	UI_ROM_PICK_LOAD_LAST,
} ui_rom_pick_result_t;

ui_rom_pick_result_t ui_rom_picker(const char *mount_pt, char *out_path, size_t path_size);

/* Show a transient full-screen menu-style notice for the requested duration. */
void ui_show_notice(const char *title, const char *message, uint32_t duration_ms);

/* Clear EPD ghosting by flashing 5 full-black then 5 full-white frames. */
void ui_clear_ghosting(void);

/* Draw a one-row "!! BATTERY LOW !!" warning bar over the top of the
 * video framebuffer.  Call after paperboy_gb_run_frame() on rendered frames
 * when battery_is_low() returns true. */
void ui_draw_bat_low_overlay(uint8_t *fb);
