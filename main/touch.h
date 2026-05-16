#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * Initialise the GT911 capacitive touchscreen over I2C.
 *
 * M5PaperS3 wiring:
 *   SDA  = GPIO 41
 *   SCL  = GPIO 42
 *   INT  = GPIO 48  (active-low when touch data is ready)
 *   I2C  = I2C_NUM_1 @ 400 kHz
 *   Addresses probed: 0x5D, 0x14 (GT911 supports both)
 *
 * Safe to call even if the hardware is absent; tp_read_buttons() will then
 * always return 0.
 */
esp_err_t tp_init(void);

/**
 * Sample the touchscreen and return a GameBoy button bitmask.
 *
 * Button zones are defined by the tp_rect_t table in touch.c.
 * All zones start as stubs ({0,0,0,0}).  Fill them in once you have
 * read the raw GT911 coordinates from the INFO log (tag "touch").
 *
 * Returns: bitmask of GB_BTN_* values that are currently touched.
 *          0 if the driver is not initialised or no touch is active.
 */
uint8_t tp_read_buttons(void);
