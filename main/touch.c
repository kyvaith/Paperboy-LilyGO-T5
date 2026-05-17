#include "touch.h"
#include "gbemu.h"

#include <string.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "touch";

/* ── Hardware constants (M5PaperS3) ─────────────────────────────────────── */

#define GT911_I2C_PORT   I2C_NUM_1
#define GT911_PIN_SDA    41
#define GT911_PIN_SCL    42
#define GT911_PIN_INT    48   /* active-low when a touch sample is ready */
#define GT911_FREQ_HZ    400000

/* I2C transaction timeout.  Long enough for the GT911 to respond, short
 * enough that repeated failures don't keep the task WDT from being fed. */
#define GT911_XFER_TIMEOUT_MS  100

/* After a bus error, skip touch polling for this long.  During the
 * cooldown the main loop runs normally (msg_flip() yields) so the
 * FreeRTOS IDLE task can run and the task WDT stays fed. */
#define TOUCH_ERROR_COOLDOWN_US  (500 * 1000)   /* 500 ms */

/* GT911 can appear at either address; which one depends on the state of the
 * INT pin during the chip's power-on reset sequence.  Probe both. */
static const uint8_t s_gt911_addrs[] = { 0x5D, 0x14 };

/* ── GT911 register map ───────────────────────────────────────────────────── */

/* 0x814E – Status register: bit[7]=buffer_ready, bits[3:0]=touch-point count.
 * 0x814F – Start of touch-point records (8 bytes each, up to 5 points):
 *   Byte 0: Track ID
 *   Byte 1: X coordinate low  [7:0]
 *   Byte 2: X coordinate high [3:0]
 *   Byte 3: Y coordinate low  [7:0]
 *   Byte 4: Y coordinate high [3:0]
 *   Byte 5: Touch area low    [7:0]
 *   Byte 6: Touch area high   [3:0]
 *   Byte 7: Reserved
 *
 * IMPORTANT: point records start at 0x814F, NOT 0x8150.  Reading from 0x8150
 * skips the Track-ID byte, shifting every subsequent byte by one and causing
 * the coordinate fields to alias into the area-size fields, which vary with
 * finger pressure and produce "random-looking" coordinates. */
#define GT911_REG_STATUS  0x814E
#define GT911_REG_POINT1  0x814F  /* first touch-point record (8 bytes × n) */

/* ── Button zones ────────────────────────────────────────────────────────── *
 *
 * Coordinates are in GT911 native space: x ∈ [0..539], y ∈ [0..959].
 * The GT911 on M5PaperS3 uses offset_rotation=1, meaning the hardware maps
 * its x-axis along the EPD long axis (960 px) and y along the short axis
 * (540 px).  Read the INFO-level log ("touch") while pressing each button
 * zone on the background image to discover the actual coordinate ranges,
 * then fill in the stubs below.
 *
 * Zone layout mirrors the on-screen GameBoy graphic:
 *   - D-pad on the left half
 *   - A / B on the right half
 *   - START / SELECT across the bottom centre
 *
 * Stub: {0, 0, 0, 0} → rectangle disabled (never matches). */

typedef struct {
    uint16_t x0, y0;   /* inclusive top-left  */
    uint16_t x1, y1;   /* inclusive bottom-right */
} tp_rect_t;

/* Each button zone holds up to 2 rects so that non-rectangular areas (e.g.
 * an L-shaped D-pad segment) can be covered without overlap.
 * Set r[1] = {0,0,0,0} when only one rect is needed.
 * Any all-zero rect is treated as "not configured" and is skipped. */
typedef struct {
    tp_rect_t r[2];
} tp_btn_zone_t;

/* Order must match s_btn_masks[] below.
 * D-pad buttons have space for a second rect; A/B/START/SELECT only need one. */
static const tp_btn_zone_t s_btn_zones[] = {
    /* GB_BTN_UP     */ { .r = { {404, 286, 479, 338}, {424, 263, 461, 286} } },
    /* GB_BTN_DOWN   */ { .r = { {404, 151, 479, 202}, {424, 203, 461, 227} } },
    /* GB_BTN_LEFT   */ { .r = { {479, 203, 532, 286}, {461, 227, 479, 263} } },
    /* GB_BTN_RIGHT  */ { .r = { {354, 203, 403, 286}, {403, 227, 424, 263} } },
    /* GB_BTN_A      */ { .r = { { 19, 227,  96, 303}, {0, 0, 0, 0} } },
    /* GB_BTN_B      */ { .r = { {125, 188, 202, 267}, {0, 0, 0, 0} } },
    /* GB_BTN_START  */ { .r = { {186,  71, 260,  99}, {0, 0, 0, 0} } },
    /* GB_BTN_SELECT */ { .r = { {277,  71, 348,  99}, {0, 0, 0, 0} } },
};

static const uint8_t s_btn_masks[] = {
    GB_BTN_UP, GB_BTN_DOWN, GB_BTN_LEFT, GB_BTN_RIGHT,
    GB_BTN_A,  GB_BTN_B,   GB_BTN_START, GB_BTN_SELECT,
};

#define NUM_BUTTONS  (sizeof(s_btn_zones) / sizeof(s_btn_zones[0]))

/* Save/load zones are intentionally left disabled until they are calibrated
 * against the baked background image. */
static const tp_btn_zone_t s_action_zones[] = {
    /* TP_ACTION_LOAD */ { .r = { {14, 921, 112, 958}, {0, 0, 0, 0} } },
    /* TP_ACTION_SAVE */ { .r = { {134, 921, 233, 958}, {0, 0, 0, 0} } },
};

static const uint8_t s_action_masks[] = {
    TP_ACTION_LOAD,
    TP_ACTION_SAVE,
};

#define NUM_ACTIONS  (sizeof(s_action_zones) / sizeof(s_action_zones[0]))

/* ── Driver state ────────────────────────────────────────────────────────── */

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_dev;
static bool    s_inited       = false;
static int64_t s_skip_until_us = 0;   /* µs: skip I2C until this time */

/* ── Low-level I2C helpers ───────────────────────────────────────────────── */

/* Write a single byte to a 16-bit GT911 register. */
static esp_err_t gt911_write_reg(uint16_t reg, uint8_t val)
{
    const uint8_t buf[3] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        val,
    };
    return i2c_master_transmit(s_i2c_dev, buf, sizeof(buf), GT911_XFER_TIMEOUT_MS);
}

/* Read one or more bytes from a 16-bit GT911 register. */
static esp_err_t gt911_read_regs(uint16_t reg, uint8_t *out, size_t len)
{
    const uint8_t reg_buf[2] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
    };
    return i2c_master_transmit_receive(s_i2c_dev, reg_buf, 2, out, len, GT911_XFER_TIMEOUT_MS);
}

/* Recover from a stuck bus and start the cooldown timer.
 *
 * i2c_master_bus_reset() bit-bangs 9 SCL clocks over the I2C pins to
 * release any SDA held low by a confused slave, then sends START+STOP.
 * This bypasses the I2C peripheral so it works even when the hardware
 * controller is in a bad state. */
static void gt911_bus_error_recovery(void)
{
    ESP_LOGW(TAG, "I2C error — resetting bus, pausing touch for 500 ms");
    i2c_master_bus_reset(s_i2c_bus);
    s_skip_until_us = esp_timer_get_time() + TOUCH_ERROR_COOLDOWN_US;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t tp_init(void)
{
    esp_err_t err;

    /* Configure INT as input with internal pull-up.
     * GT911 INT is open-drain: it can only pull LOW; it relies on a pull-up
     * to return HIGH when idle.  Without a pull-up the pin floats, reads as
     * LOW unpredictably, and causes spurious reads that confuse the GT911
     * and eventually stall the I2C bus. */
    gpio_config_t io_conf = {
        .pin_bit_mask   = (1ULL << GT911_PIN_INT),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Create I2C master bus on I2C_NUM_1 (external pull-ups fitted on PCB). */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port              = GT911_I2C_PORT,
        .sda_io_num            = GT911_PIN_SDA,
        .scl_io_num            = GT911_PIN_SCL,
        .clk_source            = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt     = 7,
        .flags.enable_internal_pullup = false,
    };
    err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    /* Probe both possible GT911 I2C addresses. */
    uint8_t found_addr = 0;
    for (size_t i = 0; i < sizeof(s_gt911_addrs); i++) {
        if (i2c_master_probe(s_i2c_bus, s_gt911_addrs[i], 50) == ESP_OK) {
            found_addr = s_gt911_addrs[i];
            break;
        }
    }
    if (found_addr == 0) {
        ESP_LOGW(TAG, "GT911 not found (SDA=GPIO%d SCL=GPIO%d) — touch disabled",
                 GT911_PIN_SDA, GT911_PIN_SCL);
        i2c_del_master_bus(s_i2c_bus);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "GT911 found at I2C 0x%02X", found_addr);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = found_addr,
        .scl_speed_hz    = GT911_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_i2c_bus);
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "Touch driver ready (INT=GPIO%d)", GT911_PIN_INT);
    return ESP_OK;
}

tp_state_t tp_read_state(void)
{
    tp_state_t state = {0};

    if (!s_inited) {
        return state;
    }

    /* Error cooldown: skip I2C entirely so the main loop can yield to the
     * IDLE task and keep the task WDT fed after a bus fault. */
    if (esp_timer_get_time() < s_skip_until_us) {
        return state;
    }

    /* INT is driven LOW by GT911 only when new touch data is pending.
     * With the pull-up in place, HIGH reliably means "no touch". */
    if (gpio_get_level(GT911_PIN_INT) != 0) {
        return state;
    }

    /* ── Read status register ─────────────────────────────────────────── */
    uint8_t status = 0;
    if (gt911_read_regs(GT911_REG_STATUS, &status, 1) != ESP_OK) {
        gt911_bus_error_recovery();
        return state;
    }

    /* bit[7]: buffer_status — 1 means new touch data is ready */
    if (!(status & 0x80)) {
        gt911_write_reg(GT911_REG_STATUS, 0);   /* clear stale flag */
        return state;
    }

    uint8_t n = status & 0x0F;
    if (n > 5) n = 5;   /* GT911 supports max 5 simultaneous touch points */

    /* ── Read touch point data ────────────────────────────────────────── */
    if (n > 0) {
        uint8_t pts[5 * 8] = {0};
        if (gt911_read_regs(GT911_REG_POINT1, pts, (size_t)n * 8) != ESP_OK) {
            /* Attempt status clear before recovering so GT911 releases INT */
            gt911_write_reg(GT911_REG_STATUS, 0);
            gt911_bus_error_recovery();
            return state;
        }

        for (uint8_t p = 0; p < n; p++) {
            const uint8_t *pt = &pts[p * 8];
            /* Each point: [track_id][x_lo][x_hi_nibble][y_lo][y_hi_nibble]... */
            uint16_t raw_x = (uint16_t)pt[1] | ((uint16_t)(pt[2] & 0x0F) << 8);
            uint16_t raw_y = (uint16_t)pt[3] | ((uint16_t)(pt[4] & 0x0F) << 8);

            /* Log raw coordinates — use these to fill in the stub rects above. */
            //ESP_LOGI(TAG, "touch[%u] raw_x=%-4u raw_y=%-4u", (unsigned)p, raw_x, raw_y);

            for (size_t b = 0; b < NUM_BUTTONS; b++) {
                for (size_t ri = 0; ri < 2; ri++) {
                    const tp_rect_t *r = &s_btn_zones[b].r[ri];
                    /* All-zero rect = not yet calibrated, skip. */
                    if (r->x0 == 0 && r->x1 == 0 && r->y0 == 0 && r->y1 == 0) {
                        continue;
                    }
                    if (raw_x >= r->x0 && raw_x <= r->x1 &&
                        raw_y >= r->y0 && raw_y <= r->y1) {
                        state.gb_buttons |= s_btn_masks[b];
                        break;  /* matched; no need to check the second rect */
                    }
                }
            }

            for (size_t a = 0; a < NUM_ACTIONS; a++) {
                for (size_t ri = 0; ri < 2; ri++) {
                    const tp_rect_t *r = &s_action_zones[a].r[ri];
                    if (r->x0 == 0 && r->x1 == 0 && r->y0 == 0 && r->y1 == 0) {
                        continue;
                    }
                    if (raw_x >= r->x0 && raw_x <= r->x1 &&
                        raw_y >= r->y0 && raw_y <= r->y1) {
                        state.actions |= s_action_masks[a];
                        break;  /* matched; no need to check the second rect */
                    }
                }
            }
        }
    }

    /* ── Clear buffer_status so GT911 deasserts INT ───────────────────── */
    if (gt911_write_reg(GT911_REG_STATUS, 0) != ESP_OK) {
        gt911_bus_error_recovery();
    }

    return state;
}

uint8_t tp_read_buttons(void)
{
    return tp_read_state().gb_buttons;
}
