#include "serial_input.h"

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "gbemu.h"
#include "sdkconfig.h"

static const char *TAG = "serial_input";

#define SERIAL_HOLD_MS          220
#define SERIAL_ACTION_HOLD_MS   250

static const uint8_t s_button_masks[] = {
    GB_BTN_UP,
    GB_BTN_DOWN,
    GB_BTN_LEFT,
    GB_BTN_RIGHT,
    GB_BTN_A,
    GB_BTN_B,
    GB_BTN_START,
    GB_BTN_SELECT,
};

static bool s_usb_ready;
static int s_escape_state;
static int64_t s_button_until_ms[sizeof(s_button_masks)];
static int64_t s_action_until_ms[3];

static void hold_button(uint8_t mask, int64_t now_ms)
{
    for (size_t i = 0; i < sizeof(s_button_masks); i++) {
        if (s_button_masks[i] == mask) {
            s_button_until_ms[i] = now_ms + SERIAL_HOLD_MS;
            return;
        }
    }
}

static void hold_action(uint8_t mask, int64_t now_ms)
{
    if (mask & TP_ACTION_LOAD) {
        s_action_until_ms[0] = now_ms + SERIAL_ACTION_HOLD_MS;
    }
    if (mask & TP_ACTION_SAVE) {
        s_action_until_ms[1] = now_ms + SERIAL_ACTION_HOLD_MS;
    }
    if (mask & TP_ACTION_CLEAR_SCREEN) {
        s_action_until_ms[2] = now_ms + SERIAL_ACTION_HOLD_MS;
    }
}

static void process_key(uint8_t ch, int64_t now_ms)
{
    if (s_escape_state == 1) {
        s_escape_state = (ch == '[' || ch == 'O') ? 2 : 0;
        return;
    }

    if (s_escape_state == 2) {
        s_escape_state = 0;
        switch (ch) {
        case 'A': hold_button(GB_BTN_UP, now_ms); return;
        case 'B': hold_button(GB_BTN_DOWN, now_ms); return;
        case 'C': hold_button(GB_BTN_RIGHT, now_ms); return;
        case 'D': hold_button(GB_BTN_LEFT, now_ms); return;
        default: return;
        }
    }

    if (ch == 0x1B) {
        s_escape_state = 1;
        return;
    }

    switch ((uint8_t)tolower(ch)) {
    case 'w': hold_button(GB_BTN_UP, now_ms); break;
    case 's': hold_button(GB_BTN_DOWN, now_ms); break;
    case 'a': hold_button(GB_BTN_LEFT, now_ms); break;
    case 'd': hold_button(GB_BTN_RIGHT, now_ms); break;
    case 'j':
    case 'x': hold_button(GB_BTN_A, now_ms); break;
    case 'k':
    case 'z': hold_button(GB_BTN_B, now_ms); break;
    case '\r':
    case '\n': hold_button(GB_BTN_START, now_ms); break;
    case ' ':
    case '\t': hold_button(GB_BTN_SELECT, now_ms); break;
    case 'r': hold_action(TP_ACTION_LOAD, now_ms); break;
    case 'p': hold_action(TP_ACTION_SAVE, now_ms); break;
    case 'c': hold_action(TP_ACTION_CLEAR_SCREEN, now_ms); break;
    default: break;
    }
}

static void drain_usb_serial_jtag(int64_t now_ms)
{
    if (!s_usb_ready || !usb_serial_jtag_is_driver_installed()) {
        return;
    }

    uint8_t buf[32];
    int len;
    do {
        len = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
        for (int i = 0; i < len; i++) {
            process_key(buf[i], now_ms);
        }
    } while (len == (int)sizeof(buf));
}

void serial_input_init(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        esp_err_t err = usb_serial_jtag_driver_install(&cfg);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "USB-Serial-JTAG input unavailable: %s", esp_err_to_name(err));
        }
    }
    s_usb_ready = usb_serial_jtag_is_driver_installed();
#endif

    ESP_LOGI(TAG, "Controls over COM: WASD/arrows, J/X=A, K/Z=B, Enter=Start, Space=Select, P=save, R=load, C=clear");
}

tp_state_t serial_input_read_state(void)
{
    const int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000);
    tp_state_t state = {0};

    drain_usb_serial_jtag(now_ms);

    for (size_t i = 0; i < sizeof(s_button_masks); i++) {
        if (s_button_until_ms[i] > now_ms) {
            state.gb_buttons |= s_button_masks[i];
        }
    }

    if (s_action_until_ms[0] > now_ms) {
        state.actions |= TP_ACTION_LOAD;
    }
    if (s_action_until_ms[1] > now_ms) {
        state.actions |= TP_ACTION_SAVE;
    }
    if (s_action_until_ms[2] > now_ms) {
        state.actions |= TP_ACTION_CLEAR_SCREEN;
    }

    return state;
}
