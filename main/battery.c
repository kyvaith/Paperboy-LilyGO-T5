#include "battery.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "battery";

/*
 * Hardware wiring (M5PaperS3):
 *   Battery+ ── R1 ── GPIO3 ── R2 ── GND   (R1 = R2, so ½× divider)
 *   GPIO3 = ADC1_CHANNEL_2 on ESP32-S3
 *
 * ADC attenuation:
 *   ADC_ATTEN_DB_12 gives a full-scale input of ~2.5 V.
 *   With the ½× divider the measurable battery range is 0–5 V, which
 *   comfortably covers the LiPo operating window of 3.0–4.2 V.
 */
#define BAT_ADC_UNIT     ADC_UNIT_1
#define BAT_ADC_CHANNEL  ADC_CHANNEL_2   /* GPIO3 on ESP32-S3 */
#define BAT_ADC_ATTEN    ADC_ATTEN_DB_12 /* 0–≈2.5 V input range  */
#define BAT_SAMPLES      8               /* readings to average    */
#define BAT_DIVIDER      2               /* resistor divider ratio */

static adc_oneshot_unit_handle_t s_adc_handle  = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_cali_valid  = false;
static bool                      s_initialised = false;

void battery_init(void)
{
    /* Initialise the ADC unit. */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BAT_ADC_UNIT,
    };
    if (adc_oneshot_new_unit(&unit_cfg, &s_adc_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise ADC unit");
        return;
    }

    /* Configure the battery channel. */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc_handle, BAT_ADC_CHANNEL, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel");
        return;
    }

    /* Try to create a line-fitting calibration handle (ESP32-S3 scheme). */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = BAT_ADC_UNIT,
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    s_cali_valid = (cali_err == ESP_OK);
    if (!s_cali_valid) {
        ESP_LOGW(TAG, "ADC calibration unavailable (%s) – using raw approximation",
                 esp_err_to_name(cali_err));
    } else {
        ESP_LOGI(TAG, "ADC calibration ready");
    }

    s_initialised = true;
}

uint32_t battery_read_mv(void)
{
    if (!s_initialised) {
        return 0;
    }

    /* Average BAT_SAMPLES raw readings to reduce noise. */
    int sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc_handle, BAT_ADC_CHANNEL, &raw);
        sum += raw;
    }
    int avg_raw = sum / BAT_SAMPLES;

    /* Convert raw count to millivolts at the ADC pin. */
    int mv_pin = 0;
    if (s_cali_valid) {
        adc_cali_raw_to_voltage(s_cali_handle, avg_raw, &mv_pin);
    } else {
        /* Fallback: 12-bit ADC, ≈2500 mV full-scale for 12 dB attenuation. */
        mv_pin = (int)((int64_t)avg_raw * 2500 / 4095);
    }

    /* Undo the ½× voltage divider to recover the actual battery voltage. */
    return (uint32_t)(mv_pin * BAT_DIVIDER);
}

/* ── Low-battery detector ────────────────────────────────────────────────── */

#define BAT_CHECK_INTERVAL_US  1000000LL   /* re-sample ADC at most once / s */

static bool    s_bat_low     = false;
static int64_t s_bat_next_us = 0;

bool battery_is_low(void)
{
    int64_t now = esp_timer_get_time();
    if (now < s_bat_next_us) {
        return s_bat_low;   /* return cached result between samples */
    }
    s_bat_next_us = now + BAT_CHECK_INTERVAL_US;

    uint32_t mv = battery_read_mv();
    if (mv == 0) {
        return false;   /* not initialised */
    }

    /* Hysteresis: enter low state below LOW threshold, clear above RECOVER. */
    if (s_bat_low) {
        if (mv >= BAT_RECOVER_THRESHOLD_MV) {
            s_bat_low = false;
        }
    } else {
        if (mv < BAT_LOW_THRESHOLD_MV) {
            s_bat_low = true;
        }
    }
    return s_bat_low;
}
