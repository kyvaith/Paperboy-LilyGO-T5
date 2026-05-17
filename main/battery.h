#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * Battery voltage monitoring via ADC.
 *
 * The M5PaperS3 feeds battery voltage through a ½× resistor divider into
 * GPIO3 (ADC1_CHANNEL_2).  battery_read_mv() compensates for the divider
 * and returns the actual battery voltage in millivolts.
 *
 * Call battery_init() once during startup before using battery_read_mv().
 */

/* Initialise the ADC unit and calibration scheme.  Non-fatal if the
 * factory calibration eFuses are not burned – a raw approximation is
 * used as a fallback. */
void battery_init(void);

/* Return the current battery voltage in millivolts.
 * Averages several raw ADC samples and applies the ½× divider correction.
 * Returns 0 if battery_init() has not been called. */
uint32_t battery_read_mv(void);

/*
 * Low-battery detector with hysteresis.
 *
 * Thresholds (mV):
 *   BAT_LOW_THRESHOLD_MV     — voltage below which the warning activates
 *   BAT_RECOVER_THRESHOLD_MV — voltage above which the warning clears
 *
 * The ADC is sampled at most once per second; the cached result is returned
 * on all other calls, so this is safe to call on every rendered frame.
 */
#define BAT_LOW_THRESHOLD_MV      3500u
#define BAT_RECOVER_THRESHOLD_MV  3600u

bool battery_is_low(void);
