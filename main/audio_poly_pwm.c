/**
 * audio_poly_pwm.c — TDM polyphonic PWM audio for M5PaperS3 buzzer
 *
 * Architecture:
 *   emulator task
 *       → writes APU registers via audio_apu_read/write()
 *         (these update s_apu, which lives here)
 *
 *   display task — audio_service_frame(), called once per vsync (~60 Hz)
 *       → advances the TDM channel index: 0 → 1 → 2 → 3 → 0 → …
 *       → if the scheduled channel is silent, searches the remaining three
 *         channels in order so the slot is never wasted while notes are playing
 *       → reads s_apu.chans[idx] state (enabled, powered, freq)
 *       → sets LEDC timer frequency to the channel's note frequency
 *       → 50% duty = full-amplitude square wave → buzzer vibrates loudly
 *       → 0% duty = silence when the channel is inactive
 *
 * Design trade-offs (intentional deviations from GB accuracy):
 *   • Volume / envelope / sweep control is ignored — always full amplitude.
 *   • Length counters are not advanced (minigb_apu callback is not called).
 *     Channels are silenced only by explicit DAC-power / NR52 register writes.
 *   • Each channel occupies exactly one vsync slot (~16.7 ms at 60 Hz).
 *     Effective per-channel repetition rate ≈ 15 Hz — audibly "buzzy" but
 *     surprisingly musical for the short, harmonically-rich waveforms typical
 *     of Game Boy chiptune music.
 *   • CH4 (noise) is approximated as a periodic tone derived from its LFSR
 *     clock rate; this produces a pitched buzz rather than actual noise.
 *
 * Frequency derivation (from minigb_apu internal representation):
 *   CH1/CH2  (pulse)  : f = 131072 / (2048 − period)  Hz
 *   CH3      (wave)   : f = 65536  / (2048 − period) × cycle_mult  Hz
 *     cycle_mult: 1 normally; 2/4/8/16 when wave RAM contains N copies of a
 *     shorter pattern (detected by checking for the shortest repeating period
 *     among the 32 nibbles: p=16→2×, p=8→4×, p=4→8×, p=2→16×).
 *   CH4      (noise)  : LFSR_clock / 128  (rough approximation)
 *     where LFSR_clock = 4194304 / (lfsr_div_lut[r] << s)
 *     and lfsr_div_lut = {8,16,32,48,64,80,96,112}, s = chan.freq, r = chan.noise.lfsr_div
 *
 * LEDC configuration:
 *   Timer 0, low-speed mode, 12-bit resolution (~20 Hz – 19.5 kHz at 80 MHz APB).
 *   GPIO21 (PAPERBOY_BUZZER_GPIO), 50% duty for active notes.
 */

#include "audio.h"

#include <stddef.h>

#include "freertos/FreeRTOS.h"

#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#include "paperboy_config.h"
#include "minigb_apu/minigb_apu.h"
#include "profiler.h"

static const char *TAG = "audio_poly";

/* -----------------------------------------------------------------------
 * APU state (owned here; emulator writes registers via audio_apu_read/write)
 * --------------------------------------------------------------------- */

static struct minigb_apu_ctx DRAM_ATTR s_apu;

/* -----------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------- */

#define BUZZER_GPIO     PAPERBOY_BUZZER_GPIO

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0

/*
 * 12-bit duty resolution (4096 steps).
 *
 * At 80 MHz APB the achievable frequency range is approximately:
 *   min ≈ 80 MHz / (1023 × 4096) ≈ 19 Hz
 *   max ≈ 80 MHz /          4096 ≈ 19 531 Hz
 *
 * This covers the full Game Boy frequency range:
 *   CH1/CH2 lowest note: 131072 / 2048 ≈ 64 Hz
 *   CH1/CH2 highest:     131072 /    1 = 131 072 Hz  (clamped below)
 *   CH3 highest:          65536 /    1 =  65 536 Hz  (clamped below)
 */
#define LEDC_DUTY_RES    LEDC_TIMER_12_BIT
#define LEDC_DUTY_BITS   12u
#define LEDC_DUTY_MAX    (1u << LEDC_DUTY_BITS)   /* 4096 */
#define LEDC_DUTY_HALF   (LEDC_DUTY_MAX >> 1u)    /* 2048 = 50% */

/* Frequency clamp — notes outside this range are treated as silence. */
#define POLY_FREQ_MIN_HZ   65u
#define POLY_FREQ_MAX_HZ   16000u

/*
 * LFSR clock divider look-up table for CH4 (noise), matching the LUT in
 * minigb_apu.c:  lfsr_div_lut[] = { 8, 16, 32, 48, 64, 80, 96, 112 }
 */
static const uint16_t s_lfsr_div_lut[8] = { 8, 16, 32, 48, 64, 80, 96, 112 };

/* -----------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------- */

static bool    s_initialised = false;
static uint8_t s_poly_chan   = 3u;   /* advances to 0 on the first tick */

/*
 * Scratch buffer for the APU callback (PCM output discarded; we call the
 * callback solely to advance envelope, length counter, and sweep state so
 * that ch->volume reaches 0 when a game mutes a channel via the envelope).
 */
#define APU_BUF_TOTAL (AUDIO_SAMPLES * 2)   /* = AUDIO_SAMPLES_TOTAL */
static int16_t DRAM_ATTR s_apu_buf[APU_BUF_TOTAL];

/* -----------------------------------------------------------------------
 * Frequency derivation
 * --------------------------------------------------------------------- */

/*
 * Detect whether CH3's 32-nibble wave RAM contains a shorter repeating
 * pattern and return the corresponding cycle multiplier.
 *
 * Wave RAM layout (minigb_apu): audio_mem[0xFF30–0xFF3F], 16 bytes.
 *   byte[k] = { nibble[2k] in bits[7:4],  nibble[2k+1] in bits[3:0] }
 *
 * Candidate periods (in nibbles): 2, 4, 8, 16.  The inner loop verifies
 * nibble[n] == nibble[n+p] for every n in [0, 32−p).  This is the standard
 * "shift-and-compare" period test; transitivity covers all repetitions.
 *
 * We test from shortest period first so we always find the true fundamental
 * (e.g. a 4-nibble sine repeated 8× is reported as 8×, not 4× or 2×).
 *
 * Returns: 32/p for the first p that passes, or 1 if no short period found.
 */
static uint8_t IRAM_ATTR wave_cycle_mult(void)
{
    const uint8_t *w = &s_apu.audio_mem[0xFF30u - AUDIO_ADDR_COMPENSATION];

    for (uint8_t p = 2u; p <= 16u; p = (uint8_t)(p << 1u)) {
        bool ok = true;
        for (uint8_t n = 0u; n < (uint8_t)(32u - p) && ok; n++) {
            uint8_t va = (n       & 1u) ? (w[n       >> 1u] & 0x0Fu)
                                        : (w[n       >> 1u] >> 4u);
            uint8_t vb = ((n + p) & 1u) ? (w[(n + p) >> 1u] & 0x0Fu)
                                        : (w[(n + p) >> 1u] >> 4u);
            if (va != vb) ok = false;
        }
        if (ok) return (uint8_t)(32u / p);
    }
    return 1u;
}

/*
 * Return the audible frequency in Hz for APU channel idx (0–3).
 * Returns 0 if the channel is inactive or the derived frequency falls
 * outside [POLY_FREQ_MIN_HZ, POLY_FREQ_MAX_HZ].
 */
static uint32_t poly_chan_freq_hz(uint8_t idx)
{
    const struct chan *ch = &s_apu.chans[idx];

    if (!ch->enabled || !ch->powered || ch->volume == 0) {
        return 0u;
    }

    uint32_t hz = 0u;

    if (idx < 2u) {
        /*
         * CH1 / CH2 — pulse channels.
         * f = 131072 / (2048 − period) Hz
         * ch->freq holds the 11-bit GB period register value.
         */
        uint16_t period = ch->freq;
        if (period < 2048u) {
            uint32_t denom = 2048u - period;
            hz = (denom > 0u) ? (131072u / denom) : 0u;
        }
    } else if (idx == 2u) {
        /*
         * CH3 — wave channel.
         * f = 65536 / (2048 − period) × cycle_mult  Hz
         *
         * cycle_mult corrects for wave RAM patterns that contain N identical
         * sub-cycles: the hardware plays the full 32 samples at the computed
         * frequency, so if the waveform has period 32/N the actual pitch is
         * N× higher than the register value implies.
         */
        uint16_t period = ch->freq;
        if (period < 2048u) {
            uint32_t denom = 2048u - period;
            hz = (denom > 0u) ? (65536u / denom) : 0u;
            hz *= wave_cycle_mult();
        }
    } else {
        /*
         * CH4 — noise channel.
         *
         * In minigb_apu:
         *   ch->freq          = frequency shift s  (from NR43[7:4], 0–13)
         *   ch->noise.lfsr_div = clock divider index r (from NR43[2:0], 0–7)
         *
         * LFSR clock = 4 194 304 / (lfsr_div_lut[r] << s)  Hz
         *
         * Dividing by 128 maps the LFSR clock into a rough audible pitch:
         *   r=0, s=0 → 524 288 / 128 = 4096 Hz  (high-pitched buzz)
         *   r=0, s=4 →  32 768 / 128 =  256 Hz  (medium buzz)
         *   r=4, s=0 →  65 536 / 128 =  512 Hz
         *
         * Values outside the audible clamp are silenced below.
         */
        uint8_t s_shift = ch->freq;           /* 0–13 when channel is live */
        uint8_t r_idx   = ch->noise.lfsr_div; /* 0–7                       */
        if (r_idx < 8u && s_shift < 20u) {    /* guard against stale state */
            uint32_t lfsr_hz =
                4194304u / ((uint32_t)s_lfsr_div_lut[r_idx] << s_shift);
            hz = lfsr_hz >> 7u;              /* / 128                      */
        }
    }

    /* Clamp to the range LEDC can reliably achieve and the buzzer can reproduce. */
    if (hz < POLY_FREQ_MIN_HZ || hz > POLY_FREQ_MAX_HZ) {
        hz = 0u;
    }

    return hz;
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void audio_poly_init(void)
{
    minigb_apu_audio_init(&s_apu);

    if (s_initialised) {
        return;
    }

    /* --- LEDC timer --------------------------------------------------- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = 1000u,    /* placeholder; overwritten each frame */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* --- LEDC channel ------------------------------------------------- */
    ledc_channel_config_t ch_cfg = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_GPIO,
        .duty           = 0u,   /* start silent */
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s_poly_chan   = 3u;     /* wraps to 0 on the first audio_service_frame() */
    s_initialised = true;

    ESP_LOGI(TAG, "Polyphonic PWM audio: GPIO%d, 12-bit, %u–%u Hz TDM range",
             BUZZER_GPIO, POLY_FREQ_MIN_HZ, POLY_FREQ_MAX_HZ);
}

void audio_poly_service_frame(void)
{
    if (!s_initialised) {
        return;
    }

    int64_t start_us = esp_timer_get_time();

    /*
     * Advance APU state machine (envelopes, length counters, sweep).
     * The PCM output in s_apu_buf is discarded — we only call the callback
     * so that ch->volume is correctly decremented to 0 when a game mutes a
     * channel via the volume envelope rather than an explicit register write.
     */
    minigb_apu_audio_callback(&s_apu, s_apu_buf);

    /* Advance the round-robin pointer, then search up to 4 channels for an
     * active note.  If the scheduled channel is silent, try the next ones in
     * order so the slot is never wasted when there is something to play. */
    s_poly_chan = (s_poly_chan + 1u) & 3u;

    uint32_t freq_hz = 0u;
    for (uint8_t i = 0u; i < 4u; i++) {
        freq_hz = poly_chan_freq_hz((s_poly_chan + i) & 3u);
        if (freq_hz > 0u) {
            break;
        }
    }

    if (freq_hz > 0u) {
        /* Active note: tune the LEDC timer and output a 50% duty square wave.
         * ledc_set_freq() computes the optimal APB divider automatically. */
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_HALF);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        /* All channels silent: hold GPIO low. */
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0u);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    profiler_audio_add(esp_timer_get_time() - start_us);

    const uint32_t AUDIO_BUDGET_US = PAPERBOY_AUDIO_BUDGET_MS * 1000;
    if (AUDIO_BUDGET_US > 0) {
        int64_t elapsed_us = esp_timer_get_time() - start_us;
        if (elapsed_us < AUDIO_BUDGET_US) {
            ets_delay_us((uint32_t)(AUDIO_BUDGET_US - elapsed_us));
        }
    }
}

void audio_poly_push_samples(const int16_t *samples, size_t n_pairs)
{
    /* PCM synthesis is bypassed in polyphony mode — no ring buffer needed. */
    (void)samples;
    (void)n_pairs;
}

size_t audio_poly_ring_free(void)
{
    /* Always report a full ring so callers never block waiting for space. */
    return (size_t)UINT32_MAX;
}

uint8_t audio_poly_apu_read(uint16_t addr)
{
    return minigb_apu_audio_read(&s_apu, addr);
}

void audio_poly_apu_write(uint16_t addr, uint8_t val)
{
    minigb_apu_audio_write(&s_apu, addr, val);
}

void audio_poly_deinit(void)
{
    if (!s_initialised) {
        return;
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0u);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0u);

    s_initialised = false;
}
