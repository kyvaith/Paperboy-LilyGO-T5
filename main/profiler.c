#include "profiler.h"

#include <string.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "paperboy_config.h"

static const char *TAG = "prof";

/* Timestamp (µs) taken at profiler_begin(). */
static int64_t s_t0[PROF_PHASES];

/* Per-frame µs accumulators (reset each frame). */
static int64_t s_frame_us[PROF_PHASES];

/* Per-second µs accumulators and counters. */
static int64_t  s_sec_us[PROF_PHASES];
static uint32_t s_sec_frames;
static uint32_t s_sec_skipped;
static int64_t  s_sec_start_us; /* esp_timer_get_time() at start of interval */

/*
 * Audio task accumulator — written by the audio task, read+reset by the
 * emulator task in profiler_frame_end().  Uses an atomic uint32_t so that
 * the single-word load/store on Xtensa LX7 is naturally race-free.
 * Max value per second: ~60 calls × ~16 ms = ~960 000 µs < UINT32_MAX.
 */
static _Atomic uint32_t s_audio_sec_us = 0;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void profiler_begin(int phase)
{
    s_t0[phase] = esp_timer_get_time();
}

void profiler_end(int phase)
{
    s_frame_us[phase] += esp_timer_get_time() - s_t0[phase];
}

void profiler_audio_add(int64_t elapsed_us)
{
    /* Saturate to uint32_t max to avoid wrap confusion. */
    uint32_t add = (elapsed_us > 0 && elapsed_us < 0xFFFFFFFF)
                   ? (uint32_t)elapsed_us : 0u;
    atomic_fetch_add_explicit(&s_audio_sec_us, add, memory_order_relaxed);
}

void profiler_frame_end(bool skipped)
{
    /* Accumulate this frame's data into per-second buckets. */
    for (int i = 0; i < PROF_PHASES; i++) {
        s_sec_us[i] += s_frame_us[i];
    }
    memset(s_frame_us, 0, sizeof(s_frame_us));
    s_sec_frames++;
    if (skipped) {
        s_sec_skipped++;
    }

    /* Initialise the timer on the first call. */
    int64_t now_us = esp_timer_get_time();
    if (s_sec_start_us == 0) {
        s_sec_start_us = now_us;
        return;
    }

    /* Print once per second. */
    if ((now_us - s_sec_start_us) < 1000000) {
        return;
    }

    /* Convert accumulated µs → milliseconds per frame. */
    float inv_frames = (s_sec_frames > 0) ? (1.0f / (float)s_sec_frames) : 0.0f;

    float ms_frame = (float)s_sec_us[PROF_FRAME] * inv_frames * 0.001f;
    float ms_lcd   = (float)s_sec_us[PROF_LCD]   * inv_frames * 0.001f;
    float ms_flip  = (float)s_sec_us[PROF_FLIP]  * inv_frames * 0.001f;
    float ms_touch = (float)s_sec_us[PROF_TOUCH] * inv_frames * 0.001f;
    float ms_cpu   = ms_frame - ms_lcd; /* pure CPU stepping */

    /* Drain the audio accumulator atomically. */
    uint32_t audio_us = atomic_exchange_explicit(&s_audio_sec_us, 0u,
                                                 memory_order_relaxed);
    float ms_audio = (float)audio_us * inv_frames * 0.001f;

    float ms_total = ms_cpu + ms_lcd + ms_touch;

    const float budget_ms = 17.7f;
    float budget_pct = 100.0f * ms_total / budget_ms;

    ESP_LOGI(TAG, "%" PRIu32 " fr (%" PRIu32 " skip) "
             "| cpu %5.2fms  lcd %5.2fms  flip %5.2fms  touch %5.2fms  audio %5.2fms "
             "| total %5.2fms / %.2fms budget (%4.1f%%)",
             s_sec_frames, s_sec_skipped,
             ms_cpu, ms_lcd, ms_flip, ms_touch, ms_audio,
             ms_total, budget_ms, budget_pct);

    /* Reset for next interval. */
    memset(s_sec_us, 0, sizeof(s_sec_us));
    s_sec_frames  = 0;
    s_sec_skipped = 0;
    s_sec_start_us = now_us;
}
