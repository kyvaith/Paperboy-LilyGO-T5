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

/* Long-window CPU-only accumulator for comparing emulator cores. */
static int64_t  s_speed_cpu_us;
static uint32_t s_speed_frames;
static int64_t  s_speed_start_us;

/*
 * Audio task accumulator — written by the audio task, read+reset by the
 * emulator task in profiler_frame_end().  Uses an atomic uint32_t so that
 * the single-word load/store on Xtensa LX7 is naturally race-free.
 * Max value per second: ~60 calls × ~16 ms = ~960 000 µs < UINT32_MAX.
 */
static _Atomic uint32_t s_audio_sec_us = 0;

static float profiler_dmg_relative_speed(float avg_cpu_ms)
{
    return (avg_cpu_ms > 0.0f) ? (16.67f / avg_cpu_ms) : 0.0f;
}

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
    int64_t frame_cpu_us = s_frame_us[PROF_FRAME] - s_frame_us[PROF_LCD];
    if (frame_cpu_us < 0) {
        frame_cpu_us = 0;
    }

    /* Accumulate this frame's data into per-second buckets. */
    for (int i = 0; i < PROF_PHASES; i++) {
        s_sec_us[i] += s_frame_us[i];
    }
    s_speed_cpu_us += frame_cpu_us;
    s_speed_frames++;
    memset(s_frame_us, 0, sizeof(s_frame_us));
    s_sec_frames++;
    if (skipped) {
        s_sec_skipped++;
    }

    /* Initialise the timer on the first call. */
    int64_t now_us = esp_timer_get_time();
    if (s_sec_start_us == 0) {
        s_sec_start_us = now_us;
    }
    if (s_speed_start_us == 0) {
        s_speed_start_us = now_us;
    }

    /* Print once per second. */
    if ((now_us - s_sec_start_us) >= 1000000) {
        float inv_frames = (s_sec_frames > 0) ? (1.0f / (float)s_sec_frames) : 0.0f;

        float ms_frame = (float)s_sec_us[PROF_FRAME] * inv_frames * 0.001f;
        float ms_lcd   = (float)s_sec_us[PROF_LCD]   * inv_frames * 0.001f;
        float ms_flip  = (float)s_sec_us[PROF_FLIP]  * inv_frames * 0.001f;
        float ms_touch = (float)s_sec_us[PROF_TOUCH] * inv_frames * 0.001f;
        float ms_cpu   = ms_frame - ms_lcd; /* pure CPU stepping */

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

        memset(s_sec_us, 0, sizeof(s_sec_us));
        s_sec_frames = 0;
        s_sec_skipped = 0;
        s_sec_start_us = now_us;
    }

#if PAPERBOY_PROF_SPEED_WINDOW_MS > 0
    if ((now_us - s_speed_start_us) >= ((int64_t)PAPERBOY_PROF_SPEED_WINDOW_MS * 1000)) {
        float inv_frames = (s_speed_frames > 0) ? (1.0f / (float)s_speed_frames) : 0.0f;
        float avg_cpu_ms = (float)s_speed_cpu_us * inv_frames * 0.001f;
        float rel_speed = profiler_dmg_relative_speed(avg_cpu_ms);
        float window_s = (float)(now_us - s_speed_start_us) * 0.000001f;

        ESP_LOGI(TAG, "avg cpu speed over %.1fs (%" PRIu32 " fr): %.2fx DMG "
                 "(avg cpu %.2fms/fr)",
                 window_s, s_speed_frames, rel_speed, avg_cpu_ms);

        s_speed_cpu_us = 0;
        s_speed_frames = 0;
        s_speed_start_us = now_us;
    }
#endif
}
