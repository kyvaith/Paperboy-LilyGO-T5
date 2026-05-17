/**
 * audio.c — LEDC PWM audio output for M5PaperS3 buzzer
 *
 * Architecture:
 *   emulator task / audio_service_frame()
 *       → minigb_apu_audio_callback() → ring buffer  (producer)
 *
 *   LEDC timer-0 OVF ISR  (fires at AUDIO_SAMPLE_RATE = 32768 Hz)
 *       → pops one mono int16 sample from ring buffer  (consumer)
 *       → maps it to a 10-bit duty cycle (0–1023)
 *       → writes duty directly via LEDC LL registers (ISR-safe)
 *
 * The I2S PDM TX path is replaced by LEDC PWM at ~32.768 kHz / 10-bit.
 * The buzzer + 2N7002W see a ~32 kHz square wave whose duty encodes
 * amplitude — well within the MOSFET's bandwidth and the buzzer's response.
 *
 * ISR overhead: ~80 Xtensa cycles at 240 MHz ≈ 0.7 % of one core.
 * No DMA, no secondary task, no FreeRTOS involvement in the hot path.
 */

#include "audio.h"

#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"

#include "driver/ledc.h"
#include "hal/ledc_ll.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#include "paperboy_config.h"
#include "minigb_apu/minigb_apu.h"
#include "profiler.h"

static const char *TAG = "audio";

/* -----------------------------------------------------------------------
 * APU state (owned here; emulator writes registers via audio_apu_read/write)
 * --------------------------------------------------------------------- */

static struct minigb_apu_ctx DRAM_ATTR s_apu;

/*
 * Scratch buffer for one APU frame (stereo interleaved int16).
 * Kept at module scope (DRAM) to avoid bloating the stack.
 */
#define APU_BUF_SIZE  (AUDIO_SAMPLES * 2)
static int16_t DRAM_ATTR s_apu_buf[APU_BUF_SIZE];

/* -----------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------- */

/* Buzzer GPIO, configurable via menuconfig. */
#define BUZZER_GPIO     PAPERBOY_BUZZER_GPIO

/* LEDC peripheral configuration. */
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   /* 1024 steps */
#define LEDC_DUTY_BITS  10u
#define LEDC_DUTY_MAX   (1u << LEDC_DUTY_BITS)  /* 1024          */
#define LEDC_DUTY_MID   (LEDC_DUTY_MAX >> 1u)   /* 512 = silence */

/*
 * Timer overflow interrupt bit in LEDC INT_ST / INT_ENA / INT_CLR.
 * On ESP32-S3 the register layout is (ledc_struct.h):
 *   bits  0-3  : lstimer0..3_ovf   ← our timer OVF lives here
 *   bits  4-11 : duty_chng_end_lsch0..7
 *   bits 12-19 : ovf_cnt_lsch0..7
 * (Different from the "generic" ESP32 layout documented in older TRMs.)
 */
#define LEDC_OVF_MASK  (1u << (unsigned)(LEDC_TIMER))

/*
 * Ring buffer holds 8192 mono int16 samples (~250 ms at 32768 Hz).
 * Size must be a power of two for lock-free index masking.
 */
#define RING_SIZE   8192u
#define RING_MASK   (RING_SIZE - 1u)

#define AUDIO_TARGET_SAMPLES_PER_FRAME  PAPERBOY_AUDIO_TARGET_SAMPLES_PER_FRAME
#define AUDIO_BUDGET_US                ((int64_t)PAPERBOY_AUDIO_BUDGET_MS * 1000)

/* -----------------------------------------------------------------------
 * Ring buffer  (single-producer, single-consumer, lock-free)
 *
 * s_ring_head  written only by producer (audio_push_samples)
 * s_ring_tail  written only by consumer (audio output task)
 *
 * On Xtensa LX7, 32-bit aligned loads/stores are naturally atomic; the
 * acquire/release memory orders enforce the necessary ordering between the
 * data writes and the index updates.
 * --------------------------------------------------------------------- */

static int16_t DRAM_ATTR s_ring[RING_SIZE];
static _Atomic uint32_t  s_ring_head = 0;   /* next write index (producer) */
static _Atomic uint32_t  s_ring_tail = 0;   /* next read  index (consumer) */

static inline uint32_t ring_count(void)
{
    return atomic_load_explicit(&s_ring_head, memory_order_acquire) -
           atomic_load_explicit(&s_ring_tail, memory_order_relaxed);
}

static inline uint32_t ring_free_count(void)
{
    return RING_SIZE - ring_count();
}

/* -----------------------------------------------------------------------
 * LEDC / module state
 * --------------------------------------------------------------------- */

static bool          s_initialised = false;
static intr_handle_t s_intr_handle = NULL;
static uint32_t      s_pending_target_samples = 0;

/* -----------------------------------------------------------------------
 * LEDC OVF ISR — fires once per PWM period at AUDIO_SAMPLE_RATE (32768 Hz)
 *
 * Placed in IRAM: no flash-cache dependency, no driver calls, no allocation.
 * Target: < 100 Xtensa LX7 cycles per invocation.
 *
 * Timing model:
 *   Period N wraps (OVF fires) → we write duty[N] + set low_speed_update
 *   Period N+1 wraps           → hardware latches duty[N]; output shows it
 * One-period pipeline latency ≈ 30 µs — completely inaudible.
 * --------------------------------------------------------------------- */

static IRAM_ATTR void ledc_audio_isr(void *arg)
{
    /* Ignore spurious interrupts from other LEDC timers/channels. */
    if (!(LEDC.int_st.val & LEDC_OVF_MASK)) {
        return;
    }
    /* W1C: write only our bit so we don't disturb other pending flags. */
    LEDC.int_clr.val = LEDC_OVF_MASK;

    /*
     * Pop one sample from the lock-free ring buffer.
     *   tail is written only by us (ISR) → relaxed load is safe.
     *   head is written by the producer with release → acquire here.
     */
    uint32_t tail = atomic_load_explicit(&s_ring_tail, memory_order_relaxed);
    uint32_t head = atomic_load_explicit(&s_ring_head, memory_order_acquire);

    uint32_t duty;
    if (head != tail) {
        int16_t sample = s_ring[tail & RING_MASK];
        atomic_store_explicit(&s_ring_tail, tail + 1u, memory_order_release);

        /*
         * Map signed int16 → 10-bit unsigned duty:
         *   -32768 → 0,   0 → 512,   +32767 → 1023
         * Arithmetic shift gives -512..511; bias to 0..1023.
         * Clamp handles the ±1 asymmetry of two's complement.
         */
        int32_t d = (int32_t)(sample >> 6) + (int32_t)LEDC_DUTY_MID;
        if (d < 0) {
            duty = 0u;
        } else if (d >= (int32_t)LEDC_DUTY_MAX) {
            duty = LEDC_DUTY_MAX - 1u;
        } else {
            duty = (uint32_t)d;
        }
    } else {
        /* Ring underrun — hold DC midpoint (silence, no offset). */
        duty = LEDC_DUTY_MID;
    }

    /*
     * Write duty via LEDC LL inline functions (direct register writes,
     * no locking needed — only this ISR touches this channel's duty).
     *
     * ledc_ll_set_duty_int_part: stores duty << 4 in the shadow latch.
     * ledc_ll_ls_channel_update: sets conf0.low_speed_update, causing the
     *   hardware to transfer the shadow latch at the next timer wrap.
     */
    ledc_ll_set_duty_int_part(&LEDC, LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_ll_ls_channel_update(&LEDC, LEDC_MODE, LEDC_CHANNEL);
}

/* -----------------------------------------------------------------------
 * Internal helpers
 * --------------------------------------------------------------------- */

static void audio_generate_chunk_locked(void)
{
    minigb_apu_audio_callback(&s_apu, s_apu_buf);

    uint32_t head = atomic_load_explicit(&s_ring_head, memory_order_relaxed);
    for (uint32_t i = 0; i < AUDIO_SAMPLES; i++) {
        int32_t mono = ((int32_t)s_apu_buf[i * 2] +
                        (int32_t)s_apu_buf[i * 2 + 1]) >> 1;
        s_ring[(head + i) & RING_MASK] = (int16_t)mono;
    }
    atomic_store_explicit(&s_ring_head, head + AUDIO_SAMPLES,
                          memory_order_release);
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void audio_pcm_init(void)
{
    if (s_initialised) {
        atomic_store_explicit(&s_ring_head, 0, memory_order_relaxed);
        atomic_store_explicit(&s_ring_tail, 0, memory_order_relaxed);
        s_pending_target_samples = 0;
        minigb_apu_audio_init(&s_apu);
        return;
    }

    /* --- LEDC timer --------------------------------------------------- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = AUDIO_SAMPLE_RATE,
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
        .duty           = LEDC_DUTY_MID,   /* start at silence */
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    /* --- Ring buffer + APU -------------------------------------------- */
    atomic_store_explicit(&s_ring_head, 0, memory_order_relaxed);
    atomic_store_explicit(&s_ring_tail, 0, memory_order_relaxed);
    s_pending_target_samples = 0;
    minigb_apu_audio_init(&s_apu);

    /* --- LEDC OVF interrupt ------------------------------------------- */
    /*
     * Enable the timer overflow interrupt.  INT_ENA is a plain RW register;
     * the read-modify-write is safe here since we're the only writer during
     * init.  LEDC_OVF_MASK = bit (16 + LEDC_TIMER) per the TRM.
     */
    LEDC.int_ena.val |= LEDC_OVF_MASK;

    /*
     * Allocate the interrupt.  ESP_INTR_FLAG_IRAM ensures the handler runs
     * from IRAM and remains callable even when the flash cache is disabled.
     * No level flag → defaults to level 1 (easily upgradeable to level 4/5
     * later for lower latency via xt_highint4 if needed).
     */
    ESP_ERROR_CHECK(esp_intr_alloc(ETS_LEDC_INTR_SOURCE,
                                   ESP_INTR_FLAG_IRAM,
                                   ledc_audio_isr,
                                   NULL,
                                   &s_intr_handle));

    ESP_LOGI(TAG, "LEDC PWM audio: GPIO%d, %" PRIu32 " Hz actual, 10-bit duty, "
                  "ring=%u samples (~%u ms)",
             BUZZER_GPIO,
             ledc_get_freq(LEDC_MODE, LEDC_TIMER),
             RING_SIZE,
             (unsigned)(RING_SIZE * 1000u / AUDIO_SAMPLE_RATE));

    s_initialised = true;
}

void audio_pcm_service_frame(void)
{
    if (!s_initialised) {
        return;
    }

    int64_t start_us = esp_timer_get_time();

    s_pending_target_samples += AUDIO_TARGET_SAMPLES_PER_FRAME;

    while (s_pending_target_samples >= AUDIO_SAMPLES &&
           ring_free_count() >= AUDIO_SAMPLES) {
        if (AUDIO_BUDGET_US > 0 && (esp_timer_get_time() - start_us) >= AUDIO_BUDGET_US) {
            break;
        }

        audio_generate_chunk_locked();
        s_pending_target_samples -= AUDIO_SAMPLES;
    }

    /* No pump call needed — the LEDC OVF ISR is the ring consumer. */
    profiler_audio_add(esp_timer_get_time() - start_us);

    if (AUDIO_BUDGET_US > 0) {
        int64_t elapsed_us = esp_timer_get_time() - start_us;
        if (elapsed_us < AUDIO_BUDGET_US) {
            ets_delay_us((uint32_t)(AUDIO_BUDGET_US - elapsed_us));
        }
    }
}

void audio_pcm_push_samples(const int16_t *samples, size_t n_pairs)
{
    if (n_pairs == 0) {
        return;
    }

    uint32_t head  = atomic_load_explicit(&s_ring_head, memory_order_relaxed);
    uint32_t free_ = RING_SIZE -
                     (head - atomic_load_explicit(&s_ring_tail, memory_order_acquire));

    /* Clamp to available space — drop excess silently (emulator too fast). */
    if (n_pairs > free_) {
        n_pairs = free_;
    }

    for (size_t i = 0; i < n_pairs; i++) {
        /* Down-mix stereo → mono: (L + R) / 2, avoiding overflow. */
        int32_t mono = ((int32_t)samples[i * 2] + (int32_t)samples[i * 2 + 1]) >> 1;
        s_ring[(head + (uint32_t)i) & RING_MASK] = (int16_t)mono;
    }

    atomic_store_explicit(&s_ring_head, head + (uint32_t)n_pairs,
                          memory_order_release);
}

size_t audio_pcm_ring_free(void)
{
    return (size_t)ring_free_count();
}

uint8_t audio_pcm_apu_read(uint16_t addr)
{
    return minigb_apu_audio_read(&s_apu, addr);
}

void audio_pcm_apu_write(uint16_t addr, uint8_t val)
{
    minigb_apu_audio_write(&s_apu, addr, val);
}

void audio_pcm_deinit(void)
{
    s_initialised = false;

    /* Disable the OVF interrupt before freeing the handler. */
    if (s_intr_handle) {
        LEDC.int_ena.val &= ~LEDC_OVF_MASK;
        esp_intr_free(s_intr_handle);
        s_intr_handle = NULL;
    }

    ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
}
