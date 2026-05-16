/**
 * audio.c — I2S PDM TX audio output for M5PaperS3 buzzer
 *
 * Architecture:
 *   emulator task
 *       → writes APU registers via audio_apu_read/write()
 *         (these update s_apu, which lives here)
 *
 *   audio output task  (owns the APU synthesis loop)
 *       → whenever ring has room for ≥ AUDIO_SAMPLES mono samples:
 *             minigb_apu_audio_callback() → down-mix → ring buffer
 *       → drains ring buffer in DMA_CHUNK-sample blocks
 *       → i2s_channel_write() → I2S PDM TX DMA → GPIO → RC → buzzer
 *
 * Decoupling: the APU runs at the I2S hardware clock rate (~59.7 calls/sec
 * in steady state) regardless of emulation speed.  The emulator only writes
 * register state; it never calls the synthesis callback itself.
 *
 * Concurrency: both tasks share core 0.  FreeRTOS single-core preemption
 * guarantees that the audio task (higher priority) and the emulator task
 * never truly run simultaneously, so s_apu access is race-free.
 */

#include "audio.h"

#include <string.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_pdm.h"
#include "driver/i2s_common.h"

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "minigb_apu/minigb_apu.h"
#include "profiler.h"

static const char *TAG = "audio";

/* -----------------------------------------------------------------------
 * APU state (owned here; emulator writes registers via audio_apu_read/write)
 * --------------------------------------------------------------------- */

static struct minigb_apu_ctx DRAM_ATTR s_apu;

/*
 * Scratch buffer for one APU frame (stereo interleaved int16).
 * Kept at module scope (DRAM) to avoid bloating the audio task's stack.
 * Size is AUDIO_SAMPLES_TOTAL ≈ 1100; use a fixed compile-time constant.
 */
#define APU_BUF_SIZE  1100u   /* ≥ AUDIO_SAMPLES_TOTAL at 32768 Hz / 59.73 Hz */
static int16_t DRAM_ATTR s_apu_buf[APU_BUF_SIZE];

/* -----------------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------------- */

/* Buzzer GPIO, configurable via menuconfig (default GPIO 45 on M5PaperS3). */
#define BUZZER_GPIO     21

/*
 * Ring buffer holds CONFIG_PAPERBOY_AUDIO_RING_FRAMES GB frames of mono
 * samples.  Each frame is AUDIO_SAMPLES pairs → AUDIO_SAMPLES mono samples.
 * Size must be a power of two for the lock-free index arithmetic.
 *
 * AUDIO_SAMPLES ≈ 548.  6 frames ≈ 3288 samples; round up to 4096.
 */
#define RING_SIZE_FRAMES    CONFIG_PAPERBOY_AUDIO_RING_FRAMES
#define RING_SIZE_RAW       (RING_SIZE_FRAMES * AUDIO_SAMPLES)

/* Next power-of-two ≥ RING_SIZE_RAW, evaluated at compile time. */
static inline uint32_t next_pow2(uint32_t v)
{
    v--;
    v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16;
    return v + 1;
}
/* Work around "variable-length array" restrictions: use a generous fixed
 * power-of-two that is always ≥ 16 * AUDIO_SAMPLES (≤ 8800).            */
#define RING_SIZE   8192u   /* 8 K mono int16 samples, ~250 ms @ 32768 Hz */
#define RING_MASK   (RING_SIZE - 1u)

/*
 * DMA write chunk: 512 samples (1024 bytes).
 * At 32768 Hz this is ~15.6 ms, so the output task loops ~64×/sec — well
 * below any meaningful interrupt pressure on the emulator core.
 */
#define DMA_CHUNK   512u

/* Audio output task */
#define AUDIO_TASK_STACK    4096
#define AUDIO_TASK_PRIORITY (configMAX_PRIORITIES - 2)

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
 * I2S / task state
 * --------------------------------------------------------------------- */

static i2s_chan_handle_t s_tx_chan  = NULL;
static TaskHandle_t      s_task    = NULL;
static volatile bool     s_running = false;

/* -----------------------------------------------------------------------
 * Audio output task
 * --------------------------------------------------------------------- */

static void audio_task(void *arg)
{
    (void)arg;

    /* DMA write buffer: static so it doesn't consume task stack. */
    static int16_t dma_buf[DMA_CHUNK];

    while (s_running) {
        int64_t t_audio_start = esp_timer_get_time();

        /*
         * Step 1 — APU synthesis.
         *
         * Call the APU callback whenever there is room for at least one full
         * frame of mono samples.  This clocks the APU at the I2S hardware
         * rate (≈59.7 calls/sec in steady state) rather than at the variable
         * emulation frame rate, eliminating sample-rate-mismatch stutter.
         *
         * minigb_apu_audio_callback() always produces exactly AUDIO_SAMPLES
         * stereo pairs (≈548 at 32768 Hz / 59.73 Hz).  We down-mix to mono
         * and push only AUDIO_SAMPLES values into the ring.
         */
        if (ring_free_count() >= AUDIO_SAMPLES) {
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

        /*
         * Step 2 — Drain ring into DMA write buffer.
         * Pad with silence if the ring doesn't have a full chunk yet
         * (can happen briefly at startup or after a long underrun).
         */
        uint32_t avail = ring_count();
        uint32_t n     = (avail < DMA_CHUNK) ? avail : DMA_CHUNK;

        if (n > 0) {
            uint32_t tail = atomic_load_explicit(&s_ring_tail, memory_order_relaxed);
            for (uint32_t i = 0; i < n; i++) {
                dma_buf[i] = s_ring[(tail + i) & RING_MASK];
            }
            atomic_store_explicit(&s_ring_tail, tail + n, memory_order_release);
        }

        /* Synthesis + ring-fill is now complete — record CPU-busy time before
         * blocking on i2s_channel_write() (the emulator runs during that wait). */
        profiler_audio_add(esp_timer_get_time() - t_audio_start);

        if (n < DMA_CHUNK) {
            memset(&dma_buf[n], 0, (DMA_CHUNK - n) * sizeof(int16_t));
            if (n == 0) {
                static uint32_t s_underrun_count = 0;
                if ((++s_underrun_count & 0x3F) == 0) {
                    ESP_LOGD(TAG, "audio underrun #%lu",
                             (unsigned long)s_underrun_count);
                }
            }
        }

        /* Step 3 — Write to I2S DMA (blocks until buffer slot accepted). */
        size_t written = 0;
        esp_err_t err = i2s_channel_write(s_tx_chan,
                                          dma_buf,
                                          DMA_CHUNK * sizeof(int16_t),
                                          &written,
                                          portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_write: %s", esp_err_to_name(err));
        }
    }

    vTaskDelete(NULL);
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void audio_init(void)
{
    /* --- I2S channel -------------------------------------------------- */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO,
                                                            I2S_ROLE_MASTER);
    /*
     * 8 DMA descriptors × 512 frames = 4 K samples = ~125 ms internal DMA
     * depth.  auto_clear outputs silence instead of stale data on underrun
     * at the hardware level (belt-and-suspenders alongside our ring buffer).
     */
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = DMA_CHUNK;
    chan_cfg.auto_clear    = true;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    /* --- PDM TX mode -------------------------------------------------- */
    i2s_pdm_tx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_TX_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk          = GPIO_NUM_NC,   /* no external PDM clock needed */
            .dout         = (gpio_num_t)BUZZER_GPIO,
            .invert_flags = { .clk_inv = false },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(s_tx_chan, &pdm_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

    /* --- APU ---------------------------------------------------------- */
    minigb_apu_audio_init(&s_apu);

    ESP_LOGI(TAG, "I2S PDM TX started: GPIO%d, %u Hz, ring=%u samples (~%u ms)",
             BUZZER_GPIO, AUDIO_SAMPLE_RATE, RING_SIZE,
             (unsigned)(RING_SIZE * 1000u / AUDIO_SAMPLE_RATE));

    /* --- Output task -------------------------------------------------- */
    s_running = true;
    BaseType_t rc = xTaskCreatePinnedToCore(audio_task,
                                            "audio_out",
                                            AUDIO_TASK_STACK,
                                            NULL,
                                            AUDIO_TASK_PRIORITY,
                                            &s_task,
                                            0 /* PRO_CPU – same core as emulator,
                                               * but blocks in i2s_channel_write()
                                               * for ~15.6 ms between ~2 µs bursts,
                                               * so CPU overhead is < 0.01%. */);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
    }
}

void audio_push_samples(const int16_t *samples, size_t n_pairs)
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

size_t audio_ring_free(void)
{
    return (size_t)ring_free_count();
}

uint8_t audio_apu_read(uint16_t addr)
{
    return minigb_apu_audio_read(&s_apu, addr);
}

void audio_apu_write(uint16_t addr, uint8_t val)
{
    minigb_apu_audio_write(&s_apu, addr, val);
}

void audio_deinit(void)
{
    s_running = false;

    if (s_tx_chan) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }

    /* The task deletes itself once s_running is false; give it a moment. */
    vTaskDelay(pdMS_TO_TICKS(50));
    s_task = NULL;
}
