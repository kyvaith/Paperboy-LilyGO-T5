#pragma once

#include <stdint.h>
#include <stddef.h>

/* ── Runtime audio engine selection ─────────────────────────────────────────
 *
 * Must be set before the first audio_init() call (i.e. before the game is
 * launched).  Once audio_init() has been called the selection is frozen for
 * the lifetime of the session.
 */
typedef enum {
    AUDIO_ENGINE_PCM  = 0,  /* built-in PCM backend (PDM or PWM, build-time choice) */
    AUDIO_ENGINE_POLY = 1,  /* polyphonic square-wave buzzer (audio_poly_pwm) */
    AUDIO_ENGINE_MUTE = 2,  /* no audio output */
    AUDIO_ENGINE_COUNT,
} audio_engine_t;

/**
 * Select the audio engine.  Must be called before audio_init(); calling it
 * afterwards has no effect.
 */
void audio_set_engine(audio_engine_t engine);

/** Return the currently selected audio engine. */
audio_engine_t audio_get_engine(void);

/** Return a short human-readable name for the given engine (never NULL). */
const char *audio_engine_name(audio_engine_t engine);

/**
 * Initialize I2S PDM TX audio output.
 *
 * Configures the I2S peripheral in PDM TX mode, resets the APU state, and
 * prepares the frame-driven audio path. Must be called once before any other
 * audio_* functions.
 */
void audio_init(void);

/**
 * Run the frame-budgeted audio service.
 *
 * Intended to be called from the CPU1 display loop once per display frame.
 * It synthesizes queued APU samples, pushes them into the software ring, pumps
 * as much queued PCM into the I2S DMA as possible without blocking, and pads
 * the configured audio budget when it finishes early.
 */
void audio_service_frame(void);

/**
 * Push one Game Boy frame worth of stereo PCM samples into the ring buffer.
 *
 * Down-mixes the interleaved stereo pairs to mono before queuing.  Excess
 * samples are silently dropped when the ring buffer is full (emulator running
 * faster than real-time).
 *
 * @param samples   Interleaved stereo int16 pairs: [L0, R0, L1, R1, ...]
 *                  produced by minigb_apu_audio_callback().
 * @param n_pairs   Number of stereo pairs (AUDIO_SAMPLES_TOTAL / 2).
 */
void audio_push_samples(const int16_t *samples, size_t n_pairs);

/**
 * Return the number of free mono sample slots currently available in the ring
 * buffer.
 *
 * This can be used by the emulator to decide whether to call
 * minigb_apu_audio_callback() independently of the video frame cadence, so
 * that the APU is driven at the audio output rate rather than the (variable)
 * emulation speed.
 */
size_t audio_ring_free(void);

/**
 * Read an APU register on behalf of the emulator CPU.
 * Wraps minigb_apu_audio_read(); addr must be in 0xFF10–0xFF3F.
 */
uint8_t audio_apu_read(uint16_t addr);

/**
 * Write an APU register on behalf of the emulator CPU.
 * Wraps minigb_apu_audio_write(); addr must be in 0xFF10–0xFF3F.
 */
void audio_apu_write(uint16_t addr, uint8_t val);

/**
 * Deinitialize audio output.
 *
 * Stops the output task, disables and deletes the I2S channel, and frees the
 * ring buffer.
 */
void audio_deinit(void);
