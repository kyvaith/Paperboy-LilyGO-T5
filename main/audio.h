#pragma once

#include <stdint.h>
#include <stddef.h>

/**
 * Initialize I2S PDM TX audio output.
 *
 * Allocates the internal ring buffer, configures the I2S peripheral in PDM TX
 * mode, and starts the audio output task.  Must be called once before any
 * other audio_* functions.
 */
void audio_init(void);

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
