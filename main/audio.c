/**
 * audio.c — Runtime audio engine dispatcher for Paperboy / M5PaperS3
 *
 * Three engines are available, selected by audio_set_engine() before the
 * first call to audio_init():
 *
 *   AUDIO_ENGINE_PCM  — PCM output via the build-selected backend
 *                       (audio_pdm.c  = I2S PDM,  or
 *                        audio_pwm.c  = LEDC PCM ISR)
 *   AUDIO_ENGINE_POLY — Polyphonic square-wave buzzer (audio_poly_pwm.c)
 *   AUDIO_ENGINE_MUTE — No audio hardware; APU register state is still
 *                       maintained so emulated games run correctly.
 *
 * After audio_init() is called the selection is frozen for the session.
 */

#include "audio.h"
#include "minigb_apu/minigb_apu.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ── Forward declarations: PCM backend (audio_pdm.c or audio_pwm.c) ──── */
void    audio_pcm_init(void);
void    audio_pcm_service_frame(void);
void    audio_pcm_push_samples(const int16_t *samples, size_t n_pairs);
size_t  audio_pcm_ring_free(void);
uint8_t audio_pcm_apu_read(uint16_t addr);
void    audio_pcm_apu_write(uint16_t addr, uint8_t val);
void    audio_pcm_deinit(void);

/* ── Forward declarations: polyphony backend (audio_poly_pwm.c) ────────  */
void    audio_poly_init(void);
void    audio_poly_service_frame(void);
void    audio_poly_push_samples(const int16_t *samples, size_t n_pairs);
size_t  audio_poly_ring_free(void);
uint8_t audio_poly_apu_read(uint16_t addr);
void    audio_poly_apu_write(uint16_t addr, uint8_t val);
void    audio_poly_deinit(void);

/* ── Mute-mode APU ────────────────────────────────────────────────────── */

/* A minimal APU context kept alive in mute mode so that emulated register
 * reads/writes work correctly even with no hardware output. */
static struct minigb_apu_ctx s_mute_apu;

/* ── Engine state ─────────────────────────────────────────────────────── */

static audio_engine_t s_engine  = AUDIO_ENGINE_PCM;
static bool           s_locked  = false;   /* set to true after audio_init() */

/* ── Public selection API ─────────────────────────────────────────────── */

void audio_set_engine(audio_engine_t engine)
{
    if (s_locked) {
        return;   /* too late — engine is already running */
    }
    if (engine < AUDIO_ENGINE_COUNT) {
        s_engine = engine;
    }
}

audio_engine_t audio_get_engine(void)
{
    return s_engine;
}

const char *audio_engine_name(audio_engine_t engine)
{
    switch (engine) {
        case AUDIO_ENGINE_PCM:  return "PCM";
        case AUDIO_ENGINE_POLY: return "Polyphony";
        case AUDIO_ENGINE_MUTE: return "Mute";
        default:                return "?";
    }
}

/* ── Dispatcher implementations ─────────────────────────────────────── */

void audio_init(void)
{
    s_locked = true;
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:
            audio_pcm_init();
            break;
        case AUDIO_ENGINE_POLY:
            audio_poly_init();
            break;
        case AUDIO_ENGINE_MUTE:
        default:
            minigb_apu_audio_init(&s_mute_apu);
            break;
    }
}

void audio_service_frame(void)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  audio_pcm_service_frame();  break;
        case AUDIO_ENGINE_POLY: audio_poly_service_frame(); break;
        default: break;
    }
}

void audio_push_samples(const int16_t *samples, size_t n_pairs)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  audio_pcm_push_samples(samples, n_pairs);  break;
        case AUDIO_ENGINE_POLY: audio_poly_push_samples(samples, n_pairs); break;
        default: (void)samples; (void)n_pairs; break;
    }
}

size_t audio_ring_free(void)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  return audio_pcm_ring_free();
        case AUDIO_ENGINE_POLY: return audio_poly_ring_free();
        default:                return (size_t)-1;   /* always "free" */
    }
}

uint8_t audio_apu_read(uint16_t addr)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  return audio_pcm_apu_read(addr);
        case AUDIO_ENGINE_POLY: return audio_poly_apu_read(addr);
        default:                return minigb_apu_audio_read(&s_mute_apu, addr);
    }
}

void audio_apu_write(uint16_t addr, uint8_t val)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  audio_pcm_apu_write(addr, val);  break;
        case AUDIO_ENGINE_POLY: audio_poly_apu_write(addr, val); break;
        default:                minigb_apu_audio_write(&s_mute_apu, addr, val); break;
    }
}

void audio_deinit(void)
{
    switch (s_engine) {
        case AUDIO_ENGINE_PCM:  audio_pcm_deinit();  break;
        case AUDIO_ENGINE_POLY: audio_poly_deinit(); break;
        default: break;
    }
    s_locked = false;
}
