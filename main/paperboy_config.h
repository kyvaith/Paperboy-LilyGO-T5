#pragma once

/*
 * Local build-time tuning knobs.
 *
 * Keep these in a normal header so iteration does not require regenerating
 * sdkconfig or touching the full Kconfig dependency graph.
 */

// Currently frame time is 17.7ms (~57fps)
// Slightly lower than 59.7 so we are more likely to hit it
#define PAPERBOY_BUZZER_GPIO                    21

/*
 * PCM audio settings (used by audio.c / audio_pwm.c only).
 * Not referenced by audio_poly_pwm.c.
 */
#define PAPERBOY_AUDIO_RING_FRAMES              6u
#define PAPERBOY_AUDIO_TARGET_SAMPLES_PER_FRAME 704u
#define PAPERBOY_AUDIO_BUDGET_MS                2