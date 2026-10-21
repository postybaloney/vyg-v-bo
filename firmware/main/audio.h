#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define AUDIO_RATE_HZ       16000
#define AUDIO_FRAME_SAMPLES 160

// Mic stream starts on init. Playback is a 100 ms ring: prefill 40 ms, silence
// on underrun, drop or duplicate one sample when the fill drifts.
esp_err_t audio_init(void);
bool audio_take_frame(int16_t samples[AUDIO_FRAME_SAMPLES], uint32_t *seq);
void audio_play(const int16_t *samples, int n, uint32_t seq);
