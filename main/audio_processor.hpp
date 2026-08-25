#pragma once

#include <cstddef>
#include <cstdint>

#include "app_config.h"

class AudioProcessor {
public:
    void process_tx_frame(int16_t *pcm, size_t samples);
    void process_rx_frame(int16_t *pcm, size_t samples);
    void reset();

private:
    void apply_dc_block(int16_t *pcm, size_t samples);
    void apply_preemphasis(int16_t *pcm, size_t samples);
    void apply_noise_gate(int16_t *pcm, size_t samples);
    void apply_noise_suppress(int16_t *pcm, size_t samples);
    void apply_agc(int16_t *pcm, size_t samples);
    void apply_voice_bandpass(int16_t *pcm, size_t samples);
    void apply_deemphasis(int16_t *pcm, size_t samples);
    void apply_soft_limiter(int16_t *pcm, size_t samples);

    // DC block state
    int32_t dc_x_prev_ = 0;
    int32_t dc_y_prev_ = 0;

    // Pre/de-emphasis state
    int16_t preemph_prev_ = 0;
    int16_t deemph_prev_ = 0;

    // Noise gate state
    int32_t gate_smooth_ = 0;
    uint8_t gate_hold_ = 0;
    uint8_t tx_mute_remaining_ = 0;

    // AGC state
    int32_t agc_gain_q15_ = 1 << 15;

    // Noise suppression state
    int32_t ns_floor_q15_ = 0;

    // Voice bandpass biquad state (2nd-order IIR)
    int32_t bp_x1_ = 0;
    int32_t bp_x2_ = 0;
    int32_t bp_y1_ = 0;
    int32_t bp_y2_ = 0;
};
