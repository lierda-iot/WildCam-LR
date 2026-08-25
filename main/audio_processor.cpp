#include "audio_processor.hpp"

#include <cstdlib>
#include <cstring>

namespace {

constexpr int32_t kQ15One = 1 << 15;
constexpr int32_t kDcAlpha = 31785; // ~0.97 in Q15, fc ≈ 80 Hz @ 16 kHz
constexpr int32_t kPreemphAlpha = APP_AUDIO_DSP_PREEMPH_ALPHA_Q15;
constexpr int32_t kGateThresh = APP_AUDIO_DSP_NOISE_GATE_THRESH;
constexpr int32_t kLimiterThresh = APP_AUDIO_DSP_LIMITER_THRESHOLD;
constexpr int32_t kAgcTarget = APP_AUDIO_DSP_AGC_TARGET_Q15;
constexpr int32_t kAgcMaxGain = APP_AUDIO_DSP_AGC_MAX_GAIN_Q15;

// Voice bandpass biquad coefficients (Q15), ~300 Hz – 3400 Hz @ 16 kHz.
// Designed as a 2nd-order Butterworth BPF, center ~1200 Hz, BW ~3100 Hz.
constexpr int32_t kBpB0 =  13653;  //  0.4167 in Q15
constexpr int32_t kBpB1 =  0;
constexpr int32_t kBpB2 = -13653;  // -0.4167 in Q15
constexpr int32_t kBpA1 = -12973;  // -0.3960 in Q15 (negated for direct-form)
constexpr int32_t kBpA2 =  5462;   //  0.1667 in Q15

inline int16_t sat16(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

inline int32_t isqrt32(int32_t x)
{
    if (x <= 0) return 0;
    int32_t r = 0;
    int32_t bit = 1 << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= r + bit) {
            x -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

} // namespace

void AudioProcessor::reset()
{
    dc_x_prev_ = 0;
    dc_y_prev_ = 0;
    preemph_prev_ = 0;
    deemph_prev_ = 0;
    gate_smooth_ = 0;
    gate_hold_ = 0;
    tx_mute_remaining_ = APP_AUDIO_DSP_TX_MUTE_FRAMES;
    agc_gain_q15_ = kQ15One;
    ns_floor_q15_ = 0;
    bp_x1_ = 0;
    bp_x2_ = 0;
    bp_y1_ = 0;
    bp_y2_ = 0;
}

void AudioProcessor::process_tx_frame(int16_t *pcm, size_t samples)
{
#if !APP_AUDIO_DSP_ENABLE
    (void)pcm; (void)samples;
    return;
#else
    if (tx_mute_remaining_ > 0) {
        --tx_mute_remaining_;
        std::memset(pcm, 0, samples * sizeof(int16_t));
        return;
    }
    apply_dc_block(pcm, samples);
    apply_noise_suppress(pcm, samples);
    apply_noise_gate(pcm, samples);
    apply_voice_bandpass(pcm, samples);
    apply_agc(pcm, samples);
    apply_preemphasis(pcm, samples);
#endif
}

void AudioProcessor::process_rx_frame(int16_t *pcm, size_t samples)
{
#if !APP_AUDIO_DSP_ENABLE
    (void)pcm; (void)samples;
    return;
#else
    apply_deemphasis(pcm, samples);
    apply_soft_limiter(pcm, samples);
#endif
}

void AudioProcessor::apply_dc_block(int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples; ++i) {
        int32_t x = pcm[i];
        int32_t y = x - dc_x_prev_ + ((kDcAlpha * dc_y_prev_) >> 15);
        dc_x_prev_ = x;
        dc_y_prev_ = y;
        pcm[i] = sat16(y);
    }
}

void AudioProcessor::apply_preemphasis(int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples; ++i) {
        int32_t x = pcm[i];
        int32_t y = x - ((kPreemphAlpha * preemph_prev_) >> 15);
        preemph_prev_ = static_cast<int16_t>(x);
        pcm[i] = sat16(y);
    }
}

void AudioProcessor::apply_noise_gate(int16_t *pcm, size_t samples)
{
    int32_t sum_sq = 0;
    for (size_t i = 0; i < samples; ++i) {
        int32_t s = pcm[i];
        sum_sq += (s * s) >> 8;
    }
    int32_t rms_approx = 0;
    if (samples > 0) {
        rms_approx = sum_sq / static_cast<int32_t>(samples);
    }

    bool above = rms_approx > (kGateThresh * kGateThresh >> 8);
    if (above) {
        if (gate_hold_ < APP_AUDIO_DSP_NOISE_GATE_ATTACK) {
            ++gate_hold_;
        }
    } else {
        if (gate_hold_ > 0) {
            --gate_hold_;
        }
    }

    bool gate_open = gate_hold_ >= APP_AUDIO_DSP_NOISE_GATE_ATTACK;
    int32_t target = gate_open ? kQ15One : 0;
    int32_t smooth_step = (target - gate_smooth_) >> APP_AUDIO_DSP_NOISE_GATE_RELEASE;
    gate_smooth_ += smooth_step;

    if (gate_smooth_ < kQ15One) {
        for (size_t i = 0; i < samples; ++i) {
            pcm[i] = sat16((static_cast<int32_t>(pcm[i]) * gate_smooth_) >> 15);
        }
    }
}

void AudioProcessor::apply_noise_suppress(int16_t *pcm, size_t samples)
{
    int32_t sum_sq = 0;
    for (size_t i = 0; i < samples; ++i) {
        int32_t s = pcm[i];
        sum_sq += (s * s) >> 10;
    }
    int32_t rms_q15 = 0;
    if (samples > 0) {
        int32_t mean_sq = sum_sq / static_cast<int32_t>(samples);
        rms_q15 = isqrt32(mean_sq << 10);
    }

    if (rms_q15 < ns_floor_q15_) {
        ns_floor_q15_ -= (ns_floor_q15_ - rms_q15) >> APP_AUDIO_DSP_NS_FLOOR_DECAY;
    } else {
        ns_floor_q15_ += (rms_q15 - ns_floor_q15_) >> APP_AUDIO_DSP_NS_FLOOR_ADAPT;
    }

    int32_t noise_est = ns_floor_q15_ << APP_AUDIO_DSP_NS_OVERSUBTRACT;
    int32_t snr_q15 = kQ15One;
    if (noise_est > 0 && rms_q15 > noise_est) {
        snr_q15 = ((rms_q15 - noise_est) << 15) / rms_q15;
    } else if (noise_est > 0) {
        snr_q15 = 0;
    }

    int32_t gain = snr_q15;
    if (gain < APP_AUDIO_DSP_NS_MIN_GAIN_Q15) {
        gain = APP_AUDIO_DSP_NS_MIN_GAIN_Q15;
    }

    for (size_t i = 0; i < samples; ++i) {
        pcm[i] = sat16((static_cast<int32_t>(pcm[i]) * gain) >> 15);
    }
}

void AudioProcessor::apply_voice_bandpass(int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples; ++i) {
        int32_t x = pcm[i];
        int32_t y = (kBpB0 * x + kBpB1 * bp_x1_ + kBpB2 * bp_x2_
                     - kBpA1 * bp_y1_ - kBpA2 * bp_y2_) >> 15;
        bp_x2_ = bp_x1_;
        bp_x1_ = x;
        bp_y2_ = bp_y1_;
        bp_y1_ = y;
        pcm[i] = sat16(y);
    }
}

void AudioProcessor::apply_agc(int16_t *pcm, size_t samples)
{
    int32_t peak = 0;
    for (size_t i = 0; i < samples; ++i) {
        int32_t s = pcm[i];
        int32_t a = s < 0 ? -s : s;
        if (a > peak) peak = a;
    }

    if (peak < 8) {
        return;
    }

    int32_t desired_gain = (kAgcTarget << 15) / peak;
    if (desired_gain > kAgcMaxGain) {
        desired_gain = kAgcMaxGain;
    }
    if (desired_gain < kQ15One >> 1) {
        desired_gain = kQ15One >> 1;
    }

    if (desired_gain > agc_gain_q15_) {
        agc_gain_q15_ += (desired_gain - agc_gain_q15_) >> APP_AUDIO_DSP_AGC_ATTACK_SHIFT;
    } else {
        agc_gain_q15_ -= (agc_gain_q15_ - desired_gain) >> APP_AUDIO_DSP_AGC_RELEASE_SHIFT;
    }

    for (size_t i = 0; i < samples; ++i) {
        pcm[i] = sat16((static_cast<int32_t>(pcm[i]) * agc_gain_q15_) >> 15);
    }
}

void AudioProcessor::apply_deemphasis(int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples; ++i) {
        int32_t x = pcm[i];
        int32_t y = x + ((kPreemphAlpha * deemph_prev_) >> 15);
        deemph_prev_ = sat16(y);
        pcm[i] = deemph_prev_;
    }
}

void AudioProcessor::apply_soft_limiter(int16_t *pcm, size_t samples)
{
    for (size_t i = 0; i < samples; ++i) {
        int32_t s = pcm[i];
        if (s > kLimiterThresh) {
            s = kLimiterThresh + ((s - kLimiterThresh) >> 2);
            pcm[i] = sat16(s);
        } else if (s < -kLimiterThresh) {
            s = -kLimiterThresh + ((s + kLimiterThresh) >> 2);
            pcm[i] = sat16(s);
        }
    }
}
