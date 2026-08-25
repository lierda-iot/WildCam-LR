#include "opus_codec.hpp"

#include "opus.h"

#include "app_config.h"
#include "esp_log.h"

namespace {
constexpr const char *TAG = "opus_codec";
}

OpusCodec::~OpusCodec()
{
    if (encoder_ != nullptr) {
        opus_encoder_destroy(encoder_);
        encoder_ = nullptr;
    }
    if (decoder_ != nullptr) {
        opus_decoder_destroy(decoder_);
        decoder_ = nullptr;
    }
}

esp_err_t OpusCodec::init()
{
    int err = OPUS_OK;
    encoder_ = opus_encoder_create(APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_CHANNELS,
                                   APP_OPUS_APPLICATION, &err);
    if (err != OPUS_OK || encoder_ == nullptr) {
        ESP_LOGE(TAG, "opus_encoder_create failed: %d", err);
        return ESP_FAIL;
    }

    decoder_ = opus_decoder_create(APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_CHANNELS, &err);
    if (err != OPUS_OK || decoder_ == nullptr) {
        ESP_LOGE(TAG, "opus_decoder_create failed: %d", err);
        return ESP_FAIL;
    }

    opus_encoder_ctl(encoder_, OPUS_SET_BITRATE(APP_OPUS_BITRATE_BPS));
    opus_encoder_ctl(encoder_, OPUS_SET_VBR(APP_OPUS_USE_VBR));
    opus_encoder_ctl(encoder_, OPUS_SET_COMPLEXITY(APP_OPUS_COMPLEXITY));
    opus_encoder_ctl(encoder_, OPUS_SET_DTX(APP_OPUS_USE_DTX));
    opus_encoder_ctl(encoder_, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

    ESP_LOGI(TAG, "Opus ready: %u Hz mono %u ms %d bps complexity=%d",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS,
             APP_OPUS_BITRATE_BPS, APP_OPUS_COMPLEXITY);
    return ESP_OK;
}

esp_err_t OpusCodec::init_decoder_only()
{
    int err = OPUS_OK;
    decoder_ = opus_decoder_create(APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_CHANNELS, &err);
    if (err != OPUS_OK || decoder_ == nullptr) {
        ESP_LOGE(TAG, "opus_decoder_create failed: %d", err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Opus decoder-only: %u Hz mono %u ms",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS);
    return ESP_OK;
}

int OpusCodec::encode(const int16_t *pcm, int frame_samples, uint8_t *out, size_t out_capacity)
{
    if (!can_encode()) return OPUS_INVALID_STATE;
    return opus_encode(encoder_, pcm, frame_samples, out, static_cast<opus_int32>(out_capacity));
}

int OpusCodec::decode(const uint8_t *packet, size_t packet_len, int16_t *pcm, int frame_samples)
{
    if (!ready()) return OPUS_INVALID_STATE;
    return opus_decode(decoder_, packet, static_cast<opus_int32>(packet_len), pcm, frame_samples, 0);
}

int OpusCodec::decode_lost(int16_t *pcm, int frame_samples)
{
    if (!ready()) return OPUS_INVALID_STATE;
    return opus_decode(decoder_, nullptr, 0, pcm, frame_samples, 0);
}

void OpusCodec::reset_encoder()
{
    if (encoder_ != nullptr) {
        opus_encoder_ctl(encoder_, OPUS_RESET_STATE);
    }
}

void OpusCodec::reset_decoder()
{
    if (decoder_ != nullptr) {
        opus_decoder_ctl(decoder_, OPUS_RESET_STATE);
    }
}
