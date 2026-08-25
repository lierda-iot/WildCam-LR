#include "audio_diagnostics.hpp"

#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "app_config.h"

namespace {
constexpr const char *TAG = "audio_diag";
constexpr int REC_SECONDS = 3;
constexpr size_t REC_BUF_BYTES = APP_AUDIO_SAMPLE_RATE_HZ * 2 * REC_SECONDS;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}
} // namespace

esp_err_t AudioDiagnostics::init()
{
    rec_cap_ = REC_BUF_BYTES;
    rec_buf_ = static_cast<uint8_t *>(heap_caps_malloc(rec_cap_, MALLOC_CAP_SPIRAM));
    if (rec_buf_ == nullptr) {
        rec_buf_ = static_cast<uint8_t *>(heap_caps_malloc(rec_cap_, MALLOC_CAP_8BIT));
    }
    if (rec_buf_ == nullptr) {
        ESP_LOGE(TAG, "record buffer alloc %u failed", static_cast<unsigned>(rec_cap_));
        return ESP_ERR_NO_MEM;
    }

    audio_sem_ = xSemaphoreCreateBinary();
    if (audio_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    apply_state();
    BaseType_t ok = xTaskCreate(task_trampoline, "audio", 4096, this, 6, nullptr);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "record buffer %u bytes @%p", static_cast<unsigned>(rec_cap_), rec_buf_);
    return ESP_OK;
}

void AudioDiagnostics::handle_button(bsp_btn_id_t id, bool pressed)
{
    switch (id) {
    case BSP_BTN_USER1:
        if (pressed) {
            start_record("user1");
        } else {
            stop_record("user1");
        }
        break;

    default:
        break;
    }
}

void AudioDiagnostics::play_startup_chime()
{
#if APP_STARTUP_CHIME_ENABLE
    ESP_LOGI(TAG, "startup chime: %d/%d Hz, %d ms, amp=%d",
             APP_STARTUP_CHIME_FREQ1_HZ, APP_STARTUP_CHIME_FREQ2_HZ,
             APP_STARTUP_CHIME_TONE_MS, APP_STARTUP_CHIME_AMP);
    pa_enable_for_cue();
    play_soft_tone(APP_STARTUP_CHIME_FREQ1_HZ, APP_STARTUP_CHIME_TONE_MS,
                   APP_STARTUP_CHIME_AMP);
    play_silence(APP_STARTUP_CHIME_GAP_MS);
    play_soft_tone(APP_STARTUP_CHIME_FREQ2_HZ, APP_STARTUP_CHIME_TONE_MS,
                   APP_STARTUP_CHIME_AMP);
    play_silence(100);
    bsp_audio_pa_enable(false);
    ESP_LOGI(TAG, "startup chime done");
#else
    ESP_LOGI(TAG, "startup chime disabled");
#endif
}

void AudioDiagnostics::task_trampoline(void *arg)
{
    static_cast<AudioDiagnostics *>(arg)->task();
}

void AudioDiagnostics::task()
{
    while (true) {
        xSemaphoreTake(audio_sem_, portMAX_DELAY);

        if (state_ == State::recording) {
            ESP_LOGI(TAG, "record cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            ESP_LOGI(TAG, "record cue done: PA off, start capture");
            bsp_audio_pa_enable(false);
            drain_rx(APP_AUDIO_SAMPLE_RATE_HZ / 20);

            while (state_ == State::recording) {
                size_t room = rec_cap_ - rec_len_;
                if (room == 0) {
                    ESP_LOGW(TAG, "record buffer full");
                    state_ = State::playing;
                    apply_state();
                    break;
                }
                size_t got = record_mono_chunk(room);
                if (got == 0) break;
                rec_len_ += got;
            }
            log_record_stats();
        }

        if (state_ == State::playing) {
            ESP_LOGI(TAG, "playback cue: PA on, then beep");
            pa_enable_for_cue();
            beep_beep();
            play_mono_buffer();
            play_silence(80);
            ESP_LOGI(TAG, "playback done");
            state_ = State::idle;
            apply_state();
        }
    }
}

void AudioDiagnostics::apply_state()
{
    bsp_audio_set_volume(volume_);
    bsp_audio_pa_enable(state_ == State::playing);

    bool rec = state_ == State::recording;
    bool play = state_ == State::playing;
    // Green-on-idle removed for low-power testing: idle is the steady state and
    // the always-lit green LED (D10) is a constant ~mA drain. Keep green off.
    bsp_led_set(false, false, rec || play);
}

void AudioDiagnostics::start_record(const char *source)
{
    if (state_ != State::idle) return;
    rec_len_ = 0;
    reset_record_stats();
    state_ = State::recording;
    ESP_LOGI(TAG, "%s down -> local record diagnostic", source);
    apply_state();
    xSemaphoreGive(audio_sem_);
}

void AudioDiagnostics::stop_record(const char *source)
{
    if (state_ != State::recording) return;
    state_ = State::playing;
    ESP_LOGI(TAG, "%s up -> local playback diagnostic (%u bytes)",
             source, static_cast<unsigned>(rec_len_));
    apply_state();
    xSemaphoreGive(audio_sem_);
}

void AudioDiagnostics::set_volume_delta(int delta)
{
    int next = static_cast<int>(volume_) + delta;
    if (next < 0) next = 0;
    if (next > 100) next = 100;
    volume_ = static_cast<uint8_t>(next);
    ESP_LOGI(TAG, "volume -> %u%%", volume_);
    apply_state();
}

void AudioDiagnostics::reset_record_stats()
{
    rec_left_min_ = 0;
    rec_left_max_ = 0;
    rec_right_min_ = 0;
    rec_right_max_ = 0;
    rec_left_sum_abs_ = 0;
    rec_right_sum_abs_ = 0;
    rec_frame_count_ = 0;
}

size_t AudioDiagnostics::record_mono_chunk(size_t room)
{
    int16_t stereo[APP_AUDIO_IO_CHUNK_BYTES / sizeof(int16_t)];
    size_t got = 0;
    esp_err_t err = bsp_audio_read(stereo, sizeof(stereo), &got);
    if (err != ESP_OK || got < 4) {
        ESP_LOGW(TAG, "record read failed: %s, got=%u",
                 esp_err_to_name(err), static_cast<unsigned>(got));
        return 0;
    }

    auto *dst = reinterpret_cast<int16_t *>(rec_buf_ + rec_len_);
    size_t frames = got / 4;
    size_t max_frames = room / sizeof(int16_t);
    if (frames > max_frames) frames = max_frames;

    for (size_t i = 0; i < frames; i++) {
        int16_t left = stereo[2 * i];
        int16_t right = stereo[2 * i + 1];
        if (left < rec_left_min_) rec_left_min_ = left;
        if (left > rec_left_max_) rec_left_max_ = left;
        if (right < rec_right_min_) rec_right_min_ = right;
        if (right > rec_right_max_) rec_right_max_ = right;
        rec_left_sum_abs_ += static_cast<uint32_t>(abs16(left));
        rec_right_sum_abs_ += static_cast<uint32_t>(abs16(right));
        dst[i] = (abs16(left) >= abs16(right)) ? left : right;
    }
    rec_frame_count_ += frames;

    return frames * sizeof(int16_t);
}

void AudioDiagnostics::play_mono_buffer()
{
    int16_t stereo[256 * 2];
    const auto *src = reinterpret_cast<const int16_t *>(rec_buf_);
    size_t samples = rec_len_ / sizeof(int16_t);
    size_t pos = 0;
    size_t total_written = 0;

    ESP_LOGI(TAG, "play mono start: samples=%u bytes=%u",
             static_cast<unsigned>(samples), static_cast<unsigned>(rec_len_));

    while (pos < samples) {
        size_t batch = samples - pos;
        if (batch > 256) batch = 256;

        for (size_t i = 0; i < batch; i++) {
            int16_t s = src[pos + i];
            stereo[2 * i] = s;
            stereo[2 * i + 1] = s;
        }

        size_t sent = 0;
        esp_err_t err = bsp_audio_write(stereo, batch * 4, &sent);
        if (err != ESP_OK || sent == 0) {
            ESP_LOGW(TAG, "playback write failed: %s, pos=%u",
                     esp_err_to_name(err), static_cast<unsigned>(pos));
            break;
        }
        total_written += sent;
        pos += sent / 4;
    }
    ESP_LOGI(TAG, "play mono done: wrote=%u bytes", static_cast<unsigned>(total_written));
}

void AudioDiagnostics::log_record_stats()
{
    const auto *samples = reinterpret_cast<const int16_t *>(rec_buf_);
    size_t count = rec_len_ / sizeof(int16_t);
    int16_t min = 0;
    int16_t max = 0;
    uint32_t sum_abs = 0;

    for (size_t i = 0; i < count; i++) {
        int16_t s = samples[i];
        if (s < min) min = s;
        if (s > max) max = s;
        sum_abs += static_cast<uint32_t>(abs16(s));
    }

    uint32_t avg_abs = count ? sum_abs / count : 0;
    ESP_LOGI(TAG, "record stats: samples=%u min=%d max=%d avg_abs=%u",
             static_cast<unsigned>(count), min, max, static_cast<unsigned>(avg_abs));
    ESP_LOGI(TAG, "record raw channels: frames=%u L[min=%d max=%d avg_abs=%u] R[min=%d max=%d avg_abs=%u]",
             static_cast<unsigned>(rec_frame_count_),
             rec_left_min_, rec_left_max_,
             static_cast<unsigned>(rec_frame_count_ ? rec_left_sum_abs_ / rec_frame_count_ : 0),
             rec_right_min_, rec_right_max_,
             static_cast<unsigned>(rec_frame_count_ ? rec_right_sum_abs_ / rec_frame_count_ : 0));
}

void AudioDiagnostics::drain_rx(int frames)
{
    uint8_t scratch[APP_AUDIO_IO_CHUNK_BYTES];
    int left = frames * 4;
    while (left > 0) {
        size_t got = 0;
        int want = left < static_cast<int>(sizeof(scratch)) ? left : static_cast<int>(sizeof(scratch));
        if (bsp_audio_read(scratch, want, &got) != ESP_OK || got == 0) break;
        left -= static_cast<int>(got);
    }
}

void AudioDiagnostics::pa_enable_for_cue()
{
    esp_err_t err = bsp_audio_pa_enable(true);
    ESP_LOGI(TAG, "PA enable for cue -> %s", esp_err_to_name(err));
    vTaskDelay(pdMS_TO_TICKS(APP_PA_SETTLE_MS));
}

void AudioDiagnostics::play_tone(int freq_hz, int duration_ms)
{
    const int period = APP_AUDIO_SAMPLE_RATE_HZ / freq_hz;
    const int half = period / 2;
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int16_t buf[128 * 2];
    int frame = 0;

    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        for (int i = 0; i < batch; i++) {
            int pos = (frame + i) % period;
            int16_t s = (pos < half) ? APP_BEEP_AMP : -APP_BEEP_AMP;
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }
        size_t w = 0;
        esp_err_t err = bsp_audio_write(buf, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "tone write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::play_silence(int duration_ms)
{
    static const int16_t zeros[128 * 2] = { 0 };
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int frame = 0;
    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;
        size_t w = 0;
        esp_err_t err = bsp_audio_write(zeros, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "silence write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::play_soft_tone(int freq_hz, int duration_ms, int amp)
{
    const int period = APP_AUDIO_SAMPLE_RATE_HZ / freq_hz;
    const int total = APP_AUDIO_SAMPLE_RATE_HZ * duration_ms / 1000;
    int16_t buf[128 * 2];
    int frame = 0;

    while (frame < total) {
        int batch = total - frame;
        if (batch > 128) batch = 128;

        for (int i = 0; i < batch; i++) {
            int t = frame + i;
            int pos = t % period;
            int quarter = period / 4;
            if (quarter <= 0) quarter = 1;

            int tri;
            if (pos < quarter) {
                tri = (amp * pos) / quarter;
            } else if (pos < 3 * quarter) {
                tri = amp - (2 * amp * (pos - quarter)) / (2 * quarter);
            } else {
                tri = -amp + (amp * (pos - 3 * quarter)) / quarter;
            }

            int fade = t;
            int tail = total - 1 - t;
            if (tail < fade) fade = tail;
            const int fade_len = APP_AUDIO_SAMPLE_RATE_HZ / 100;
            if (fade > fade_len) fade = fade_len;
            if (fade < 0) fade = 0;

            int16_t s = static_cast<int16_t>((tri * fade) / fade_len);
            buf[2 * i] = s;
            buf[2 * i + 1] = s;
        }

        size_t w = 0;
        esp_err_t err = bsp_audio_write(buf, batch * 4, &w);
        if (err != ESP_OK || w != batch * 4) {
            ESP_LOGW(TAG, "soft tone write failed: %s, wrote %u/%u",
                     esp_err_to_name(err), static_cast<unsigned>(w), static_cast<unsigned>(batch * 4));
            break;
        }
        frame += batch;
    }
}

void AudioDiagnostics::beep_beep()
{
    play_tone(APP_BEEP_FREQ_HZ, APP_BEEP_ON_MS);
    play_silence(APP_BEEP_GAP_MS);
    play_tone(APP_BEEP_FREQ_HZ, APP_BEEP_ON_MS);
    play_silence(APP_BEEP_TAIL_MS);
}
