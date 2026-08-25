#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "bsp.h"

class AudioDiagnostics {
public:
    esp_err_t init();
    void handle_button(bsp_btn_id_t id, bool pressed);
    void play_startup_chime();

private:
    enum class State {
        idle,
        recording,
        playing,
    };

    static void task_trampoline(void *arg);

    void task();
    void apply_state();
    void start_record(const char *source);
    void stop_record(const char *source);
    void set_volume_delta(int delta);

    void reset_record_stats();
    size_t record_mono_chunk(size_t room);
    void play_mono_buffer();
    void log_record_stats();
    void drain_rx(int frames);

    void pa_enable_for_cue();
    void play_tone(int freq_hz, int duration_ms);
    void play_silence(int duration_ms);
    void play_soft_tone(int freq_hz, int duration_ms, int amp);
    void beep_beep();

    SemaphoreHandle_t audio_sem_ = nullptr;
    uint8_t *rec_buf_ = nullptr;
    size_t rec_cap_ = 0;
    size_t rec_len_ = 0;
    volatile uint8_t volume_ = 70;
    volatile State state_ = State::idle;

    int16_t rec_left_min_ = 0;
    int16_t rec_left_max_ = 0;
    int16_t rec_right_min_ = 0;
    int16_t rec_right_max_ = 0;
    uint32_t rec_left_sum_abs_ = 0;
    uint32_t rec_right_sum_abs_ = 0;
    size_t rec_frame_count_ = 0;
};
