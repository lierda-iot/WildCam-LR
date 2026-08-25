#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_config.h"
#include "bsp.h"
#include "LiotLr2021.h"
#include "opus_codec.hpp"
#include "audio_processor.hpp"
#include "image_transfer.hpp"
#include "audio_ringbuf.hpp"
#include "opus_ringbuf.hpp"

enum class ImageCmdAckStatus : uint8_t {
    accepted = 0,
    busy = 1,
    cooldown = 2,
    duplicate = 3,
    rejected = 4,
};

enum class ImageRxError : uint8_t {
    crc_mismatch = 0,
    timeout,
    no_memory,
};

typedef ImageCmdAckStatus (*image_capture_cb_t)(uint16_t session_id);
typedef void (*image_request_rejected_cb_t)(ImageCmdAckStatus status);
typedef void (*image_rx_complete_cb_t)(ImageTransfer *xfer);
typedef void (*image_rx_progress_cb_t)(uint16_t session_id, uint16_t received,
                                       uint16_t total, int16_t rssi);
typedef void (*image_rx_error_cb_t)(ImageRxError error);
typedef void (*image_rx_eot_cb_t)(uint16_t missing_count, bool is_first_eot);
typedef void (*config_received_cb_t)(uint8_t key, uint32_t value);
typedef void (*frequency_committed_cb_t)(uint32_t frequency_hz);
typedef void (*frequency_change_result_cb_t)(bool success, uint32_t frequency_hz);
// Called on the gateway when a node's battery voltage arrives (via ImageStart
// or a periodic Vbat broadcast), so the app can push it to the UI.
typedef void (*vbat_received_cb_t)(uint16_t vbat_mv);
// Low power: called when the node enters (true) / leaves (false) CAD sleep
// standby, so the app can release/restore power-hungry peripherals (camera, I2S).
typedef void (*low_power_standby_cb_t)(bool entering);

class RadioPing {
public:
    esp_err_t init();
    esp_err_t init_gateway();
    esp_err_t start();
    esp_err_t start_gateway();
    void handle_button(bsp_btn_id_t id, bool pressed);
    void suspend();
    void resume();

    // Image transfer: B triggers A to capture. Returns false (and does nothing)
    // if a request/RX is already in progress — a new request cannot preempt an
    // active transfer (see the body for why restarting mid-transfer deadlocks).
    bool trigger_image_capture();
    // True while the gateway owns the radio for an image request or RX (or, on
    // the node, while an image TX is running). Callers use this to avoid firing
    // a second transfer request on top of an active one.
    bool image_busy() const { return image_tx_active_ || image_rx_pending_ || image_req_active_; }
    bool frequency_change_busy() const {
        return frequency_change_request_pending_ || frequency_change_active_ ||
               frequency_rollback_active_;
    }
    // Request image RX cancellation; the radio task performs the serialized teardown.
    void abort_image_rx() { image_rx_abort_req_ = true; }
    // Image transfer: A sends JPEG fragments to B.
    // Takes ownership of `jpeg` (must be a heap_caps allocation): the tx task
    // frees it when the transfer completes/aborts. Callers must not free it.
    void send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id);
    // Register callbacks
    void set_image_capture_cb(image_capture_cb_t cb) { image_capture_cb_ = cb; }
    void set_image_request_rejected_cb(image_request_rejected_cb_t cb) { image_request_rejected_cb_ = cb; }
    void set_image_rx_complete_cb(image_rx_complete_cb_t cb) { image_rx_complete_cb_ = cb; }
    void set_image_rx_progress_cb(image_rx_progress_cb_t cb) { image_rx_progress_cb_ = cb; }
    void set_image_rx_error_cb(image_rx_error_cb_t cb) { image_rx_error_cb_ = cb; }
    void set_vbat_received_cb(vbat_received_cb_t cb) { vbat_received_cb_ = cb; }
    void set_image_rx_eot_cb(image_rx_eot_cb_t cb) { image_rx_eot_cb_ = cb; }
    void set_config_received_cb(config_received_cb_t cb) { config_received_cb_ = cb; }
    void set_frequency_committed_cb(frequency_committed_cb_t cb) { frequency_committed_cb_ = cb; }
    void set_frequency_change_result_cb(frequency_change_result_cb_t cb) { frequency_change_result_cb_ = cb; }
    void set_low_power_standby_cb(low_power_standby_cb_t cb) { low_power_standby_cb_ = cb; }
    void set_inter_packet_us(uint32_t us) { image_tx_inter_packet_us_ = us; }

    bool send_config(uint8_t key, uint32_t value);
    bool request_frequency_change(uint32_t frequency_hz);
    bool set_initial_frequency(uint32_t frequency_hz);
    uint32_t current_frequency_hz() const { return current_frequency_hz_; }

    ImageTransfer &image_xfer() { return image_xfer_; }
    uint32_t last_transfer_ms() const { return image_rx_transfer_ms_; }
    bool image_tx_busy() const { return image_tx_active_; }
    void pause_audio_capture() { image_tx_active_ = true; }
    void resume_audio_capture() { image_tx_active_ = false; }
    void enable_opus_preenc(bool en) { opus_preenc_enabled_ = en; }
    void set_sound_trigger_level(uint32_t level) { sound_trigger_level_ = level; }
    void set_pir_enabled(bool en) { pir_enabled_ = en; }
    void IRAM_ATTR pir_trigger() { pir_triggered_ = true; }
    // PIR wake is armed outside the 15-second cooldown and cleared by the ISR.
    void IRAM_ATTR set_pir_armed(bool armed) { pir_armed_ = armed; }
    bool pir_armed() const { return pir_armed_; }

    // Release the PIR keep-awake guard when a triggered capture is dropped.
    void notify_capture_dropped() { pir_push_wake_ = false; }

    // Keep the low-power node awake while a capture and image push are active.
    // image_tx_task releases the guard; the poll loop provides an 8-second fallback.
    void notify_capture_starting();

    // Keep the node awake during synchronous audio playback; calls must be paired.
    void audio_playback_begin() { audio_playing_ = true; }
    void audio_playback_end() { audio_playing_ = false; }

    // Audio ring buffer for pre-capture retrospective recording
    size_t snapshot_audio(int16_t *out, size_t max_samples);
    size_t snapshot_opus(uint8_t *out, size_t max_bytes);

private:
    enum class Mode {
        idle,
        rx_pending,
        tx_pending,
        cad_pending,
    };

    static void task_trampoline(void *arg);
    static void tx_task_trampoline(void *arg);
    static void play_task_trampoline(void *arg);
    static void image_tx_task_trampoline(void *arg);
    static void irq_callback(void *context);

    void task();
    void tx_task();
    void play_task();
    void poll_once();
    void handle_irq(ral_irq_t irq);
    void schedule_rx();
    void schedule_tx();
    bool configure_flrc();
    bool apply_frequency(uint32_t frequency_hz);
    bool is_frequency_preset(uint32_t frequency_hz) const;
    bool change_frequency(uint32_t frequency_hz);
    bool build_voice_packet(uint16_t *tx_size);
    void capture_voice_packet();
    void handle_rx_packet();
    void dispatch_rx_packet(uint16_t len, int16_t rssi);
    void queue_voice_packet(uint16_t len, int16_t rssi);
    void log_rx(uint16_t seq, uint16_t len, int16_t rssi);
    void wait_for_jitter_buffer();
    void conceal_missing_frames(uint16_t seq);
    bool read_mono_frame(int16_t *mono, size_t samples);
    void play_mono_frame(const int16_t *mono, size_t samples);
    void set_playback_pa(bool on);
    void update_playback_timeout();

    // Image transfer methods
    void handle_image_cmd();
    void handle_image_cmd_ack();
    // Send one ImageCmd; the request round handles LoRa wakeup and FLRC setup.
    void send_image_cmd_once();
    // Begin a request round and fill the node's low-power wake window with ImageCmd retries.
    void start_image_req_round();
    // Called from the radio task loop: resend ImageCmd when the retry interval
    // elapses and the node hasn't acked yet. Runs in the SAME task as poll_once
    // so radio access stays serialized (no IRQ/mode races with an esp_timer).
    void check_image_req_retry();
    void stop_image_req_retry();
    void handle_image_start(uint16_t len);
    void handle_image_data(uint16_t len);
    void handle_image_eot();
    void handle_image_nack();
    void handle_image_done();
    void image_tx_task();
    bool send_single_packet(const uint8_t *data, uint16_t len);
    // Forward declaration: ImageTxRequest is defined further down (line ~215),
    // but the burst helpers below take it by const-ref so an incomplete type is
    // enough here.
    struct ImageTxRequest;
    // Build one ImageData fragment packet into `pkt` (14B header + frag + CRC16)
    // and return the total on-air length. `pkt` must hold APP_FLRC_MAX_PAYLOAD_BYTES.
    uint16_t build_image_fragment(uint8_t *pkt, const ImageTxRequest &req,
                                  uint16_t frag_index, uint16_t total_fragments);
    // Stream sequential or requested image fragments through the 1024-byte FLRC TX FIFO.
    void burst_send_fragments(const ImageTxRequest &req, uint16_t total_fragments,
                              const uint16_t *indices, uint16_t count);
    bool wait_for_tx_done(uint32_t timeout_ms);
    void check_image_rx_timeout();
    void check_image_rx_abort();
    bool send_config_ack(uint8_t key, uint32_t value, uint16_t transaction_id = 0);
    void handle_frequency_config(uint16_t transaction_id, uint32_t frequency_hz);
    void handle_frequency_confirm(uint16_t transaction_id, uint32_t frequency_hz);
    void check_frequency_rollback();
    bool configure_lora_cad();
    void enter_low_power_cad();
    // Light-sleep the ESP32 for up to `ms`, waking on the timer or (if PIR is
    // enabled) the PIR GPIO. Returns true if woken by PIR. Used during CAD
    // standby to save whole-chip power.
    bool low_power_sleep(uint32_t ms);
    void handle_cad_irq(ral_irq_t irq);

    // Battery voltage broadcast: send a small FLRC packet with current cached voltage.
    void send_vbat_broadcast();
    // Maintenance tick for low-power nodes: sample voltage every 60s, broadcast every 5min.
    void vbat_maintenance_tick();
    bool send_lora_wakeup();

    struct VoicePacket {
        uint16_t seq;
        uint16_t len;
        int16_t rssi;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    struct TxFrame {
        uint16_t seq;
        uint16_t len;
        uint8_t payload[APP_OPUS_MAX_PACKET_BYTES];
    };

    struct ImageTxRequest {
        const uint8_t *jpeg;
        size_t jpeg_len;
        uint16_t session_id;
    };

    static RadioPing *instance_;

    TaskHandle_t task_handle_ = nullptr;
    ralf_t radio_ = RALF_LR20XX_INSTANTIATE(nullptr);
    OpusCodec codec_;
    AudioProcessor audio_proc_;
    ImageTransfer image_xfer_;
    AudioRingBuf audio_ringbuf_;
    OpusRingBuf opus_ringbuf_;
    QueueHandle_t voice_queue_ = nullptr;
    QueueHandle_t tx_queue_ = nullptr;
    QueueHandle_t image_tx_queue_ = nullptr;
    Mode mode_ = Mode::idle;
    volatile bool ptt_active_ = false;
    volatile bool suspended_ = false;
    bool tx_burst_active_ = false;
    bool tx_flush_pending_ = false;
    volatile bool irq_pending_ = false;

    uint8_t tx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    uint8_t rx_buf_[APP_FLRC_MAX_PAYLOAD_BYTES] = {};
    int16_t tx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};
    int16_t rx_pcm_[APP_AUDIO_FRAME_SAMPLES] = {};

    uint16_t tx_seq_ = 0;
    uint16_t expected_rx_seq_ = 0;
    bool have_expected_rx_seq_ = false;
    uint32_t rx_packets_ = 0;
    uint32_t rx_lost_ = 0;
    uint32_t rx_crc_errors_ = 0;
    uint32_t rx_unknown_packets_ = 0;
    uint32_t rx_queue_drops_ = 0;
    uint32_t tx_queue_drops_ = 0;
    uint32_t last_rx_audio_ms_ = 0;
    uint16_t expected_play_seq_ = 0;
    bool have_expected_play_seq_ = false;
    bool playback_pa_on_ = false;
    bool playback_active_ = false;

    // Image transfer state
    image_capture_cb_t image_capture_cb_ = nullptr;
    image_request_rejected_cb_t image_request_rejected_cb_ = nullptr;
    image_rx_complete_cb_t image_rx_complete_cb_ = nullptr;
    image_rx_progress_cb_t image_rx_progress_cb_ = nullptr;
    image_rx_error_cb_t image_rx_error_cb_ = nullptr;
    vbat_received_cb_t vbat_received_cb_ = nullptr;
    image_rx_eot_cb_t image_rx_eot_cb_ = nullptr;
    config_received_cb_t config_received_cb_ = nullptr;
    frequency_committed_cb_t frequency_committed_cb_ = nullptr;
    frequency_change_result_cb_t frequency_change_result_cb_ = nullptr;
    low_power_standby_cb_t low_power_standby_cb_ = nullptr;
    uint16_t image_session_id_ = 1;
    volatile bool image_tx_active_ = false;
    bool opus_preenc_enabled_ = false;
    uint32_t image_tx_inter_packet_us_ = APP_IMAGE_TX_INTER_PACKET_US;
    uint32_t image_rx_last_frag_ms_ = 0;
    uint32_t image_rx_last_progress_ms_ = 0;
    uint32_t image_rx_expected_crc32_ = 0;
    bool image_rx_pending_ = false;
    volatile bool image_rx_abort_req_ = false;
    uint16_t image_rx_nack_sent_ = 0;
    uint16_t image_rx_eot_count_ = 0;
    int16_t image_rx_last_rssi_ = 0;
    uint16_t image_rx_done_session_ = 0;
    // ImageCmd retries keep one session ID and stop on ImageCmdAck or ImageStart.
    // The radio task owns all retry operations.
    bool image_req_active_ = false;
    uint16_t image_req_session_ = 0;
    uint32_t image_req_next_ms_ = 0;
    // End of the current low-power request round; unused in normal continuous retry mode.
    uint32_t image_req_round_end_ms_ = 0;
    uint32_t image_cmd_sent_ms_ = 0;
    uint32_t image_rx_start_ms_ = 0;
    uint32_t image_rx_transfer_ms_ = 0;
    uint32_t image_rx_done_ms_ = 0;

    // NACK receive state for TX side (A)
    volatile bool image_nack_received_ = false;
    volatile bool image_done_received_ = false;
    uint16_t nack_indices_[APP_IMAGE_NACK_MAX_INDICES] = {};
    uint16_t nack_count_ = 0;

    // Config ACK state
    volatile bool config_ack_received_ = false;
    uint8_t config_ack_key_ = 0;
    uint32_t config_ack_value_ = 0;
    uint16_t config_ack_transaction_id_ = 0;

    // Two-stage frequency change state.
    volatile bool frequency_change_active_ = false;
    volatile bool frequency_change_request_pending_ = false;
    uint32_t frequency_change_request_hz_ = APP_FLRC_FREQUENCY_HZ;
    bool frequency_rollback_active_ = false;
    uint32_t current_frequency_hz_ = APP_FLRC_FREQUENCY_HZ;
    uint32_t frequency_previous_hz_ = APP_FLRC_FREQUENCY_HZ;
    uint32_t frequency_pending_hz_ = APP_FLRC_FREQUENCY_HZ;
    uint32_t frequency_rollback_deadline_ms_ = 0;
    uint16_t frequency_transaction_id_ = 0;

    // Low power CAD state
    bool low_power_cad_active_ = false;
    bool is_gateway_ = false;
    uint32_t cad_wakeup_ms_ = 0;
    // CAD watchdog timestamp; a timeout recovers from a missed CAD_DONE interrupt.
    uint32_t cad_pending_ms_ = 0;
    // Keep a PIR-initiated transmitter awake until image_tx_task takes control.
    volatile bool pir_push_wake_ = false;
    uint32_t pir_push_wake_ms_ = 0;
    // Untimed keep-awake while a synchronous audio clip (post-capture voice
    // alarm) is playing. Blocks CAD light sleep so I2S output isn't halted and
    // the node doesn't re-sleep mid-clip. Set/cleared via audio_playback_*().
    volatile bool audio_playing_ = false;

    // Sound/PIR trigger state (shared cooldown)
    uint32_t sound_trigger_level_ = 0;
    bool pir_enabled_ = false;
    int64_t last_trigger_us_ = 0;
    uint16_t sound_trigger_session_id_ = 0xC000;
    volatile bool pir_triggered_ = false;
    volatile bool pir_armed_ = false;

    // Delay sound-trigger dispatch until the Opus ring contains post-trigger frames.
    bool sound_trigger_pending_ = false;
    int64_t sound_trigger_fire_us_ = 0;
    uint16_t sound_trigger_pending_session_ = 0;

    // Battery voltage maintenance (low-power node only): last sample timestamp
    // and last broadcast timestamp. Non-low-power nodes use the bsp_vbat task.
    uint32_t vbat_last_sample_ms_ = 0;
    uint32_t vbat_last_broadcast_ms_ = 0;
};
