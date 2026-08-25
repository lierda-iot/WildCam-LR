#include "radio_ping.hpp"

#include <cstring>
#include <cmath>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "bsp.h"

#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_system.h"

extern volatile bool g_low_power_enabled;

namespace {
constexpr const char *TAG = "radio_ping";
constexpr uint8_t kSyncWord[4] = {
    APP_FLRC_SYNC_WORD_0,
    APP_FLRC_SYNC_WORD_1,
    APP_FLRC_SYNC_WORD_2,
    APP_FLRC_SYNC_WORD_3,
};
constexpr uint8_t kMagic[4] = { 'L', 'R', 'P', '1' };
constexpr uint8_t kPacketTypePing = 1;
constexpr uint8_t kPacketTypeVoice = 2;
constexpr uint8_t kPacketTypeImageCmd = 3;
constexpr uint8_t kPacketTypeImageData = 4;
constexpr uint8_t kPacketTypeImageNack = 5;
constexpr uint8_t kPacketTypeImageDone = 6;
constexpr uint8_t kPacketTypeImageEOT = 7;
constexpr uint8_t kPacketTypeImageStart = 8;
constexpr uint8_t kPacketTypeConfig = 9;
constexpr uint8_t kPacketTypeConfigAck = 10;
constexpr uint8_t kPacketTypeImageCmdAck = 11;
constexpr uint8_t kPacketTypeVbat = 12;
constexpr uint8_t kPacketTypeFrequencyConfirm = 13;
constexpr uint16_t kHeaderSize = 14;
constexpr uint32_t kFrequencyPresetsHz[APP_FLRC_FREQUENCY_PRESET_COUNT] =
    APP_FLRC_FREQUENCY_PRESETS_HZ;

/* Low-power node battery voltage maintenance cadence and broadcast interval. */
constexpr uint32_t kVbatLowPowerSampleIntervalMs = 60000;     /* 60 s */
constexpr uint32_t kVbatBroadcastIntervalMs = 300000;         /* 5 min */
constexpr uint32_t kConfigAckTimeoutMs = 500;

int32_t abs16(int16_t v)
{
    return v < 0 ? -static_cast<int32_t>(v) : v;
}

void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint16_t get_u16_le(const uint8_t *p)
{
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t get_u32_le(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

TickType_t ms_to_ticks_min_1(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

uint32_t crc32_ieee(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xEDB88320U : (crc >> 1);
        }
    }
    return ~crc;
}

} // namespace

RadioPing *RadioPing::instance_ = nullptr;

bool RadioPing::set_initial_frequency(uint32_t frequency_hz)
{
    if (!is_frequency_preset(frequency_hz)) return false;
    current_frequency_hz_ = frequency_hz;
    frequency_previous_hz_ = frequency_hz;
    frequency_pending_hz_ = frequency_hz;
    return true;
}

esp_err_t RadioPing::init()
{
    instance_ = this;

    esp_err_t err = codec_.init();
    if (err != ESP_OK) {
        return err;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    tx_queue_ = xQueueCreate(APP_VOICE_TX_QUEUE_LEN, sizeof(TxFrame));
    if (tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }
    image_tx_queue_ = xQueueCreate(1, sizeof(ImageTxRequest));
    if (image_tx_queue_ == nullptr) {
        ESP_LOGE(TAG, "image tx queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    if (!audio_ringbuf_.init(APP_AUDIO_RINGBUF_SAMPLES)) {
        ESP_LOGW(TAG, "audio ring buffer alloc failed (PSRAM?)");
    }

    size_t opus_ring_frames = APP_AUDIO_RINGBUF_SECONDS * 1000 / APP_AUDIO_FRAME_MS;
    if (!opus_ringbuf_.init(opus_ring_frames)) {
        ESP_LOGW(TAG, "opus ring buffer alloc failed (PSRAM?)");
    }

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled for camera isolation");
    return ESP_OK;
#endif

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_reset(&radio_.ral);
    if (status == RAL_STATUS_OK) status = ral_init(&radio_.ral);
    if (status == RAL_STATUS_OK) {
        status = ral_set_rx_tx_fallback_mode(&radio_.ral, RAL_FALLBACK_STDBY_XOSC);
    }
    if (status == RAL_STATUS_OK) status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK && !configure_flrc()) status = RAL_STATUS_ERROR;
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_irq_config_radio_irq(&RadioPing::irq_callback, this);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "direct RAL init failed: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 direct RAL initialized: FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             current_frequency_hz_, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#else
    ESP_LOGW(TAG, "LR2021 auto RX disabled for camera isolation");
#endif
    return ESP_OK;
}

esp_err_t RadioPing::init_gateway()
{
    instance_ = this;
    is_gateway_ = true;

    esp_err_t err = codec_.init_decoder_only();
    if (err != ESP_OK) {
        return err;
    }

    voice_queue_ = xQueueCreate(APP_VOICE_RX_QUEUE_LEN, sizeof(VoicePacket));
    if (voice_queue_ == nullptr) {
        ESP_LOGE(TAG, "voice queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

#if !APP_RADIO_HW_INIT_ENABLE
    ESP_LOGW(TAG, "LR2021 hardware init disabled");
    return ESP_OK;
#endif

    smtc_modem_hal_protect_api_call();
    ral_status_t status = ral_reset(&radio_.ral);
    if (status == RAL_STATUS_OK) status = ral_init(&radio_.ral);
    if (status == RAL_STATUS_OK) {
        status = ral_set_rx_tx_fallback_mode(&radio_.ral, RAL_FALLBACK_STDBY_XOSC);
    }
    if (status == RAL_STATUS_OK) status = ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    if (status == RAL_STATUS_OK && !configure_flrc()) status = RAL_STATUS_ERROR;
    if (status == RAL_STATUS_OK) {
        status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    }
    smtc_modem_hal_irq_config_radio_irq(&RadioPing::irq_callback, this);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        ESP_LOGE(TAG, "direct RAL init failed: %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LR2021 gateway mode (RX-only): FLRC rf=%lu Hz br=%lu bps bw=%lu Hz",
             current_frequency_hz_, APP_FLRC_BITRATE_BPS, APP_FLRC_BANDWIDTH_HZ);
#if APP_RADIO_AUTO_RX_ENABLE
    schedule_rx();
#endif
    return ESP_OK;
}

esp_err_t RadioPing::start_gateway()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
                                            APP_RADIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(play_task_trampoline, "voice_play",
                                 APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                                 APP_VOICE_PLAY_TASK_PRIORITY, nullptr,
                                 APP_VOICE_PLAY_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t RadioPing::start()
{
    BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "radio_ping",
                                            APP_RADIO_TASK_STACK_BYTES, this,
                                            APP_RADIO_TASK_PRIORITY, &task_handle_,
                                            APP_RADIO_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(tx_task_trampoline, "voice_tx",
                                 APP_VOICE_TX_TASK_STACK_BYTES, this,
                                 APP_VOICE_TX_TASK_PRIORITY, nullptr,
                                 APP_VOICE_TX_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(play_task_trampoline, "voice_play",
                                 APP_VOICE_PLAY_TASK_STACK_BYTES, this,
                                 APP_VOICE_PLAY_TASK_PRIORITY, nullptr,
                                 APP_VOICE_PLAY_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ok = xTaskCreatePinnedToCore(image_tx_task_trampoline, "img_tx",
                                 APP_IMAGE_TASK_STACK_BYTES, this,
                                 APP_IMAGE_TASK_PRIORITY, nullptr,
                                 APP_IMAGE_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void RadioPing::handle_button(bsp_btn_id_t id, bool pressed)
{
    if (id != APP_PTT_BUTTON) return;
    if (suspended_) return;

    ptt_active_ = pressed;
    ESP_LOGI(TAG, "PTT %s -> FLRC voice %s", pressed ? "down" : "up",
             pressed ? "TX" : "RX");

    if (pressed) {
        bool new_burst = !tx_burst_active_;
        tx_burst_active_ = true;
        tx_flush_pending_ = false;

        if (new_burst) {
            set_playback_pa(false);
            playback_active_ = false;
            have_expected_play_seq_ = false;
            codec_.reset_encoder();
            audio_proc_.reset();
            if (voice_queue_ != nullptr) {
                xQueueReset(voice_queue_);
            }
            if (tx_queue_ != nullptr) {
                xQueueReset(tx_queue_);
            }
        }

        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }
    } else {
        tx_flush_pending_ = tx_burst_active_;
        if (mode_ == Mode::idle) {
            schedule_tx();
        }
    }
}

void RadioPing::suspend()
{
    suspended_ = true;
    ptt_active_ = false;
    tx_burst_active_ = false;
    tx_flush_pending_ = false;
    irq_pending_ = false;

    if (tx_queue_ != nullptr) {
        xQueueReset(tx_queue_);
    }
    if (voice_queue_ != nullptr) {
        xQueueReset(voice_queue_);
    }

    set_playback_pa(false);
    playback_active_ = false;
    have_expected_play_seq_ = false;

    smtc_modem_hal_protect_api_call();
    (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    ESP_LOGI(TAG, "radio suspended");
}

void RadioPing::resume()
{
    smtc_modem_hal_protect_api_call();
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::idle;
    suspended_ = false;
    ESP_LOGI(TAG, "radio resumed");
}

void RadioPing::task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->task();
}

void RadioPing::tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->tx_task();
}

void RadioPing::play_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->play_task();
}

void RadioPing::task()
{
    while (true) {
        if (frequency_change_request_pending_) {
            uint32_t requested_hz = frequency_change_request_hz_;
            bool ok = change_frequency(requested_hz);
            frequency_change_request_pending_ = false;
            if (frequency_change_result_cb_) {
                frequency_change_result_cb_(ok, current_frequency_hz_);
            }
        }
        if (!suspended_) {
            poll_once();
            update_playback_timeout();
            check_image_rx_timeout();
            check_image_req_retry();
            check_image_rx_abort();
            check_frequency_rollback();
        }
        ulTaskNotifyTake(pdTRUE, ms_to_ticks_min_1(APP_RADIO_TASK_POLL_MS));
    }
}

bool RadioPing::request_frequency_change(uint32_t frequency_hz)
{
    if (!is_gateway_ || task_handle_ == nullptr ||
        frequency_change_request_pending_ || frequency_change_active_ ||
        image_busy() || !is_frequency_preset(frequency_hz) ||
        frequency_hz == current_frequency_hz_) {
        ESP_LOGE(TAG,
                 "frequency request rejected: gateway=%d task=%d pending=%d busy=%d current=%lu requested=%lu",
                 is_gateway_, task_handle_ != nullptr,
                 frequency_change_request_pending_,
                 frequency_change_active_ || image_busy(),
                 static_cast<unsigned long>(current_frequency_hz_),
                 static_cast<unsigned long>(frequency_hz));
        return false;
    }

    frequency_change_request_hz_ = frequency_hz;
    frequency_change_request_pending_ = true;
    xTaskNotifyGive(task_handle_);
    ESP_LOGW(TAG, "frequency request queued: %lu Hz",
             static_cast<unsigned long>(frequency_hz));
    return true;
}

void RadioPing::tx_task()
{
    while (true) {
        if (suspended_ || image_tx_active_) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        // Low-power nodes skip microphone processing during CAD sleep; PIR remains a wake source.
        if (g_low_power_enabled && !is_gateway_) {
            if (pir_triggered_) {
                pir_triggered_ = false;
                bool dispatched = false;
                if (pir_enabled_) {
                    int64_t now = esp_timer_get_time();
                    if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                        last_trigger_us_ = now;
                        ESP_LOGI(TAG, "PIR trigger! (low power)");
                        if (image_capture_cb_) {
                            image_capture_cb_(sound_trigger_session_id_++);
                            dispatched = true;
                        }
                    }
                }
                // Release the keep-awake guard if the PIR capture was not dispatched.
                if (!dispatched) {
                    pir_push_wake_ = false;
                }
            }
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        if (!read_mono_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES)) {
            vTaskDelay(ms_to_ticks_min_1(APP_AUDIO_FRAME_MS));
            continue;
        }

        audio_ringbuf_.write(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);

        if (sound_trigger_level_ > 0) {
            int64_t sum_sq = 0;
            for (size_t i = 0; i < APP_AUDIO_FRAME_SAMPLES; i++) {
                int32_t s = tx_pcm_[i];
                sum_sq += s * s;
            }
            uint16_t rms = (uint16_t)sqrtf((float)sum_sq / APP_AUDIO_FRAME_SAMPLES);
            uint16_t thresh = (sound_trigger_level_ == 1) ? APP_SOUND_TRIGGER_THRESH_LOW :
                              (sound_trigger_level_ == 2) ? APP_SOUND_TRIGGER_THRESH_MED :
                                                           APP_SOUND_TRIGGER_THRESH_HIGH;
            if (rms >= thresh) {
                int64_t now = esp_timer_get_time();
                // Arm a delayed trigger (don't dispatch yet). last_trigger_us_ is
                // stamped now so the cooldown covers the detection instant, and
                // sound_trigger_pending_ blocks re-arming while one is in flight.
                if (!sound_trigger_pending_ &&
                    (now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                    last_trigger_us_ = now;
                    sound_trigger_fire_us_ = now + (int64_t)APP_SOUND_TRIGGER_DELAY_MS * 1000LL;
                    sound_trigger_pending_session_ = sound_trigger_session_id_++;
                    sound_trigger_pending_ = true;
                    ESP_LOGI(TAG, "sound trigger detected! rms=%u thresh=%u, fire in %ums",
                             rms, thresh, (unsigned)APP_SOUND_TRIGGER_DELAY_MS);
                }
            }
        }

        if (pir_triggered_) {
            pir_triggered_ = false;
            if (pir_enabled_) {
                int64_t now = esp_timer_get_time();
                if ((now - last_trigger_us_) >= (int64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000LL) {
                    last_trigger_us_ = now;
                    ESP_LOGI(TAG, "PIR trigger!");
                    if (image_capture_cb_) {
                        image_capture_cb_(sound_trigger_session_id_++);
                    }
                }
            }
        }

        if (opus_preenc_enabled_) {
            uint8_t enc_buf[APP_OPUS_MAX_PACKET_BYTES];
            int enc_len = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                        enc_buf, APP_OPUS_MAX_PACKET_BYTES);
            if (enc_len > 0 && enc_len <= 255) {
                opus_ringbuf_.write(enc_buf, static_cast<uint8_t>(enc_len));
            }
        }

        // Dispatch delayed sound capture after encoding the post-trigger Opus frames.
        if (sound_trigger_pending_ && esp_timer_get_time() >= sound_trigger_fire_us_) {
            sound_trigger_pending_ = false;
            ESP_LOGI(TAG, "sound trigger firing (delayed %ums)",
                     (unsigned)APP_SOUND_TRIGGER_DELAY_MS);
            if (image_capture_cb_) {
                image_capture_cb_(sound_trigger_pending_session_);
            }
        }
    }
}

void RadioPing::play_task()
{
    VoicePacket packet;

    while (true) {
        if (xQueueReceive(voice_queue_, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (suspended_) {
            continue;
        }

        if (!playback_active_) {
            codec_.reset_decoder();
        }
        wait_for_jitter_buffer();
        conceal_missing_frames(packet.seq);

        int decoded = codec_.decode(packet.payload, packet.len, rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
        if (decoded <= 0) {
            ESP_LOGW(TAG, "Opus decode failed: %d", decoded);
            continue;
        }

        audio_proc_.process_rx_frame(rx_pcm_, static_cast<size_t>(decoded));
        play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
        last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
        playback_active_ = true;
    }
}

void RadioPing::poll_once()
{
    if (irq_pending_) {
        irq_pending_ = false;
        ral_irq_t irq = RAL_IRQ_NONE;
        smtc_modem_hal_protect_api_call();
        ral_status_t status = ral_get_and_clear_irq_status(&radio_.ral, &irq);
        smtc_modem_hal_unprotect_api_call();
        if (status == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
            handle_irq(irq);
        }
    }

    // Recover to standby if CAD_DONE is missing for 2 seconds.
    if (mode_ == Mode::cad_pending && cad_pending_ms_ != 0 &&
        (smtc_modem_hal_get_time_in_ms() - cad_pending_ms_) >= 2000) {
        ESP_LOGW(TAG, "CAD watchdog: no CAD_DONE in 2s, resetting to idle");
        smtc_modem_hal_protect_api_call();
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }

    if (mode_ == Mode::idle) {
        if (frequency_rollback_active_ && !is_gateway_) {
            schedule_rx();
        } else if (g_low_power_enabled && !is_gateway_) {
            // Only nodes enter CAD sleep; gateways remain in FLRC RX for unsolicited image pushes.
            if (audio_playing_) {
                // Stay awake during synchronous audio playback to keep I2S running.
            } else if (pir_push_wake_) {
                // Stay awake while a PIR image push starts; use an 8-second safety timeout.
                if (smtc_modem_hal_get_time_in_ms() - pir_push_wake_ms_ >= 8000) {
                    pir_push_wake_ = false;
                    enter_low_power_cad();
                }
            } else if (cad_wakeup_ms_ != 0 &&
                (smtc_modem_hal_get_time_in_ms() - cad_wakeup_ms_) < APP_LP_WAKE_WINDOW_MS) {
                // Window is refreshed on every RX packet (handle_rx_packet), so
                // this stays open as long as there is traffic; it closes only
                // after APP_LP_WAKE_WINDOW_MS of total silence.
                schedule_rx();
            } else {
                cad_wakeup_ms_ = 0;
                // Node is awake and idle here; do voltage sampling / broadcast
                // now (before dropping into CAD light sleep) so we never spin up
                // the chip just for VBAT. Broadcast goes out on this awake window.
                vbat_maintenance_tick();
                enter_low_power_cad();
            }
        } else if (tx_burst_active_) {
            schedule_tx();
            if (!tx_burst_active_ && !ptt_active_ && mode_ == Mode::idle) {
                schedule_rx();
            }
        } else if (!ptt_active_) {
            if (low_power_cad_active_) {
                low_power_cad_active_ = false;
                configure_flrc();
                ESP_LOGI(TAG, "low power off, back to FLRC RX");
            }
            // Non-low-power node: periodic 5-min voltage broadcast. Guarded
            // internally by timestamp so this is cheap to call every idle pass.
            vbat_maintenance_tick();
            schedule_rx();
        }
    }
    taskYIELD();
}

void RadioPing::irq_callback(void *context)
{
    auto *self = static_cast<RadioPing *>(context);
    if (self != nullptr) {
        self->irq_pending_ = true;
        if (self->task_handle_ != nullptr) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(self->task_handle_, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void RadioPing::handle_irq(ral_irq_t irq)
{
    Mode completed_mode = mode_;

    if (completed_mode == Mode::rx_pending) {
        if ((irq & RAL_IRQ_RX_DONE) != 0) {
            // Do not rearm continuous RX during an image burst because set_rx clears queued FIFO data.
            // The ready ACK aligns the FIFO once, and hardware remains in continuous RX.
            bool image_stream = image_rx_pending_;
            mode_ = Mode::idle;
            handle_rx_packet();
            if (image_stream) {
                // Control packets in the stream (EOT/NACK/Start) run their own
                // schedule_rx and leave mode_ = rx_pending; only a pure data
                // fragment leaves it idle. Either way, stay in continuous RX.
                if (mode_ == Mode::idle) {
                    mode_ = Mode::rx_pending;
                }
            } else if (mode_ == Mode::idle && !ptt_active_ && !tx_burst_active_) {
                schedule_rx();
            }
        } else if ((irq & RAL_IRQ_RX_CRC_ERROR) != 0) {
            mode_ = Mode::idle;
            rx_crc_errors_++;
            if ((rx_crc_errors_ % 10) == 1) {
                ESP_LOGW(TAG, "RX CRC errors=%lu", static_cast<unsigned long>(rx_crc_errors_));
            }
            // A CRC failure leaves an unknown FIFO boundary; clear and rearm RX.
            // The EOT/NACK cycle recovers discarded fragments.
            schedule_rx();
        } else if ((irq & RAL_IRQ_RX_HDR_ERROR) != 0) {
            mode_ = Mode::idle;
            ESP_LOGW(TAG, "RX header error");
            schedule_rx();
        } else if ((irq & RAL_IRQ_RX_TIMEOUT) != 0) {
            mode_ = Mode::idle;
        } else {
            // Non-terminal IRQ (e.g. FIFO_LEVEL, PREAMBLE_DETECTED)
            // Do NOT reset mode or re-arm — packet still being received
        }
    } else if (completed_mode == Mode::tx_pending) {
        mode_ = Mode::idle;
        if ((irq & RAL_IRQ_TX_DONE) == 0) {
            ESP_LOGW(TAG, "TX irq=0x%08lx", static_cast<unsigned long>(irq));
        }
        if (APP_FLRC_VOICE_TX_GAP_MS > 0) {
            vTaskDelay(ms_to_ticks_min_1(APP_FLRC_VOICE_TX_GAP_MS));
        }
    } else if (completed_mode == Mode::cad_pending) {
        handle_cad_irq(irq);
    }
}

void RadioPing::schedule_rx()
{
    if (mode_ != Mode::idle) return;

    // Gateways use continuous RX to avoid gaps before unsolicited node transmissions.
    // Nodes use timed RX so they can return to CAD sleep.
    uint32_t rx_timeout = APP_FLRC_RX_TIMEOUT_MS;
    if (image_rx_pending_ || is_gateway_) {
        rx_timeout = RAL_RX_TIMEOUT_CONTINUOUS_MODE;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(false);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral,
                                                 RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT |
                                                 RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR);
    if (status == RAL_STATUS_OK) status = ral_set_rx(&radio_.ral, rx_timeout);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::rx_pending;
    } else {
        ESP_LOGW(TAG, "schedule RX failed: %d", status);
    }
}

void RadioPing::schedule_tx()
{
    if (mode_ != Mode::idle) return;
    if (tx_queue_ == nullptr) return;

    UBaseType_t queued = uxQueueMessagesWaiting(tx_queue_);
    if (queued == 0) {
        if (!ptt_active_) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }
    if (ptt_active_ && queued < APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        return;
    }

    uint16_t tx_size = 0;
    if (!build_voice_packet(&tx_size)) {
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
            tx_burst_active_ = false;
        }
        return;
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, tx_buf_, tx_size);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status == RAL_STATUS_OK) {
        mode_ = Mode::tx_pending;
        if (!ptt_active_ && uxQueueMessagesWaiting(tx_queue_) == 0) {
            tx_flush_pending_ = false;
        }
    } else {
        ESP_LOGW(TAG, "schedule TX failed: %d", status);
    }
}

bool RadioPing::configure_flrc()
{
    ralf_params_flrc_t params = {};
    params.rf_freq_in_hz = current_frequency_hz_;
    params.output_pwr_in_dbm = APP_FLRC_TX_POWER_DBM;
    params.mod_params.raw_bit_rate = APP_FLRC_RAW_BIT_RATE;
    params.mod_params.cr = APP_FLRC_CODING_RATE;
    params.mod_params.pulse_shape = APP_FLRC_PULSE_SHAPE;
    params.pkt_params.preamble_len = APP_FLRC_PREAMBLE_LEN;
    params.pkt_params.sync_word_len = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    params.pkt_params.tx_syncword = RAL_FLRC_TX_SYNCWORD_1;
    params.pkt_params.match_sync_word = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    params.pkt_params.pld_is_fix = false;
    params.pkt_params.pld_len_in_bytes = APP_FLRC_MAX_PAYLOAD_BYTES;
    params.pkt_params.crc_type = RAL_FLRC_CRC_2_BYTES;
    // Upgraded driver takes three RX-match sync-word pointers; we only use
    // syncword slot 1 (tx_syncword/match_sync_word above), so point [0] at our
    // sync word and leave the unused slots null.
    params.sync_word[0] = kSyncWord;
    params.sync_word[1] = nullptr;
    params.sync_word[2] = nullptr;
    // is_tx=true makes ralf_setup_flrc set TX power/freq AND program sync-word
    // slot 1 (via tx_syncword). RX matching uses RAL_FLRC_RX_MATCH_SYNCWORD_1,
    // which checks that same slot 1, so both TX and RX work from this one setup.
    params.is_tx = true;
    params.crc_seed = 0xFFFFFFFFUL;
    params.crc_polynomial = 0x04C11DB7UL;
    if (ralf_setup_flrc(&radio_, &params) != RAL_STATUS_OK) {
        return false;
    }

    // Allocate 1024-byte TX and RX FIFOs for two 511-byte burst fragments.
    const void *ctx = radio_.ral.context;
    if (lr20xx_radio_fifo_configure_1024_byte_tx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 TX FIFO failed");
        return false;
    }
    if (lr20xx_radio_fifo_configure_1024_byte_rx_fifo(ctx) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "configure_flrc: 1024 RX FIFO failed");
        return false;
    }
    return true;
}

bool RadioPing::is_frequency_preset(uint32_t frequency_hz) const
{
    for (uint32_t preset : kFrequencyPresetsHz) {
        if (preset == frequency_hz) return true;
    }
    return false;
}

bool RadioPing::apply_frequency(uint32_t frequency_hz)
{
    if (!is_frequency_preset(frequency_hz)) {
        ESP_LOGW(TAG, "reject unsupported frequency: %lu Hz",
                 static_cast<unsigned long>(frequency_hz));
        return false;
    }

    smtc_modem_hal_protect_api_call();
    if (mode_ != Mode::idle) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    current_frequency_hz_ = frequency_hz;
    bool ok = configure_flrc();
    smtc_modem_hal_unprotect_api_call();

    if (!ok) {
        ESP_LOGE(TAG, "frequency apply failed: %lu Hz",
                 static_cast<unsigned long>(frequency_hz));
        return false;
    }
    ESP_LOGI(TAG, "frequency applied: %lu Hz",
             static_cast<unsigned long>(frequency_hz));
    return true;
}

bool RadioPing::build_voice_packet(uint16_t *tx_size)
{
    TxFrame frame;
    if (tx_queue_ == nullptr || xQueueReceive(tx_queue_, &frame, 0) != pdTRUE) {
        return false;
    }

    std::memcpy(tx_buf_, kMagic, sizeof(kMagic));
    tx_buf_[4] = kPacketTypeVoice;
    tx_buf_[5] = 2;
    put_u16_le(&tx_buf_[6], frame.seq);
    put_u32_le(&tx_buf_[8], smtc_modem_hal_get_time_in_ms());
    tx_buf_[12] = 0;
    tx_buf_[13] = 0;

    uint16_t offset = kHeaderSize;
    uint8_t frame_count = 0;
    while (frame_count < APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        if (frame.len == 0 || frame.len > APP_OPUS_MAX_PACKET_BYTES ||
            offset + 1U + frame.len > APP_FLRC_MAX_PAYLOAD_BYTES) {
            break;
        }

        tx_buf_[offset++] = static_cast<uint8_t>(frame.len);
        std::memcpy(&tx_buf_[offset], frame.payload, frame.len);
        offset = static_cast<uint16_t>(offset + frame.len);
        frame_count++;

        if (frame_count >= APP_FLRC_OPUS_FRAMES_PER_PACKET ||
            xQueueReceive(tx_queue_, &frame, 0) != pdTRUE) {
            break;
        }
    }

    if (frame_count == 0) {
        return false;
    }

    tx_buf_[12] = frame_count;
    *tx_size = offset;
    return true;
}

void RadioPing::capture_voice_packet()
{
    if (tx_queue_ == nullptr) {
        return;
    }

    audio_proc_.process_tx_frame(tx_pcm_, APP_AUDIO_FRAME_SAMPLES);

    TxFrame frame = {
        .seq = tx_seq_++,
        .len = 0,
        .payload = {},
    };

    int encoded = codec_.encode(tx_pcm_, APP_AUDIO_FRAME_SAMPLES,
                                frame.payload,
                                APP_OPUS_MAX_PACKET_BYTES);
    if (encoded <= 0) {
        ESP_LOGW(TAG, "Opus encode failed: %d", encoded);
        return;
    }
    if (encoded > 255) {
        ESP_LOGW(TAG, "Opus packet too large: %d", encoded);
        return;
    }

    frame.len = static_cast<uint16_t>(encoded);

    if (xQueueSend(tx_queue_, &frame, 0) != pdTRUE) {
        TxFrame dropped;
        (void)xQueueReceive(tx_queue_, &dropped, 0);
        tx_queue_drops_++;
        if (xQueueSend(tx_queue_, &frame, 0) == pdTRUE) {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(tx_queue_drops_));
            }
        } else {
            if ((tx_queue_drops_ % APP_TX_DROP_LOG_EVERY_N) == 1) {
                ESP_LOGW(TAG, "voice TX queue full drops=%lu",
                         static_cast<unsigned long>(tx_queue_drops_));
            }
        }
    }
}

void RadioPing::handle_rx_packet()
{
    // Drain every complete 511-byte fragment queued by one or more RX_DONE events.
    // Leave a partial fragment for the next event to preserve FIFO alignment.
    for (int drained = 0; ; drained++) {
        uint16_t level = 0;
        smtc_modem_hal_protect_api_call();
        ral_status_t lvl_status =
            (ral_status_t) lr20xx_radio_fifo_get_rx_level(radio_.ral.context, &level);
        smtc_modem_hal_unprotect_api_call();

        if (lvl_status != RAL_STATUS_OK || level == 0) {
            break;
        }
        // Later iterations with a sub-fragment level: fragment still in flight.
        if (drained > 0 && level < APP_FLRC_MAX_PAYLOAD_BYTES) {
            break;
        }

        uint16_t take = (level >= APP_FLRC_MAX_PAYLOAD_BYTES) ? APP_FLRC_MAX_PAYLOAD_BYTES : level;

        uint16_t len = 0;
        ral_flrc_rx_pkt_status_t pkt_status = {};
        smtc_modem_hal_protect_api_call();
        ral_status_t status =
            (ral_status_t) lr20xx_radio_fifo_read_rx(radio_.ral.context, rx_buf_, take);
        if (status == RAL_STATUS_OK) {
            len = take;
            status = ral_get_flrc_rx_pkt_status(&radio_.ral, &pkt_status);
        }
        smtc_modem_hal_unprotect_api_call();

        if (status != RAL_STATUS_OK) {
            ESP_LOGW(TAG, "RX read failed: %d", status);
            break;
        }

        dispatch_rx_packet(len, pkt_status.rssi_sync_in_dbm);
    }
}

void RadioPing::dispatch_rx_packet(uint16_t len, int16_t rssi)
{
    if (len < kHeaderSize || std::memcmp(rx_buf_, kMagic, sizeof(kMagic)) != 0) {
        rx_unknown_packets_++;
        if ((rx_unknown_packets_ % 50U) == 1U) {
            ESP_LOGW(TAG, "RX unknown packets=%lu len=%u rssi=%d hdr=%02x%02x%02x%02x",
                     static_cast<unsigned long>(rx_unknown_packets_), len, rssi,
                     rx_buf_[0], rx_buf_[1], rx_buf_[2], rx_buf_[3]);
        }
        return;
    }

    // Valid traffic refreshes an existing low-power FLRC wake window.
    if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
    }

    if (rx_buf_[4] == kPacketTypeVoice) {
        queue_voice_packet(len, rssi);
    } else if (rx_buf_[4] == kPacketTypePing) {
        uint16_t seq = get_u16_le(&rx_buf_[6]);
        log_rx(seq, len, rssi);
    } else if (rx_buf_[4] == kPacketTypeImageCmd) {
        handle_image_cmd();
    } else if (rx_buf_[4] == kPacketTypeImageData) {
        image_rx_last_rssi_ = rssi;
        // Keep continuous RX active through image data; set_rx would clear the next fragment.
        handle_image_data(len);
    } else if (rx_buf_[4] == kPacketTypeImageNack) {
        handle_image_nack();
    } else if (rx_buf_[4] == kPacketTypeImageDone) {
        handle_image_done();
    } else if (rx_buf_[4] == kPacketTypeImageEOT) {
        handle_image_eot();
    } else if (rx_buf_[4] == kPacketTypeImageStart) {
        handle_image_start(len);
    } else if (rx_buf_[4] == kPacketTypeImageCmdAck) {
        handle_image_cmd_ack();
    } else if (rx_buf_[4] == kPacketTypeConfig) {
        uint16_t transaction_id = get_u16_le(&rx_buf_[6]);
        uint8_t key = rx_buf_[8];
        uint32_t value = get_u32_le(&rx_buf_[9]);
        if (key == APP_CFG_KEY_FREQUENCY) {
            ESP_LOGW(TAG, "RX frequency Config: tx=%u hz=%lu", transaction_id,
                     static_cast<unsigned long>(value));
        } else {
            ESP_LOGI(TAG, "RX Config: key=%u value=%lu", key,
                     static_cast<unsigned long>(value));
        }
        if (key == APP_CFG_KEY_FREQUENCY) {
            handle_frequency_config(transaction_id, value);
        } else {
            if (config_received_cb_) {
                config_received_cb_(key, value);
            }
            (void)send_config_ack(key, value, transaction_id);
        }
        // Low power: config applied + ACK sent, work done. End the wake window
        // so the main loop returns to CAD sleep on the next idle pass.
        if (g_low_power_enabled && !is_gateway_ && cad_wakeup_ms_ != 0) {
            cad_wakeup_ms_ = 0;
            ESP_LOGI(TAG, "config ACK sent, ending wake window -> CAD sleep");
        }
    } else if (rx_buf_[4] == kPacketTypeConfigAck) {
        config_ack_transaction_id_ = get_u16_le(&rx_buf_[6]);
        config_ack_key_ = rx_buf_[8];
        config_ack_value_ = get_u32_le(&rx_buf_[9]);
        ESP_LOGI(TAG, "RX ConfigAck: tx=%u key=%u value=%lu",
                 config_ack_transaction_id_, config_ack_key_,
                 static_cast<unsigned long>(config_ack_value_));
        config_ack_received_ = true;
    } else if (rx_buf_[4] == kPacketTypeFrequencyConfirm) {
        handle_frequency_confirm(get_u16_le(&rx_buf_[6]), get_u32_le(&rx_buf_[9]));
    } else if (rx_buf_[4] == kPacketTypeVbat) {
        // Battery voltage broadcast: [14..15] vbat_mv, [16..19] CRC32 over [0..15].
        if (len >= kHeaderSize + 6) {
            uint32_t hdr_crc = crc32_ieee(rx_buf_, 16);
            uint32_t rx_crc = get_u32_le(&rx_buf_[16]);
            if (hdr_crc == rx_crc) {
                uint16_t vbat_mv = get_u16_le(&rx_buf_[14]);
                ESP_LOGI(TAG, "RX Vbat: %u mV (%u.%02u V)", vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
                if (vbat_mv > 0 && vbat_received_cb_) vbat_received_cb_(vbat_mv);
            } else {
                ESP_LOGW(TAG, "RX Vbat CRC mismatch, dropping");
            }
        }
    } else {
        ESP_LOGW(TAG, "RX unsupported packet type=%u len=%u rssi=%d", rx_buf_[4], len, rssi);
    }
}

void RadioPing::queue_voice_packet(uint16_t len, int16_t rssi)
{
    uint8_t frame_count = rx_buf_[12];
    if (frame_count == 0 || frame_count > APP_FLRC_OPUS_FRAMES_PER_PACKET) {
        ESP_LOGW(TAG, "RX bad voice packet len=%u frames=%u", len, frame_count);
        return;
    }

    uint16_t seq = get_u16_le(&rx_buf_[6]);
    uint16_t offset = kHeaderSize;
    for (uint8_t i = 0; i < frame_count; i++) {
        if (offset >= len) {
            ESP_LOGW(TAG, "RX truncated voice packet len=%u frames=%u", len, frame_count);
            return;
        }

        uint8_t opus_len = rx_buf_[offset++];
        if (opus_len == 0 || opus_len > APP_OPUS_MAX_PACKET_BYTES || offset + opus_len > len) {
            ESP_LOGW(TAG, "RX bad voice frame len=%u opus_len=%u", len, opus_len);
            return;
        }

        uint16_t frame_seq = static_cast<uint16_t>(seq + i);
        log_rx(frame_seq, len, rssi);

        VoicePacket packet = {
            .seq = frame_seq,
            .len = opus_len,
            .rssi = rssi,
            .payload = {},
        };
        std::memcpy(packet.payload, &rx_buf_[offset], opus_len);
        offset = static_cast<uint16_t>(offset + opus_len);

        if (xQueueSend(voice_queue_, &packet, 0) != pdTRUE) {
            VoicePacket dropped;
            (void)xQueueReceive(voice_queue_, &dropped, 0);
            rx_queue_drops_++;
            if (xQueueSend(voice_queue_, &packet, 0) == pdTRUE) {
                ESP_LOGW(TAG, "voice queue full, dropped oldest seq=%u drops=%lu",
                         dropped.seq, static_cast<unsigned long>(rx_queue_drops_));
            } else {
                ESP_LOGW(TAG, "voice queue full drops=%lu seq=%u",
                         static_cast<unsigned long>(rx_queue_drops_), frame_seq);
            }
        }
    }
}

void RadioPing::log_rx(uint16_t seq, uint16_t len, int16_t rssi)
{
    if (!have_expected_rx_seq_) {
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
        have_expected_rx_seq_ = true;
    } else if (seq != expected_rx_seq_) {
        uint16_t gap = static_cast<uint16_t>(seq - expected_rx_seq_);
        if (gap < 0x8000) {
            rx_lost_ += gap;
            ESP_LOGW(TAG, "FLRC loss gap=%u expected=%u got=%u total_lost=%lu rssi=%d dBm",
                     gap, expected_rx_seq_, seq, static_cast<unsigned long>(rx_lost_), rssi);
        }
        expected_rx_seq_ = static_cast<uint16_t>(seq + 1);
    } else {
        expected_rx_seq_ = static_cast<uint16_t>(expected_rx_seq_ + 1);
    }

    rx_packets_++;
}

void RadioPing::wait_for_jitter_buffer()
{
    if (playback_active_ || voice_queue_ == nullptr || APP_RX_JITTER_FRAMES <= 1U) {
        return;
    }

    const UBaseType_t target_waiting = static_cast<UBaseType_t>(APP_RX_JITTER_FRAMES - 1U);
    const uint32_t start_ms = smtc_modem_hal_get_time_in_ms();
    while (uxQueueMessagesWaiting(voice_queue_) < target_waiting) {
        if (smtc_modem_hal_get_time_in_ms() - start_ms >= APP_RX_JITTER_BUFFER_MS) {
            break;
        }
        vTaskDelay(ms_to_ticks_min_1(1));
    }
}

void RadioPing::conceal_missing_frames(uint16_t seq)
{
    if (!have_expected_play_seq_) {
        expected_play_seq_ = seq;
        have_expected_play_seq_ = true;
    }

    uint16_t gap = static_cast<uint16_t>(seq - expected_play_seq_);
    if (gap > 0 && gap <= APP_RX_MAX_PLC_FRAMES) {
        for (uint16_t i = 0; i < gap; i++) {
            int decoded = codec_.decode_lost(rx_pcm_, APP_AUDIO_FRAME_SAMPLES);
            if (decoded <= 0) {
                ESP_LOGW(TAG, "Opus PLC failed: %d", decoded);
                break;
            }
            play_mono_frame(rx_pcm_, static_cast<size_t>(decoded));
            last_rx_audio_ms_ = smtc_modem_hal_get_time_in_ms();
            playback_active_ = true;
        }
    }

    expected_play_seq_ = static_cast<uint16_t>(seq + 1);
}

bool RadioPing::read_mono_frame(int16_t *mono, size_t samples)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    size_t got_total = 0;
    const size_t target = samples * 2 * sizeof(int16_t);

    while (got_total < target) {
        size_t got = 0;
        esp_err_t err = bsp_audio_read(reinterpret_cast<uint8_t *>(stereo) + got_total,
                                       target - got_total, &got);
        if (err != ESP_OK || got == 0) {
            if (!image_tx_active_) {
                //ESP_LOGW(TAG, "audio read failed: %s got=%u",
                         //esp_err_to_name(err), static_cast<unsigned>(got));
            }
            return false;
        }
        got_total += got;
    }

    for (size_t i = 0; i < samples; i++) {
        int16_t left = stereo[2 * i];
        int16_t right = stereo[2 * i + 1];
        mono[i] = (abs16(left) >= abs16(right)) ? left : right;
    }
    return true;
}

void RadioPing::play_mono_frame(const int16_t *mono, size_t samples)
{
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    if (samples > APP_AUDIO_FRAME_SAMPLES) {
        samples = APP_AUDIO_FRAME_SAMPLES;
    }

    for (size_t i = 0; i < samples; i++) {
        stereo[2 * i] = mono[i];
        stereo[2 * i + 1] = mono[i];
    }

    set_playback_pa(true);
    size_t written = 0;
    esp_err_t err = bsp_audio_write(stereo, samples * 2 * sizeof(int16_t), &written);
    if (err != ESP_OK || written == 0) {
        ESP_LOGW(TAG, "audio write failed: %s written=%u",
                 esp_err_to_name(err), static_cast<unsigned>(written));
    }
}

void RadioPing::set_playback_pa(bool on)
{
    if (playback_pa_on_ == on) {
        return;
    }
    esp_err_t err = bsp_audio_pa_enable(on);
    if (err == ESP_OK) {
        playback_pa_on_ = on;
    } else {
        ESP_LOGW(TAG, "PA %s failed: %s", on ? "enable" : "disable", esp_err_to_name(err));
    }
}

void RadioPing::update_playback_timeout()
{
    if (!playback_active_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (now - last_rx_audio_ms_ > APP_RX_AUDIO_TIMEOUT_MS) {
        set_playback_pa(false);
        playback_active_ = false;
        have_expected_play_seq_ = false;
    }
}

// --- Image transfer implementation ---

void RadioPing::image_tx_task_trampoline(void *arg)
{
    static_cast<RadioPing *>(arg)->image_tx_task();
}

bool RadioPing::trigger_image_capture()
{
    // Reject new image requests while a transfer is active to prevent half-duplex deadlock.
    if (image_busy()) {
        ESP_LOGW(TAG, "image transfer in progress (tx=%d rx=%d req=%d), ignoring new request",
                 image_tx_active_, image_rx_pending_, image_req_active_);
        return false;
    }

    // Pick the session ONCE for this whole request. Retries reuse it (they do
    // NOT ++), so a resend can never spawn a second capture / a different JPEG.
    image_req_session_ = image_session_id_++;
    if (image_session_id_ == 0) {
        image_session_id_ = 1;
    }
    // Start ImageCmd retries; low-power mode first opens the node's LoRa wake window.
    image_req_active_ = true;
    // Suspend radio polling during the first wakeup so TX_DONE has one IRQ consumer.
    bool was_suspended = suspended_;
    suspended_ = true;
    start_image_req_round();
    suspended_ = was_suspended;

    image_rx_pending_ = true;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    schedule_rx();
    return true;
}

// Start a low-power request round with a LoRa wakeup, then retry ImageCmd every 30 ms.
// Normal mode skips the wakeup and retries continuously.
void RadioPing::start_image_req_round()
{
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (g_low_power_enabled) {
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed (round retry)");
            // Back off before the next wakeup attempt rather than spinning. Push
            // round_end out too so the round-timeout check in check_image_req_retry
            // doesn't re-fire immediately and defeat the backoff.
            uint32_t backoff = now + APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS;
            image_req_round_end_ms_ = backoff;
            image_req_next_ms_ = backoff;
            return;
        }
        configure_flrc();
        image_req_round_end_ms_ = now + APP_IMAGE_REQ_ROUND_MS;
    }
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

// Send one ImageCmd for image_req_session_ (build + TX only). The LoRa wakeup /
// FLRC reconfig is handled once per round by start_image_req_round.
void RadioPing::send_image_cmd_once()
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageCmd;
    pkt[5] = 1;
    put_u16_le(&pkt[6], image_req_session_);
    put_u32_le(&pkt[8], smtc_modem_hal_get_time_in_ms());
    pkt[12] = 0;
    pkt[13] = 0;

    image_cmd_sent_ms_ = smtc_modem_hal_get_time_in_ms();

    // Stop RX before TX
    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    send_single_packet(pkt, kHeaderSize);
    // Keep the RX watchdog fresh so a long capture doesn't trip the 10s giveup.
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
}

// Runs in the radio task (next to poll_once), so send_image_cmd_once here shares
// the task with RX handling — no IRQ/mode races. Resends ImageCmd once per
// interval until the node acks or an ImageStart backstop clears image_req_active_.
void RadioPing::check_image_req_retry()
{
    if (!image_req_active_) return;
    // A transfer already started (ImageStart / data) — stop requesting.
    if (image_tx_active_) {
        image_req_active_ = false;
        return;
    }
    uint32_t now = smtc_modem_hal_get_time_in_ms();

    // Start a new low-power wakeup round when the current request window expires.
    if (g_low_power_enabled && (int32_t)(now - image_req_round_end_ms_) >= 0) {
        ESP_LOGI(TAG, "ImageCmd round timed out, new wakeup round (session=%u)",
                 image_req_session_);
        start_image_req_round();
        return;
    }

    if ((int32_t)(now - image_req_next_ms_) < 0) return;

    // Flood cadence: one ImageCmd, then a short RX window to catch the node's
    // ImageStart. 30ms in both modes (non-harmonic with the node's 50ms
    // ImageStart retry so they interleave).
    send_image_cmd_once();
    if (mode_ != Mode::rx_pending) {
        schedule_rx();
    }
    image_req_next_ms_ = smtc_modem_hal_get_time_in_ms() + APP_IMAGE_REQ_RETRY_INTERVAL_MS;
}

void RadioPing::stop_image_req_retry()
{
    image_req_active_ = false;
}

// image_tx_task owns and frees jpeg; a queue failure frees it here.
void RadioPing::send_image(const uint8_t *jpeg, size_t jpeg_len, uint16_t session_id)
{
    if (!image_tx_queue_) {
        ESP_LOGE(TAG, "image_tx_queue not initialized");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
        return;
    }
    ImageTxRequest req = { .jpeg = jpeg, .jpeg_len = jpeg_len, .session_id = session_id };
    if (xQueueSend(image_tx_queue_, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGE(TAG, "image_tx_queue full");
        heap_caps_free(const_cast<uint8_t *>(jpeg));
    }
}

/* Image transfer: send all fragments, then EOT; the receiver ACKs missing indices.
 * Retransmit only missing fragments and repeat until complete or timed out. */
void RadioPing::image_tx_task()
{
    ImageTxRequest req;
    while (true) {
        if (xQueueReceive(image_tx_queue_, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        image_tx_active_ = true;
        image_done_received_ = false;
        image_nack_received_ = false;
        suspended_ = true;

        // Clear the CAD flag before FLRC setup so later standby powers down the camera correctly.
        low_power_cad_active_ = false;

        vTaskDelay(pdMS_TO_TICKS(APP_RADIO_TASK_POLL_MS * 2));

        smtc_modem_hal_protect_api_call();
        if (!configure_flrc()) {
            ESP_LOGE(TAG, "image TX: configure_flrc failed");
        }
        smtc_modem_hal_unprotect_api_call();

        uint16_t total_fragments = static_cast<uint16_t>(
            (req.jpeg_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
        uint32_t jpeg_crc32 = crc32_ieee(req.jpeg, req.jpeg_len);

        // ESP_LOGI(TAG, "image TX start: session=%u jpeg=%u bytes frags=%u crc32=0x%08lx",
        //          req.session_id, static_cast<unsigned>(req.jpeg_len), total_fragments,
        //          static_cast<unsigned long>(jpeg_crc32));

        bool was_ptt = ptt_active_;
        ptt_active_ = false;
        tx_burst_active_ = false;
        if (mode_ == Mode::rx_pending) {
            smtc_modem_hal_protect_api_call();
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            smtc_modem_hal_unprotect_api_call();
            mode_ = Mode::idle;
        }

        // Retry ImageStart until the receiver is ready; the 50 ms interval avoids retry phase lock.
        bool r_ready = false;
        for (uint16_t start_try = 0; start_try < APP_IMAGE_START_RETRY_COUNT && !r_ready; start_try++) {
            // ImageStart appends CRC32 over the header and battery voltage.
            uint8_t start_pkt[kHeaderSize + 6];
            std::memcpy(start_pkt, kMagic, sizeof(kMagic));
            start_pkt[4] = kPacketTypeImageStart;
            start_pkt[5] = 1;
            put_u16_le(&start_pkt[6], req.session_id);
            put_u16_le(&start_pkt[8], total_fragments);
            put_u32_le(&start_pkt[10], jpeg_crc32);
            put_u16_le(&start_pkt[14], bsp_vbat_get_cached());  // battery voltage mV
            put_u32_le(&start_pkt[16], crc32_ieee(start_pkt, 16));
            send_single_packet(start_pkt, kHeaderSize + 6);

            image_nack_received_ = false;
            image_done_received_ = false;
            schedule_rx();

            uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
            while (!image_nack_received_ && !image_done_received_) {
                if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_START_RETRY_INTERVAL_MS) {
                    break;
                }
                if (irq_pending_) {
                    irq_pending_ = false;
                    ral_irq_t irq = RAL_IRQ_NONE;
                    smtc_modem_hal_protect_api_call();
                    ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                    smtc_modem_hal_unprotect_api_call();
                    if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                        handle_irq(irq);
                    }
                }
                taskYIELD();
            }

            if (image_nack_received_ || image_done_received_) {
                r_ready = true;
                // ESP_LOGI(TAG, "image TX: R ready, starting data burst");
            } else {
            // ESP_LOGW(TAG, "image TX: ImageStart no response, retry %u/%u",
            //          start_try + 1, APP_IMAGE_START_RETRY_COUNT);
            }
        }

        if (!r_ready) {
            // ESP_LOGW(TAG, "image TX: R not ready, aborting");
            image_tx_active_ = false;
            suspended_ = false;
            ptt_active_ = was_ptt;
            if (g_low_power_enabled && !is_gateway_ && (cad_wakeup_ms_ != 0 || pir_push_wake_)) {
                cad_wakeup_ms_ = 0;
                pir_push_wake_ = false;
                ESP_LOGI(TAG, "image TX aborted, ending wake window -> CAD sleep");
            }
            if (!ptt_active_) schedule_rx();
            // We own req.jpeg (see send_image) — free before looping for the
            // next request, even on the abort path.
            heap_caps_free(const_cast<uint8_t *>(req.jpeg));
            req.jpeg = nullptr;
            continue;
        }

        // Step 1: FLRC BURST — stream every fragment back-to-back (sequential
        // 0..total_fragments-1). indices=nullptr selects the sequential path.
        burst_send_fragments(req, total_fragments, nullptr, total_fragments);

        // ESP_LOGI(TAG, "image TX: initial burst done (%u frags)", total_fragments);

        // Step 2-8: EOT + wait ACK + retransmit loop
        bool transfer_done = false;
        // Abort after one wake window without any ACK or NACK activity.
        uint32_t last_interaction_ms = smtc_modem_hal_get_time_in_ms();
        for (uint16_t round = 0; round < APP_IMAGE_NACK_MAX_RETRIES && !transfer_done; round++) {
            if (smtc_modem_hal_get_time_in_ms() - last_interaction_ms > APP_LP_WAKE_WINDOW_MS) {
                ESP_LOGW(TAG, "image TX: no ACK/NACK for %ums, aborting transfer",
                         static_cast<unsigned>(APP_LP_WAKE_WINDOW_MS));
                break;
            }
            // Give R time to process last packets before sending EOT
            vTaskDelay(pdMS_TO_TICKS(30));

            // Send EOT, retry if no response
            bool got_response = false;
            for (uint16_t eot_try = 0; eot_try < APP_IMAGE_EOT_RETRY_COUNT; eot_try++) {
                uint8_t eot[kHeaderSize];
                std::memcpy(eot, kMagic, sizeof(kMagic));
                eot[4] = kPacketTypeImageEOT;
                eot[5] = 1;
                put_u16_le(&eot[6], req.session_id);
                put_u16_le(&eot[8], total_fragments);
                eot[10] = 0; eot[11] = 0; eot[12] = 0; eot[13] = 0;
                send_single_packet(eot, kHeaderSize);

                // Wait for ACK
                image_nack_received_ = false;
                image_done_received_ = false;
                schedule_rx();

                uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
                while (!image_nack_received_ && !image_done_received_) {
                    if (smtc_modem_hal_get_time_in_ms() - wait_start > APP_IMAGE_EOT_RETRY_INTERVAL_MS) {
                        break;
                    }
                    if (irq_pending_) {
                        irq_pending_ = false;
                        ral_irq_t irq = RAL_IRQ_NONE;
                        smtc_modem_hal_protect_api_call();
                        ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                        smtc_modem_hal_unprotect_api_call();
                        if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                            handle_irq(irq);
                        }
                    }
                    taskYIELD();
                }

                if (image_nack_received_ || image_done_received_) {
                    got_response = true;
                    break;
                }
                // ESP_LOGW(TAG, "image TX: EOT no response, retry %u/%u",
                //          eot_try + 1, APP_IMAGE_EOT_RETRY_COUNT);
            }

            if (!got_response) {
                // Without an ACK, resend EOT instead of flooding the full image.
                // Retransmit data only when an explicit NACK lists missing fragments.
                continue;
            }

            // Got a response — the link is alive, reset the no-interaction timer.
            last_interaction_ms = smtc_modem_hal_get_time_in_ms();

            if (image_done_received_ || nack_count_ == 0) {
                // ESP_LOGI(TAG, "image TX complete: all received");
                transfer_done = true;
                break;
            }

            // Retransmit only the fragment indices listed by the NACK.
            burst_send_fragments(req, total_fragments, nack_indices_, nack_count_);
        }

        image_tx_active_ = false;
        suspended_ = false;
        ptt_active_ = was_ptt;
        // ESP_LOGI(TAG, "image TX finished: session=%u done=%d",
        //          req.session_id, transfer_done ? 1 : 0);

        // Clear the active wake guard so low-power operation can resume.
        if (g_low_power_enabled && !is_gateway_ && (cad_wakeup_ms_ != 0 || pir_push_wake_)) {
            cad_wakeup_ms_ = 0;
            pir_push_wake_ = false;
            ESP_LOGI(TAG, "image TX done, ending wake window -> CAD sleep");
        }

        if (mode_ != Mode::rx_pending && !ptt_active_) {
            schedule_rx();
        }

        // Free the task-owned JPEG after the transfer ends.
        heap_caps_free(const_cast<uint8_t *>(req.jpeg));
        req.jpeg = nullptr;
    }
}

uint16_t RadioPing::build_image_fragment(uint8_t *pkt, const ImageTxRequest &req,
                                         uint16_t frag_index, uint16_t total_fragments)
{
    size_t offset = static_cast<size_t>(frag_index) * APP_IMAGE_FRAGMENT_DATA_SIZE;
    uint16_t frag_len = static_cast<uint16_t>(
        ((offset + APP_IMAGE_FRAGMENT_DATA_SIZE) <= req.jpeg_len)
            ? APP_IMAGE_FRAGMENT_DATA_SIZE
            : (req.jpeg_len - offset));

    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageData;
    pkt[5] = 1;
    put_u16_le(&pkt[6], req.session_id);
    put_u16_le(&pkt[8], frag_index);
    put_u16_le(&pkt[10], total_fragments);
    put_u16_le(&pkt[12], frag_len);
    std::memcpy(&pkt[kHeaderSize], req.jpeg + offset, frag_len);
    uint16_t crc = crc16_ccitt(&pkt[4], kHeaderSize - 4 + frag_len);
    put_u16_le(&pkt[kHeaderSize + frag_len], crc);

    return static_cast<uint16_t>(kHeaderSize + frag_len + 2);
}

void RadioPing::burst_send_fragments(const ImageTxRequest &req, uint16_t total_fragments,
                                     const uint16_t *indices, uint16_t count)
{
    if (count == 0) {
        return;
    }

    const void *ctx = radio_.ral.context;

    // Map the streaming position to a fragment index: sequential 0..count-1 when
    // no explicit index list is given, otherwise the caller's NACK index list.
    auto frag_at = [&](uint16_t pos) -> uint16_t {
        return indices ? indices[pos] : pos;
    };

    // Keep TCXO and the antenna active; FALLBACK_FS avoids relocking between burst packets.
    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    (void)ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_FS);
    (void)lr20xx_radio_fifo_clear_tx(ctx);
    smtc_modem_hal_unprotect_api_call();

    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    uint16_t next_write = 0;  // next streaming position to write into the FIFO

    // Pad every burst fragment to 511 bytes so queued RX packets stay FIFO-aligned.
    // frag_len identifies valid data and excludes trailing padding.
    auto build_padded = [&](uint16_t pos) {
        uint16_t len = build_image_fragment(pkt, req, frag_at(pos), total_fragments);
        if (len < APP_FLRC_MAX_PAYLOAD_BYTES) {
            std::memset(pkt + len, 0, APP_FLRC_MAX_PAYLOAD_BYTES - len);
        }
    };

    // Prefill up to 2 packets so the FIFO holds one in-flight + one queued.
    for (int prefill = 0; prefill < 2 && next_write < count; prefill++) {
        build_padded(next_write);
        smtc_modem_hal_protect_api_call();
        (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_MAX_PAYLOAD_BYTES);
        smtc_modem_hal_unprotect_api_call();
        next_write++;
    }

    // Launch: sends FIFO packet #0; any prefilled #1 waits in the FIFO.
    mode_ = Mode::tx_pending;
    smtc_modem_hal_protect_api_call();
    (void)ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    // For each remaining position: wait for the current packet's TX_DONE, fire
    // set_tx to launch the already-buffered next packet, then refill one more.
    for (uint16_t pos = 1; pos < count; pos++) {
        (void)wait_for_tx_done(50);

        mode_ = Mode::tx_pending;
        smtc_modem_hal_protect_api_call();
        (void)ral_set_tx(&radio_.ral);
        if (next_write < count) {
            build_padded(next_write);
            (void)lr20xx_radio_fifo_write_tx(ctx, pkt, APP_FLRC_MAX_PAYLOAD_BYTES);
        }
        smtc_modem_hal_unprotect_api_call();
        if (next_write < count) {
            next_write++;
        }
    }

    // Wait for the final packet to drain, then restore the normal STDBY_XOSC
    // fallback so the handshake path (EOT/ACK/NACK) turns to RX as before.
    (void)wait_for_tx_done(50);

    smtc_modem_hal_protect_api_call();
    (void)lr20xx_radio_common_set_rx_tx_fallback_mode(ctx, LR20XX_RADIO_FALLBACK_STDBY_XOSC);
    smtc_modem_hal_unprotect_api_call();
    mode_ = Mode::idle;
}

bool RadioPing::send_single_packet(const uint8_t *data, uint16_t len)
{
    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);
    ral_status_t status = ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    if (status == RAL_STATUS_OK) status = ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    if (status == RAL_STATUS_OK) status = ral_set_pkt_payload(&radio_.ral, data, len);
    if (status == RAL_STATUS_OK) status = ral_set_tx(&radio_.ral);
    smtc_modem_hal_unprotect_api_call();

    if (status != RAL_STATUS_OK) {
        return false;
    }

    mode_ = Mode::tx_pending;
    return wait_for_tx_done(50);
}

bool RadioPing::wait_for_tx_done(uint32_t timeout_ms)
{
    uint32_t start = smtc_modem_hal_get_time_in_ms();
    while (true) {
        if (irq_pending_) {
            irq_pending_ = false;
            ral_irq_t irq = RAL_IRQ_NONE;
            smtc_modem_hal_protect_api_call();
            ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
            smtc_modem_hal_unprotect_api_call();
            if (s == RAL_STATUS_OK && (irq & RAL_IRQ_TX_DONE)) {
                mode_ = Mode::idle;
                return true;
            }
        }
        if (smtc_modem_hal_get_time_in_ms() - start > timeout_ms) {
            mode_ = Mode::idle;
            return false;
        }
        taskYIELD();
    }
}

void RadioPing::handle_image_cmd()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);

    // Decide whether the capture was actually dispatched before acknowledging.
    // This prevents the gateway from waiting for an image that a busy node
    // intentionally did not start.
    ImageCmdAckStatus status = ImageCmdAckStatus::rejected;
    if (image_capture_cb_) {
        status = image_capture_cb_(session_id);
    }

    uint8_t ack[kHeaderSize];
    std::memcpy(ack, kMagic, sizeof(kMagic));
    ack[4] = kPacketTypeImageCmdAck;
    ack[5] = 1;
    put_u16_le(&ack[6], session_id);
    ack[8] = static_cast<uint8_t>(status);
    ack[9] = 0; ack[10] = 0; ack[11] = 0; ack[12] = 0; ack[13] = 0;

    if (mode_ == Mode::rx_pending) {
        smtc_modem_hal_protect_api_call();
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        mode_ = Mode::idle;
    }
    send_single_packet(ack, kHeaderSize);
    schedule_rx();

    if (status != ImageCmdAckStatus::accepted) {
        ESP_LOGW(TAG, "TX ImageCmdAck: session=%u status=%u",
                 session_id, static_cast<unsigned>(status));
    }
}

// Gateway side: node acknowledged the ImageCmd. Stop the request-retry timer.
void RadioPing::handle_image_cmd_ack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    if (session_id != image_req_session_) {
        return;
    }

    uint8_t raw_status = rx_buf_[8];
    ImageCmdAckStatus status = raw_status <= static_cast<uint8_t>(ImageCmdAckStatus::rejected)
                                   ? static_cast<ImageCmdAckStatus>(raw_status)
                                   : ImageCmdAckStatus::rejected;

    stop_image_req_retry();

    if (status == ImageCmdAckStatus::accepted ||
        status == ImageCmdAckStatus::duplicate) {
        ESP_LOGI(TAG, "RX ImageCmdAck: session=%u status=%u",
                 session_id, static_cast<unsigned>(status));
        schedule_rx();
        return;
    }

    // BUSY/COOLDOWN/REJECTED means no image will follow. Clear the gateway
    // request state immediately instead of holding the UI in image_busy() until
    // the 10-second no-data timeout expires.
    ESP_LOGW(TAG, "RX ImageCmdAck rejected: session=%u status=%u",
             session_id, static_cast<unsigned>(status));
    image_rx_pending_ = false;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
    image_rx_eot_count_ = 0;
    if (image_request_rejected_cb_) {
        image_request_rejected_cb_(status);
    }
    schedule_rx();
}

void RadioPing::handle_image_start(uint16_t len)
{
    // Validate ImageStart with CRC32 before using its fragment count or battery value.
    // Accept legacy 18-byte and current 20-byte packet layouts.
    bool has_vbat = (len >= kHeaderSize + 6);
    uint16_t crc_len = has_vbat ? 16 : kHeaderSize;
    uint16_t crc_offset = has_vbat ? 16 : kHeaderSize;

    if (len < crc_offset + 4) {
        ESP_LOGW(TAG, "ImageStart too short (len=%u), dropping", len);
        return;
    }
    uint32_t hdr_crc = crc32_ieee(rx_buf_, crc_len);
    uint32_t rx_hdr_crc = get_u32_le(&rx_buf_[crc_offset]);
    if (hdr_crc != rx_hdr_crc) {
        ESP_LOGW(TAG, "ImageStart header CRC32 mismatch (calc=0x%08lx rx=0x%08lx), dropping",
                 static_cast<unsigned long>(hdr_crc), static_cast<unsigned long>(rx_hdr_crc));
        return;
    }

    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);
    uint32_t expected_crc32 = get_u32_le(&rx_buf_[10]);
    uint16_t vbat_mv = has_vbat ? get_u16_le(&rx_buf_[14]) : 0;

    if (has_vbat) {
        ESP_LOGI(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx vbat=%u mV (%u.%02u V)",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32),
                 vbat_mv, vbat_mv / 1000, (vbat_mv % 1000) / 10);
        if (vbat_mv > 0 && vbat_received_cb_) vbat_received_cb_(vbat_mv);
    } else {
        ESP_LOGI(TAG, "RX ImageStart: session=%u total=%u crc32=0x%08lx",
                 session_id, total_frags, static_cast<unsigned long>(expected_crc32));
    }

    // Backstop: an ImageStart proves the node got our request, so stop resending
    // ImageCmd even if its ImageCmdAck was lost.
    stop_image_req_retry();

    // Ignore duplicate ImageStart packets after data arrives to avoid resetting RX progress.
    if (image_rx_pending_ &&
        session_id == image_xfer_.rx_session_id() &&
        image_xfer_.rx_received_count() > 0) {
        return;
    }

    image_rx_start_ms_ = smtc_modem_hal_get_time_in_ms();

    // Prepare RX buffer
    image_xfer_.rx_begin(session_id, total_frags);
    if (!image_xfer_.rx_active()) {
        image_rx_pending_ = false;
        if (image_rx_error_cb_) {
            image_rx_error_cb_(ImageRxError::no_memory);
        }
        schedule_rx();
        return;
    }
    image_rx_pending_ = true;
    image_rx_nack_sent_ = 0;
    image_rx_eot_count_ = 0;
    image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_last_progress_ms_ = smtc_modem_hal_get_time_in_ms();
    image_rx_expected_crc32_ = expected_crc32;

    // Queue the UI update before sending ACK. The callback only copies a small
    // value event now, so it never waits for LVGL or performs an LCD flush here.
    if (image_rx_progress_cb_) {
        image_rx_progress_cb_(session_id, 0, total_frags, 0);
    }

    // Send ACK (ready) — missing_count=0 means "ready"
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);
    put_u16_le(&pkt[8], 0);  // missing_count = 0 (ready signal)
    put_u16_le(&pkt[10], 0); // total_received = 0
    pkt[12] = 0; pkt[13] = 0;
    send_single_packet(pkt, kHeaderSize);

    // Enter RX for incoming data
    schedule_rx();
}

void RadioPing::handle_image_data(uint16_t len)
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t frag_index = get_u16_le(&rx_buf_[8]);
    uint16_t total_frags = get_u16_le(&rx_buf_[10]);
    uint16_t frag_len = get_u16_le(&rx_buf_[12]);

    if (!image_rx_pending_ || session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        return;
    }

    if (frag_len > APP_IMAGE_FRAGMENT_DATA_SIZE) {
        ESP_LOGW(TAG, "RX ImageData: bad frag_len=%u", frag_len);
        return;
    }

    // Reject fragments shorter than their declared payload and CRC length.
    if (len < static_cast<uint16_t>(kHeaderSize + frag_len + 2)) {
        return;
    }

    // Verify CRC16 appended after payload
    uint16_t rx_crc = get_u16_le(&rx_buf_[kHeaderSize + frag_len]);
    uint16_t calc_crc = crc16_ccitt(&rx_buf_[4], kHeaderSize - 4 + frag_len);
    if (rx_crc != calc_crc) {
        return;
    }

    bool complete = image_xfer_.rx_fragment(session_id, frag_index, total_frags,
                                            &rx_buf_[kHeaderSize], frag_len);
    uint32_t now_ms = smtc_modem_hal_get_time_in_ms();
    image_rx_last_frag_ms_ = now_ms;
    image_rx_pending_ = true;
    if (image_rx_progress_cb_ &&
        (complete || now_ms - image_rx_last_progress_ms_ >= 25U)) {
        image_rx_progress_cb_(session_id, image_xfer_.rx_received_count(),
                              image_xfer_.rx_total_count(), image_rx_last_rssi_);
        image_rx_last_progress_ms_ = now_ms;
    }
    if (frag_index == 0) {
        image_rx_nack_sent_ = 0;
    }
}

void RadioPing::handle_image_nack()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t missing_count = get_u16_le(&rx_buf_[8]);

    if (missing_count > APP_IMAGE_NACK_MAX_INDICES) {
        missing_count = APP_IMAGE_NACK_MAX_INDICES;
    }

    nack_count_ = missing_count;
    for (uint16_t i = 0; i < missing_count; i++) {
        nack_indices_[i] = get_u16_le(&rx_buf_[kHeaderSize + i * 2]);
    }

    uint16_t total_received = get_u16_le(&rx_buf_[10]);
    // ESP_LOGI(TAG, "RX ImageACK: session=%u missing=%u received=%u",
    //          session_id, missing_count, total_received);
    image_nack_received_ = true;
    if (missing_count == 0) {
        image_done_received_ = true;
    }
}

void RadioPing::handle_image_done()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    // ESP_LOGI(TAG, "RX ImageDone: session=%u", session_id);
    image_done_received_ = true;
}

void RadioPing::handle_image_eot()
{
    uint16_t session_id = get_u16_le(&rx_buf_[6]);
    uint16_t total_frags = get_u16_le(&rx_buf_[8]);

    // ESP_LOGI(TAG, "RX ImageEOT: session=%u received=%u/%u",
    //          session_id, image_xfer_.rx_received_count(), total_frags);

    if (!image_rx_pending_) {
        // Already completed — still send ACK so T stops retrying
        if (session_id == image_rx_done_session_) {
            uint8_t pkt[kHeaderSize];
            std::memcpy(pkt, kMagic, sizeof(kMagic));
            pkt[4] = kPacketTypeImageNack;
            pkt[5] = 3;
            put_u16_le(&pkt[6], session_id);
            put_u16_le(&pkt[8], 0);
            put_u16_le(&pkt[10], 0);
            pkt[12] = 0; pkt[13] = 0;
            send_single_packet(pkt, kHeaderSize);
            schedule_rx();
        }
        return;
    }

    if (session_id != image_xfer_.rx_session_id() ||
        total_frags != image_xfer_.rx_total_count()) {
        ESP_LOGW(TAG, "ignoring stale ImageEOT: session=%u total=%u, active=%u/%u",
                 session_id, total_frags, image_xfer_.rx_session_id(),
                 image_xfer_.rx_total_count());
        return;
    }

    // Exit continuous RX to send response
    if (mode_ == Mode::rx_pending) {
        smtc_modem_hal_protect_api_call();
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        smtc_modem_hal_unprotect_api_call();
        mode_ = Mode::idle;
    }

    // Update UI before sending NACK — node won't retransmit until it
    // receives our reply; queued UI work cannot block this radio task.
    if (image_rx_progress_cb_) {
        uint16_t total = image_xfer_.rx_total_count();
        image_rx_progress_cb_(session_id, image_xfer_.rx_received_count(),
                              total, image_rx_last_rssi_);
    }

    // Build ACK with missing indices
    uint8_t pkt[APP_FLRC_MAX_PAYLOAD_BYTES];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeImageNack;
    pkt[5] = 3;
    put_u16_le(&pkt[6], session_id);

    uint16_t missing_indices[APP_IMAGE_NACK_MAX_INDICES];
    uint16_t missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
    if (missing_count == 0 && image_rx_expected_crc32_ != 0) {
        uint32_t actual_crc32 = image_xfer_.rx_crc32();
        if (actual_crc32 != image_rx_expected_crc32_) {
            ESP_LOGW(TAG, "image RX crc32 mismatch: expected=0x%08lx actual=0x%08lx, requesting full resend",
                     static_cast<unsigned long>(image_rx_expected_crc32_),
                     static_cast<unsigned long>(actual_crc32));
            image_xfer_.rx_begin(session_id, total_frags);
            if (!image_xfer_.rx_active()) {
                image_rx_pending_ = false;
                if (image_rx_error_cb_) {
                    image_rx_error_cb_(ImageRxError::no_memory);
                }
                schedule_rx();
                return;
            }
            image_rx_pending_ = true;
            image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
            image_rx_last_progress_ms_ = 0;
            if (image_rx_error_cb_) {
                image_rx_error_cb_(ImageRxError::crc_mismatch);
            }
            missing_count = image_xfer_.rx_get_missing(missing_indices, APP_IMAGE_NACK_MAX_INDICES);
        } else {
            // ESP_LOGI(TAG, "image RX crc32 ok: 0x%08lx",
            //          static_cast<unsigned long>(actual_crc32));
        }
    }

    put_u16_le(&pkt[8], missing_count);
    put_u16_le(&pkt[10], image_xfer_.rx_received_count());
    pkt[12] = 0; pkt[13] = 0;

    for (uint16_t i = 0; i < missing_count; i++) {
        put_u16_le(&pkt[kHeaderSize + i * 2], missing_indices[i]);
    }

    uint16_t pkt_len = static_cast<uint16_t>(kHeaderSize + missing_count * 2);
    send_single_packet(pkt, pkt_len);
    schedule_rx();

    if (image_rx_eot_cb_) {
        bool is_first = (image_rx_eot_count_ == 0);
        image_rx_eot_count_++;
        image_rx_eot_cb_(missing_count, is_first);
    }

    if (missing_count == 0) {
        image_rx_done_session_ = session_id;
        uint32_t now_ms = smtc_modem_hal_get_time_in_ms();
        uint32_t transfer_ms = now_ms - image_rx_start_ms_;
        uint32_t prepare_ms = image_rx_start_ms_ - image_cmd_sent_ms_;
        uint32_t total_ms = now_ms - image_cmd_sent_ms_;
        image_rx_transfer_ms_ = transfer_ms;
        image_rx_done_ms_ = now_ms;
        ESP_LOGI(TAG, "RX complete: session=%u | prepare=%lums transfer=%lums total=%lums",
                 session_id,
                 static_cast<unsigned long>(prepare_ms),
                 static_cast<unsigned long>(transfer_ms),
                 static_cast<unsigned long>(total_ms));
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
        image_rx_pending_ = false;
    } else {
        ESP_LOGW(TAG, "image RX: sent ACK with %u missing, waiting for retransmit", missing_count);
        if (image_rx_eot_count_ == 1) {
            for (uint16_t i = 0; i < missing_count; i += 16) {
                char line[128];
                int pos = 0;
                for (uint16_t j = i; j < missing_count && j < i + 16; j++) {
                    pos += snprintf(line + pos, sizeof(line) - pos, "%u ", missing_indices[j]);
                }
                ESP_LOGW(TAG, "  missing: %s", line);
            }
        }
        image_rx_last_frag_ms_ = smtc_modem_hal_get_time_in_ms();
    }
}

void RadioPing::check_image_rx_timeout()
{
    if (!image_rx_pending_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (image_xfer_.rx_complete() &&
        now - image_rx_last_frag_ms_ < APP_IMAGE_RX_TIMEOUT_MS) {
        return;
    }
    if (image_xfer_.rx_complete()) {
        if (image_rx_expected_crc32_ != 0) {
            uint32_t actual_crc32 = image_xfer_.rx_crc32();
            if (actual_crc32 != image_rx_expected_crc32_) {
                ESP_LOGW(TAG, "image RX complete timeout crc32 mismatch: expected=0x%08lx actual=0x%08lx",
                         static_cast<unsigned long>(image_rx_expected_crc32_),
                         static_cast<unsigned long>(actual_crc32));
                image_xfer_.rx_begin(image_xfer_.rx_session_id(), image_xfer_.rx_total_count());
                if (!image_xfer_.rx_active()) {
                    image_rx_pending_ = false;
                    if (image_rx_error_cb_) {
                        image_rx_error_cb_(ImageRxError::no_memory);
                    }
                    return;
                }
                image_rx_last_frag_ms_ = now;
                image_rx_last_progress_ms_ = 0;
                if (image_rx_error_cb_) {
                    image_rx_error_cb_(ImageRxError::crc_mismatch);
                }
                return;
            }
        }
        image_rx_done_session_ = image_xfer_.rx_session_id();
        // ESP_LOGI(TAG, "image RX complete (no EOT seen before timeout)");
        if (image_rx_complete_cb_) {
            image_rx_complete_cb_(&image_xfer_);
        }
        image_rx_pending_ = false;
        return;
    }

    if (now - image_rx_last_frag_ms_ < 5000U) {
        return;
    }

    // ESP_LOGW(TAG, "image RX: timeout (5s no activity), giving up. %u/%u received",
    //          image_xfer_.rx_received_count(), image_xfer_.rx_total_count());
    image_rx_pending_ = false;
    if (image_rx_error_cb_) {
        image_rx_error_cb_(ImageRxError::timeout);
    }
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
}

// Perform UI-requested image RX teardown in the radio task.
void RadioPing::check_image_rx_abort()
{
    if (!image_rx_abort_req_) return;
    image_rx_abort_req_ = false;

    if (image_rx_pending_ || image_req_active_) {
        ESP_LOGI(TAG, "image RX aborted by user (left transfer page)");
    }
    image_req_active_ = false;
    image_rx_pending_ = false;
    image_xfer_.rx_reset();
    image_rx_nack_sent_ = 0;
    schedule_rx();
}

bool RadioPing::send_config(uint8_t key, uint32_t value)
{
    ESP_LOGI(TAG, "send_config: key=%u value=%lu", key, static_cast<unsigned long>(value));

    suspended_ = true;

    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfig;
    pkt[5] = 1;
    const uint16_t transaction_id = 0;
    put_u16_le(&pkt[6], transaction_id);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    // Wake a CAD-sleeping node before config TX, including low-power state changes.
    if (g_low_power_enabled || key == APP_CFG_KEY_LOW_POWER) {
        uint32_t t0 = smtc_modem_hal_get_time_in_ms();
        if (!send_lora_wakeup()) {
            ESP_LOGE(TAG, "LoRa wakeup failed for config");
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return false;
        }
        uint32_t elapsed = smtc_modem_hal_get_time_in_ms() - t0;
        ESP_LOGI(TAG, "LoRa wakeup preamble TX took %lu ms (config)", (unsigned long)elapsed);
        configure_flrc();
    }

    for (int attempt = 0; attempt < 3; attempt++) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();

        config_ack_received_ = false;
        config_ack_key_ = 0;
        config_ack_value_ = 0;
        config_ack_transaction_id_ = 0;
        send_single_packet(pkt, kHeaderSize);
        schedule_rx();

        uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
        while (!config_ack_received_) {
            if (smtc_modem_hal_get_time_in_ms() - wait_start > kConfigAckTimeoutMs) {
                break;
            }
            if (irq_pending_) {
                irq_pending_ = false;
                ral_irq_t irq = RAL_IRQ_NONE;
                smtc_modem_hal_protect_api_call();
                ral_status_t s = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                smtc_modem_hal_unprotect_api_call();
                if (s == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
                    handle_irq(irq);
                }
            }
            taskYIELD();
        }

        if (config_ack_received_ &&
            config_ack_key_ == key && config_ack_value_ == value &&
            config_ack_transaction_id_ == transaction_id) {
            ESP_LOGI(TAG, "send_config: ACK received on attempt %d", attempt + 1);
            suspended_ = false;
            if (!ptt_active_) schedule_rx();
            return true;
        }
        ESP_LOGW(TAG, "send_config: no ACK, attempt %d/3", attempt + 1);
    }

    ESP_LOGW(TAG, "send_config: failed after 3 attempts");
    suspended_ = false;
    if (!ptt_active_) schedule_rx();
    return false;
}

bool RadioPing::change_frequency(uint32_t frequency_hz)
{
    if (!is_gateway_ || frequency_change_active_ || image_busy() ||
        !is_frequency_preset(frequency_hz) || frequency_hz == current_frequency_hz_) {
        ESP_LOGW(TAG, "frequency change rejected: gateway=%d busy=%d current=%lu requested=%lu",
                 is_gateway_, frequency_change_active_ || image_busy(),
                 static_cast<unsigned long>(current_frequency_hz_),
                 static_cast<unsigned long>(frequency_hz));
        return false;
    }

    frequency_change_active_ = true;
    suspended_ = true;
    const uint32_t previous_hz = current_frequency_hz_;
    uint16_t transaction_id = ++frequency_transaction_id_;
    if (transaction_id == 0) {
        frequency_transaction_id_ = 1;
        transaction_id = 1;
    }

    uint8_t config_pkt[kHeaderSize] = {};
    std::memcpy(config_pkt, kMagic, sizeof(kMagic));
    config_pkt[4] = kPacketTypeConfig;
    config_pkt[5] = 1;
    put_u16_le(&config_pkt[6], transaction_id);
    config_pkt[8] = APP_CFG_KEY_FREQUENCY;
    put_u32_le(&config_pkt[9], frequency_hz);

    if (g_low_power_enabled) {
        if (!send_lora_wakeup()) {
            suspended_ = false;
            frequency_change_active_ = false;
            if (!ptt_active_) schedule_rx();
            return false;
        }
        (void)configure_flrc();
    }

    bool config_ok = false;
    for (uint32_t attempt = 0; attempt < APP_FREQUENCY_CONFIRM_RETRIES; ++attempt) {
        ESP_LOGW(TAG, "TX frequency Config: tx=%u attempt=%lu hz=%lu",
                 transaction_id, static_cast<unsigned long>(attempt + 1U),
                 static_cast<unsigned long>(frequency_hz));
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();

        config_ack_received_ = false;
        config_ack_key_ = 0;
        config_ack_value_ = 0;
        config_ack_transaction_id_ = 0;
        (void)send_single_packet(config_pkt, kHeaderSize);
        schedule_rx();

        uint32_t wait_start = smtc_modem_hal_get_time_in_ms();
        while (!config_ack_received_ &&
               smtc_modem_hal_get_time_in_ms() - wait_start <=
                   APP_FREQUENCY_CONFIRM_TIMEOUT_MS) {
            if (irq_pending_) {
                irq_pending_ = false;
                ral_irq_t irq = RAL_IRQ_NONE;
                smtc_modem_hal_protect_api_call();
                ral_status_t status = ral_get_and_clear_irq_status(&radio_.ral, &irq);
                smtc_modem_hal_unprotect_api_call();
                if (status == RAL_STATUS_OK && irq != RAL_IRQ_NONE) handle_irq(irq);
            }
            taskYIELD();
        }
        if (config_ack_received_ && config_ack_key_ == APP_CFG_KEY_FREQUENCY &&
            config_ack_value_ == frequency_hz &&
            config_ack_transaction_id_ == transaction_id) {
            ESP_LOGW(TAG, "RX frequency ConfigAck matched: tx=%u hz=%lu",
                     transaction_id, static_cast<unsigned long>(frequency_hz));
            config_ok = true;
            break;
        }
        ESP_LOGW(TAG, "frequency ConfigAck timeout/mismatch: tx=%u attempt=%lu",
                 transaction_id, static_cast<unsigned long>(attempt + 1U));
    }

    if (config_ok) {
        // The node sends several ACK copies on the old channel before applying
        // the new channel. Keep listening briefly so it can finish that sequence
        // before the gateway transmits confirmations on the new channel.
        vTaskDelay(ms_to_ticks_min_1(APP_FREQUENCY_ACK_SETTLE_MS));
    }

    if (!config_ok || !apply_frequency(frequency_hz)) {
        (void)apply_frequency(previous_hz);
        suspended_ = false;
        frequency_change_active_ = false;
        if (!ptt_active_) schedule_rx();
        ESP_LOGW(TAG, "frequency change failed: tx=%u", transaction_id);
        return false;
    }

    uint8_t confirm_pkt[kHeaderSize] = {};
    std::memcpy(confirm_pkt, kMagic, sizeof(kMagic));
    confirm_pkt[4] = kPacketTypeFrequencyConfirm;
    confirm_pkt[5] = 1;
    put_u16_le(&confirm_pkt[6], transaction_id);
    confirm_pkt[8] = APP_CFG_KEY_FREQUENCY;
    put_u32_le(&confirm_pkt[9], frequency_hz);

    bool confirm_sent = false;
    for (uint32_t attempt = 0; attempt < APP_FREQUENCY_CONFIRM_RETRIES; ++attempt) {
        ESP_LOGW(TAG, "TX frequency Confirm: tx=%u attempt=%lu hz=%lu",
                 transaction_id, static_cast<unsigned long>(attempt + 1U),
                 static_cast<unsigned long>(frequency_hz));
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();
        confirm_sent = send_single_packet(confirm_pkt, kHeaderSize) || confirm_sent;
        schedule_rx();
        vTaskDelay(ms_to_ticks_min_1(20));
    }

    if (!confirm_sent) {
        (void)apply_frequency(previous_hz);
    }
    suspended_ = false;
    frequency_change_active_ = false;
    if (!ptt_active_) schedule_rx();
    ESP_LOGI(TAG, "frequency change %s: tx=%u old=%lu new=%lu",
             confirm_sent ? "complete" : "failed", transaction_id,
             static_cast<unsigned long>(previous_hz),
             static_cast<unsigned long>(frequency_hz));
    return confirm_sent;
}

bool RadioPing::send_config_ack(uint8_t key, uint32_t value, uint16_t transaction_id)
{
    uint8_t pkt[kHeaderSize];
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeConfigAck;
    pkt[5] = 1;
    put_u16_le(&pkt[6], transaction_id);
    pkt[8] = key;
    put_u32_le(&pkt[9], value);
    pkt[13] = 0;

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending) {
        (void)ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        (void)ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    bool sent = send_single_packet(pkt, kHeaderSize);

    if (!ptt_active_) {
        schedule_rx();
    }
    return sent;
}

void RadioPing::handle_frequency_config(uint16_t transaction_id, uint32_t frequency_hz)
{
    if (is_gateway_ || !is_frequency_preset(frequency_hz) ||
        image_busy() || ptt_active_) {
        ESP_LOGW(TAG, "frequency config rejected: tx=%u hz=%lu busy=%d",
                 transaction_id, static_cast<unsigned long>(frequency_hz), image_busy());
        return;
    }

    if (frequency_rollback_active_ &&
        transaction_id == frequency_transaction_id_ &&
        frequency_hz == frequency_pending_hz_) {
        ESP_LOGI(TAG, "frequency config duplicate on pending channel: tx=%u", transaction_id);
        return;
    }

    bool ack_sent = false;
    for (uint32_t attempt = 0; attempt < APP_FREQUENCY_CONFIRM_RETRIES; ++attempt) {
        ack_sent = send_config_ack(APP_CFG_KEY_FREQUENCY, frequency_hz, transaction_id) || ack_sent;
    }
    if (!ack_sent) return;

    frequency_previous_hz_ = current_frequency_hz_;
    frequency_pending_hz_ = frequency_hz;
    frequency_transaction_id_ = transaction_id;
    if (!apply_frequency(frequency_hz)) {
        current_frequency_hz_ = frequency_previous_hz_;
        (void)apply_frequency(frequency_previous_hz_);
        return;
    }

    frequency_rollback_deadline_ms_ =
        smtc_modem_hal_get_time_in_ms() + APP_FREQUENCY_ROLLBACK_MS;
    frequency_rollback_active_ = true;
    cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
    schedule_rx();
    ESP_LOGW(TAG, "frequency pending confirm: tx=%u old=%lu new=%lu",
             transaction_id, static_cast<unsigned long>(frequency_previous_hz_),
             static_cast<unsigned long>(frequency_pending_hz_));
}

void RadioPing::handle_frequency_confirm(uint16_t transaction_id, uint32_t frequency_hz)
{
    if (is_gateway_ || !frequency_rollback_active_ ||
        transaction_id != frequency_transaction_id_ ||
        frequency_hz != frequency_pending_hz_) {
        ESP_LOGW(TAG, "ignore unmatched frequency confirm: tx=%u hz=%lu",
                 transaction_id, static_cast<unsigned long>(frequency_hz));
        return;
    }

    frequency_rollback_active_ = false;
    frequency_rollback_deadline_ms_ = 0;
    frequency_previous_hz_ = frequency_hz;
    if (frequency_committed_cb_) frequency_committed_cb_(frequency_hz);
    ESP_LOGW(TAG, "frequency confirmed: tx=%u hz=%lu",
             transaction_id, static_cast<unsigned long>(frequency_hz));
}

void RadioPing::check_frequency_rollback()
{
    if (!frequency_rollback_active_ || is_gateway_) return;
    uint32_t now = smtc_modem_hal_get_time_in_ms();
    if (static_cast<int32_t>(now - frequency_rollback_deadline_ms_) < 0) return;

    uint32_t rollback_hz = frequency_previous_hz_;
    frequency_rollback_active_ = false;
    frequency_rollback_deadline_ms_ = 0;
    ESP_LOGW(TAG, "frequency confirm timeout, rolling back to %lu Hz",
             static_cast<unsigned long>(rollback_hz));
    (void)apply_frequency(rollback_hz);
    schedule_rx();
}

size_t RadioPing::snapshot_audio(int16_t *out, size_t max_samples)
{
    return audio_ringbuf_.snapshot(out, max_samples);
}

size_t RadioPing::snapshot_opus(uint8_t *out, size_t max_bytes)
{
    return opus_ringbuf_.snapshot(out, max_bytes);
}

bool RadioPing::configure_lora_cad()
{
    const void *ctx = radio_.ral.context;

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, current_frequency_hz_);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    if (lr20xx_radio_lora_set_modulation_params(ctx, &mod) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora mod params failed");
        return false;
    }

    lr20xx_radio_lora_cad_params_t cad = {};
    cad.cad_symb_nb = 2;
    cad.pnr_delta = 0;
    cad.cad_exit_mode = LR20XX_RADIO_LORA_CAD_EXIT_MODE_STANDBYRC;
    cad.cad_timeout_in_pll_step = 0;
    cad.cad_detect_peak = 56;
    if (lr20xx_radio_lora_configure_cad_params(ctx, &cad) != LR20XX_STATUS_OK) {
        ESP_LOGE(TAG, "lora cad params failed");
        return false;
    }

    return true;
}

bool RadioPing::low_power_sleep(uint32_t ms)
{
    // Flush the console so the last log line isn't truncated when clocks stop.
    // Note: the USB Serial/JTAG console does not survive light sleep, so serial
    // output stops during CAD standby — this is expected. Run on battery/adapter.
    fflush(stdout);

    esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000ULL);

    // ESP32-S3 light-sleep wake requires a level interrupt.
    // Arm PIR wake only outside the cooldown while the high-level ISR is enabled.
    bool pir_wake = pir_enabled_ && pir_armed_;
    if (pir_wake) {
        gpio_wakeup_enable(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }

    esp_light_sleep_start();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool woken_by_pir = false;

    if (pir_wake) {
        gpio_wakeup_disable(APP_PIR_GPIO);
        if (cause == ESP_SLEEP_WAKEUP_GPIO) {
            // The HIGH_LEVEL GPIO ISR fires on resume and handles the trigger
            // (disable source, set pir_triggered_, start 15s re-arm). We just
            // report the PIR wake so the caller keeps the node awake to push.
            woken_by_pir = true;
            ESP_LOGI(TAG, "light sleep: woken by PIR");
        }
    }
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGD(TAG, "light sleep: timer wake");
    }

    return woken_by_pir;
}

void RadioPing::notify_capture_starting()
{
    // Only meaningful for a low-power node (the gateway never CAD-sleeps). Set
    // the keep-awake guard so the next poll_once idle pass does NOT drop into
    // CAD light sleep while the capture + push runs.
    if (!g_low_power_enabled || is_gateway_) return;
    pir_push_wake_ = true;
    pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
}

void RadioPing::enter_low_power_cad()
{
    if (mode_ != Mode::idle) return;

    const void *ctx = radio_.ral.context;

    if (!low_power_cad_active_) {
        smtc_modem_hal_protect_api_call();
        if (mode_ == Mode::rx_pending) {
            ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
            ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
            mode_ = Mode::idle;
        }
        smtc_modem_hal_unprotect_api_call();
        low_power_cad_active_ = true;
        ESP_LOGI(TAG, "entering low power CAD mode");
        // Release power-hungry peripherals (camera DVP). capture_frame() rebuilds
        // the camera on demand, so no explicit restore is needed on wake.
        if (low_power_standby_cb_) {
            low_power_standby_cb_(true);
        }
    }

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();

    if (!configure_lora_cad()) {
        smtc_modem_hal_unprotect_api_call();
        ESP_LOGE(TAG, "CAD config failed, fallback to FLRC RX");
        low_power_cad_active_ = false;
        configure_flrc();
        schedule_rx();
        return;
    }

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK);
    lr20xx_radio_lora_set_cad(ctx);
    smtc_modem_hal_unprotect_api_call();

    cad_pending_ms_ = smtc_modem_hal_get_time_in_ms();
    mode_ = Mode::cad_pending;
}

void RadioPing::handle_cad_irq(ral_irq_t irq)
{
    mode_ = Mode::idle;
    cad_pending_ms_ = 0;

    if ((irq & RAL_IRQ_CAD_DONE) == 0) {
        ESP_LOGW(TAG, "CAD unexpected irq=0x%08lx", static_cast<unsigned long>(irq));
        return;
    }

    ESP_LOGI(TAG, "CAD done: %s", (irq & RAL_IRQ_CAD_OK) ? "activity detected" : "channel clear");

    if ((irq & RAL_IRQ_CAD_OK) != 0) {
        ESP_LOGI(TAG, "CAD detected activity, switching to FLRC RX (activity-refreshed window)");
        low_power_cad_active_ = false;
        cad_wakeup_ms_ = smtc_modem_hal_get_time_in_ms();
        smtc_modem_hal_protect_api_call();
        configure_flrc();
        smtc_modem_hal_unprotect_api_call();
        schedule_rx();
    } else {
        const void *ctx = radio_.ral.context;
        lr20xx_system_sleep_cfg_t sleep_cfg = {};
        sleep_cfg.is_clk_32k_enabled = 1;
        sleep_cfg.is_ram_retention_enabled = 1;
        smtc_modem_hal_protect_api_call();
        lr20xx_system_set_sleep_mode(ctx, &sleep_cfg, 0);
        smtc_modem_hal_unprotect_api_call();

        // LR2021 is now asleep and SPI is idle, so light-sleep the ESP32 too
        // for the 500ms CAD off-period. Wakes on timer (next CAD) or PIR.
        bool woken_by_pir = low_power_sleep(500);

        if (woken_by_pir) {
            // A PIR wake starts a transmit path, so leave the radio asleep until image_tx_task owns it.
            // The keep-awake guard prevents CAD sleep while capture starts.
            pir_push_wake_ = true;
            pir_push_wake_ms_ = smtc_modem_hal_get_time_in_ms();
            ESP_LOGI(TAG, "PIR wake: staying awake, capture will push image");
        }
    }
}

bool RadioPing::send_lora_wakeup()
{
    const void *ctx = radio_.ral.context;

    ESP_LOGI(TAG, "sending LoRa wakeup (508 symbol preamble)");

    smtc_modem_hal_protect_api_call();
    if (mode_ == Mode::rx_pending || mode_ == Mode::cad_pending) {
        ral_set_standby(&radio_.ral, RAL_STANDBY_CFG_XOSC);
        ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
        cad_pending_ms_ = 0;
        mode_ = Mode::idle;
    }
    smtc_modem_hal_unprotect_api_call();

    smtc_modem_hal_protect_api_call();
    smtc_modem_hal_start_radio_tcxo();
    smtc_modem_hal_set_ant_switch(true);

    lr20xx_radio_common_set_pkt_type(ctx, LR20XX_RADIO_COMMON_PKT_TYPE_LORA);
    lr20xx_radio_common_set_rf_freq(ctx, current_frequency_hz_);

    lr20xx_radio_lora_mod_params_t mod = {};
    mod.sf = LR20XX_RADIO_LORA_SF7;
    mod.bw = LR20XX_RADIO_LORA_BW_125;
    mod.cr = LR20XX_RADIO_LORA_CR_4_5;
    mod.ppm = LR20XX_RADIO_LORA_NO_PPM;
    lr20xx_radio_lora_set_modulation_params(ctx, &mod);

    lr20xx_radio_lora_pkt_params_t pkt = {};
    pkt.preamble_len_in_symb = 508;
    pkt.pkt_mode = LR20XX_RADIO_LORA_PKT_EXPLICIT;
    pkt.pld_len_in_bytes = 4;
    pkt.crc = LR20XX_RADIO_LORA_CRC_ENABLED;
    pkt.iq = LR20XX_RADIO_LORA_IQ_STANDARD;
    lr20xx_radio_lora_set_packet_params(ctx, &pkt);

    uint8_t dummy[4] = {0xCA, 0xFE, 0x00, 0x01};
    lr20xx_radio_fifo_write_tx(ctx, dummy, 4);

    ral_set_dio_irq_params(&radio_.ral, RAL_IRQ_TX_DONE);
    ral_clear_irq_status(&radio_.ral, RAL_IRQ_ALL);
    lr20xx_radio_common_set_tx(ctx, 2000);
    smtc_modem_hal_unprotect_api_call();

    mode_ = Mode::tx_pending;
    bool ok = wait_for_tx_done(1500);
    if (!ok) {
        ESP_LOGE(TAG, "LoRa wakeup TX timeout");
    } else {
        ESP_LOGI(TAG, "LoRa wakeup sent");
    }
    return ok;
}

void RadioPing::send_vbat_broadcast()
{
    // Battery packet: 14-byte header, 2-byte millivolt value, and CRC32 over both.
    uint8_t pkt[kHeaderSize + 6];
    std::memset(pkt, 0, sizeof(pkt));
    std::memcpy(pkt, kMagic, sizeof(kMagic));
    pkt[4] = kPacketTypeVbat;
    put_u16_le(&pkt[14], bsp_vbat_get_cached());
    put_u32_le(&pkt[16], crc32_ieee(pkt, 16));

    send_single_packet(pkt, kHeaderSize + 6);
    ESP_LOGI(TAG, "vbat broadcast sent: %u mV", bsp_vbat_get_cached());
}

void RadioPing::vbat_maintenance_tick()
{
    // Gateway never broadcasts its voltage, only receives from nodes.
    if (is_gateway_) return;

    uint32_t now = smtc_modem_hal_get_time_in_ms();

    if (g_low_power_enabled) {
        // Refresh cached voltage every 60 seconds without an extra radio wakeup.
        if ((int32_t)(now - vbat_last_sample_ms_) >= (int32_t)kVbatLowPowerSampleIntervalMs) {
            int mv = bsp_vbat_read_mv();
            if (mv >= 0) {
                ESP_LOGD(TAG, "vbat sample: %d mV", mv);
            }
            vbat_last_sample_ms_ = now;
        }
        return;
    }

    // Non-low-power node: sampling is handled by the bsp_vbat background task
    // (15s). Broadcast the cached voltage every 5 minutes (radio is already in
    // FLRC RX here, so send_single_packet is safe).
    if ((int32_t)(now - vbat_last_broadcast_ms_) >= (int32_t)kVbatBroadcastIntervalMs) {
        send_vbat_broadcast();
        vbat_last_broadcast_ms_ = now;
    }
}
