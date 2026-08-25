#include "audio_diagnostics.hpp"
#include "camera_uart.hpp"
#include "image_transfer.hpp"
#include "radio_ping.hpp"
#include "opus_codec.hpp"
#include "ui_gateway.h"
#include "wifi_manager.h"
#include "image_store.h"
#include "captive_portal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_app_desc.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_jpeg_common.h"
#include "esp_jpeg_dec.h"

#include <stdio.h>
#include <new>

#include "app_config.h"
#include "bsp.h"

#if APP_VOICE_ALARM_ENABLE
#include "warning_voice_opus.h"
#endif

volatile bool g_low_power_enabled = false;

namespace {
constexpr const char *TAG = "app";
constexpr const char *kNvsNs = "app";
constexpr const char *kModeKey = "mode";
constexpr const char *kFrequencyIndexKey = "freq_idx";
constexpr uint32_t kFrequencyPresetsHz[APP_FLRC_FREQUENCY_PRESET_COUNT] =
    APP_FLRC_FREQUENCY_PRESETS_HZ;

uint32_t s_last_gw_capture_ms = 0;

enum class AppMode : uint8_t {
    camera = 0,
    radio = 1,
};

AudioDiagnostics g_audio;
CameraUartStreamer g_camera_uart;
RadioPing g_radio;
volatile bool g_capture_busy = false;
AppMode g_app_mode = AppMode::camera;
bool g_radio_active = false;

struct ImageRxWork {
    uint8_t *raw;
    size_t raw_len;
    uint16_t session_id;
    uint32_t transfer_ms;
    int64_t queued_at_us;
};
QueueHandle_t g_image_rx_work_queue = nullptr;
volatile bool g_image_rx_processing = false;
volatile bool g_image_ui_pending = false;
volatile uint16_t g_image_ui_pending_session = 0;

// Auto-capture timer state
esp_timer_handle_t g_auto_capture_timer = nullptr;
esp_timer_handle_t g_countdown_timer = nullptr;
uint32_t g_capture_interval_sec = APP_AUTO_CAPTURE_DEFAULT_SEC;
bool g_audio_clip_enabled = APP_AUDIO_CLIP_DEFAULT_ENABLE;
volatile bool g_voice_alarm_enabled = false;
uint16_t g_auto_session_id = 0x8000;
int64_t g_last_capture_time_us = 0;

// K6 short/long press state
int64_t g_ptt_press_time_us = 0;
bool g_ptt_held_long = false;
esp_timer_handle_t g_cooldown_retry_timer = nullptr;
esp_timer_handle_t g_ptt_timer = nullptr;

const char *mode_name(AppMode mode)
{
    return mode == AppMode::radio ? "radio" : "camera";
}

const char *short_error_name(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_NO_MEM:
        return "NO_MEM";
    case ESP_ERR_TIMEOUT:
        return "TIMEOUT";
    case ESP_ERR_INVALID_SIZE:
        return "BAD_SIZE";
    case ESP_ERR_NOT_SUPPORTED:
        return "NOT_SUP";
    default:
        return nullptr;
    }
}

void init_nvs()
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
}

AppMode load_app_mode()
{
    nvs_handle_t nvs;
    uint8_t value = static_cast<uint8_t>(AppMode::camera);
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, kModeKey, &value);
        nvs_close(nvs);
    }
    return value == static_cast<uint8_t>(AppMode::radio) ? AppMode::radio : AppMode::camera;
}

void save_app_mode(AppMode mode)
{
    nvs_handle_t nvs;
    esp_err_t e = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "open mode nvs: %s", esp_err_to_name(e));
        return;
    }
    e = nvs_set_u8(nvs, kModeKey, static_cast<uint8_t>(mode));
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "save mode nvs: %s", esp_err_to_name(e));
    }
}

uint32_t load_capture_interval()
{
    nvs_handle_t nvs;
    uint32_t val = APP_AUTO_CAPTURE_DEFAULT_SEC;
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u32(nvs, "interval", &val);
        nvs_close(nvs);
    }
    return val;
}

void save_capture_interval(uint32_t sec)
{
    nvs_handle_t nvs;
    esp_err_t e = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    if (e != ESP_OK) return;
    e = nvs_set_u32(nvs, "interval", sec);
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
}

void save_config_u8(const char *key, uint8_t val)
{
    nvs_handle_t nvs;
    if (nvs_open(kNvsNs, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_u8(nvs, key, val);
    nvs_commit(nvs);
    nvs_close(nvs);
}

uint8_t load_config_u8(const char *key, uint8_t def)
{
    nvs_handle_t nvs;
    uint8_t val = def;
    if (nvs_open(kNvsNs, NVS_READONLY, &nvs) == ESP_OK) {
        (void)nvs_get_u8(nvs, key, &val);
        nvs_close(nvs);
    }
    return val;
}

uint8_t frequency_index_from_hz(uint32_t frequency_hz)
{
    for (uint8_t i = 0; i < APP_FLRC_FREQUENCY_PRESET_COUNT; ++i) {
        if (kFrequencyPresetsHz[i] == frequency_hz) return i;
    }
    return 0;
}

uint8_t load_frequency_index()
{
    uint8_t index = load_config_u8(kFrequencyIndexKey, 0);
    return index < APP_FLRC_FREQUENCY_PRESET_COUNT ? index : 0;
}

bool save_frequency_index(uint8_t index)
{
    if (index >= APP_FLRC_FREQUENCY_PRESET_COUNT) return false;

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(kNvsNs, NVS_READWRITE, &nvs);
    bool opened = (err == ESP_OK);
    if (err == ESP_OK) err = nvs_set_u8(nvs, kFrequencyIndexKey, index);
    if (err == ESP_OK) err = nvs_commit(nvs);
    if (opened) nvs_close(nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save frequency preset to NVS failed: %s",
                 esp_err_to_name(err));
        return false;
    }
    return true;
}

void on_frequency_committed(uint32_t frequency_hz)
{
    uint8_t index = frequency_index_from_hz(frequency_hz);
    if (save_frequency_index(index)) {
        ESP_LOGI(TAG, "frequency preset committed to NVS: index=%u hz=%lu",
                 index, static_cast<unsigned long>(frequency_hz));
    }
}

ImageCmdAckStatus on_image_capture_request(uint16_t session_id);

void auto_capture_timer_cb(void *arg)
{
    (void)arg;
    if (g_app_mode != AppMode::camera) return;
    if (g_capture_busy) {
        ESP_LOGW(TAG, "auto-capture skipped: busy");
        return;
    }
    g_last_capture_time_us = esp_timer_get_time();
    uint16_t sid = g_auto_session_id++;
    if (g_auto_session_id == 0) g_auto_session_id = 0x8000;
    ESP_LOGI(TAG, "auto-capture trigger: session=%u interval=%lus",
             sid, static_cast<unsigned long>(g_capture_interval_sec));
    on_image_capture_request(sid);
}

void start_auto_capture_timer()
{
    if (g_auto_capture_timer) {
        esp_timer_stop(g_auto_capture_timer);
    }
    if (g_capture_interval_sec == 0) {
        ESP_LOGI(TAG, "auto-capture disabled");
        return;
    }
    uint64_t period_us = static_cast<uint64_t>(g_capture_interval_sec) * 1000000ULL;
    if (!g_auto_capture_timer) {
        const esp_timer_create_args_t args = {
            .callback = auto_capture_timer_cb,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "auto_cap",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&args, &g_auto_capture_timer);
    }
    esp_timer_start_periodic(g_auto_capture_timer, period_us);
    ESP_LOGI(TAG, "auto-capture timer started: %lus", static_cast<unsigned long>(g_capture_interval_sec));
}

void countdown_timer_cb(void *arg)
{
    (void)arg;
    if (g_app_mode != AppMode::camera) return;
    if (g_capture_busy) return;

    char buf[48];
    if (g_capture_interval_sec == 0) {
        snprintf(buf, sizeof(buf), "Auto: Off");
    } else {
        int64_t elapsed_us = esp_timer_get_time() - g_last_capture_time_us;
        int32_t remaining = static_cast<int32_t>(g_capture_interval_sec) -
                            static_cast<int32_t>(elapsed_us / 1000000LL);
        if (remaining < 0) remaining = 0;

        const char *unit;
        uint32_t val;
        if (g_capture_interval_sec >= 3600) {
            unit = "h"; val = g_capture_interval_sec / 3600;
        } else if (g_capture_interval_sec >= 60) {
            unit = "min"; val = g_capture_interval_sec / 60;
        } else {
            unit = "s"; val = g_capture_interval_sec;
        }
        snprintf(buf, sizeof(buf), "%lu%s | %lds", static_cast<unsigned long>(val), unit,
                 static_cast<long>(remaining));
    }
    if (g_audio_clip_enabled) {
        strncat(buf, " radio", sizeof(buf) - strlen(buf) - 1);
    }
    bsp_lcd_set_camera_status(buf);
}

void update_camera_timer_status()
{
    g_last_capture_time_us = esp_timer_get_time();
    countdown_timer_cb(nullptr);
}

void start_countdown_timer()
{
    if (g_countdown_timer) return;
    const esp_timer_create_args_t args = {
        .callback = countdown_timer_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "countdown",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &g_countdown_timer);
    esp_timer_start_periodic(g_countdown_timer, 1000000ULL);
}

// Use esp_timer for the 15-second PIR rearm because FreeRTOS ticks stop during light sleep.
static esp_timer_handle_t g_pir_rearm_timer = nullptr;

// Use a high-level PIR interrupt for light-sleep wake.
// Disable it in the ISR while the pin is high; the timer rearms it after the cooldown.
static void IRAM_ATTR pir_isr_handler(void *arg)
{
    // Kill the level trigger NOW so it can't re-fire while GPIO12 stays high.
    gpio_intr_disable(APP_PIR_GPIO);
    static_cast<RadioPing *>(arg)->set_pir_armed(false);
    static_cast<RadioPing *>(arg)->pir_trigger();
    if (g_pir_rearm_timer != nullptr) {
        // The disabled interrupt prevents reentry; stopping the timer from this ISR is unsafe.
        esp_timer_start_once(g_pir_rearm_timer,
                             (uint64_t)APP_TRIGGER_COOLDOWN_SEC * 1000000ULL);
    }
}

// Re-arm the PIR level trigger 15s after the last detection.
static void pir_rearm_timer_cb(void *arg)
{
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level re-armed", APP_PIR_GPIO);
}

void pir_arm_timer_cb(void *arg)
{
    if (g_pir_rearm_timer == nullptr) {
        const esp_timer_create_args_t rearm_args = {
            .callback = pir_rearm_timer_cb,
            .arg = arg,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "pir_rearm",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&rearm_args, &g_pir_rearm_timer);
    }
    gpio_intr_disable(APP_PIR_GPIO);
    gpio_isr_handler_add(APP_PIR_GPIO, pir_isr_handler, arg);
    gpio_set_intr_type(APP_PIR_GPIO, GPIO_INTR_HIGH_LEVEL);
    static_cast<RadioPing *>(arg)->set_pir_armed(true);
    gpio_intr_enable(APP_PIR_GPIO);
    ESP_LOGI(TAG, "PIR: GPIO%d high-level armed", APP_PIR_GPIO);
}

void on_config_received(uint8_t key, uint32_t value)
{
    if (key == APP_CFG_KEY_INTERVAL) {
        if (value != 0 && value < APP_AUTO_CAPTURE_MIN_SEC) {
            value = APP_AUTO_CAPTURE_MIN_SEC;
        }
        g_capture_interval_sec = value;
        save_capture_interval(value);
        start_auto_capture_timer();
        update_camera_timer_status();
        ESP_LOGI(TAG, "config: interval=%lus", static_cast<unsigned long>(value));
    } else if (key == APP_CFG_KEY_INTER_PACKET) {
        g_radio.set_inter_packet_us(value);
        ESP_LOGI(TAG, "config: inter_packet=%luus", static_cast<unsigned long>(value));
        char buf[16];
        snprintf(buf, sizeof(buf), "%luus", static_cast<unsigned long>(value));
        bsp_lcd_set_camera_status(buf);
    } else if (key == APP_CFG_KEY_AUDIO_CLIP) {
        g_audio_clip_enabled = (value != 0);
        save_config_u8("audio_clip", value ? 1 : 0);
        update_camera_timer_status();
        ESP_LOGI(TAG, "config: audio_clip=%s", g_audio_clip_enabled ? "on" : "off");
    } else if (key == APP_CFG_KEY_SOUND_TRIGGER) {
        g_radio.set_sound_trigger_level(value);
        save_config_u8("snd_trig", (uint8_t)value);
        ESP_LOGI(TAG, "config: sound_trigger=%lu", static_cast<unsigned long>(value));
    } else if (key == APP_CFG_KEY_PIR_TRIGGER) {
        g_radio.set_pir_enabled(value != 0);
        save_config_u8("pir", value ? 1 : 0);
        ESP_LOGI(TAG, "config: pir_trigger=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_VOICE_ALARM) {
        g_voice_alarm_enabled = (value != 0);
        save_config_u8("alarm", value ? 1 : 0);
        ESP_LOGI(TAG, "config: voice_alarm=%s", value ? "on" : "off");
    } else if (key == APP_CFG_KEY_LOW_POWER) {
        g_low_power_enabled = (value != 0);
        save_config_u8("lowpwr", value ? 1 : 0);
        ESP_LOGI(TAG, "config: low_power=%s", value ? "on" : "off");
        if (value != 0) {
            // Low power sleeps the CPU during CAD standby, so sound trigger and
            // audio clip can't work. Zero them (and persist) so they stay off
            // even after low power is turned off, matching the gateway UI.
            g_audio_clip_enabled = false;
            save_config_u8("audio_clip", 0);
            g_radio.set_sound_trigger_level(0);
            save_config_u8("snd_trig", 0);
            update_camera_timer_status();
            ESP_LOGI(TAG, "low power: sound trigger + audio clip disabled");
        }
    }
}

// Node low power: called by the radio when entering CAD sleep standby. Release
// the camera to save power; capture_frame() rebuilds it on the next capture.
void on_low_power_standby(bool entering)
{
    if (!entering) return;
    if (g_capture_busy) return;  // never release mid-capture
    g_camera_uart.low_power_standby();
}

void switch_mode_and_restart()
{
    AppMode next = g_app_mode == AppMode::camera ? AppMode::radio : AppMode::camera;
    ESP_LOGW(TAG, "K5 mode switch: %s -> %s, restarting",
             mode_name(g_app_mode), mode_name(next));
    save_app_mode(next);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

// PTT long-press timer callback
void ptt_long_press_cb(void *arg)
{
    (void)arg;
    // Only gateway mode arms this timer now (K6 long-press >1.5s -> mode switch,
    // handled in on_button on release). Camera mode no longer binds K6/PTT, so
    // there is no camera voice activation here anymore.
    g_ptt_held_long = true;
}

void play_audio_clip(const uint8_t *opus_packed, size_t total_len);

// Image capture task: runs on device A (camera mode) when ImageCmd received
struct ImageCaptureCtx {
    uint16_t session_id;
};

void image_capture_task(void *arg)
{
    auto *ctx = static_cast<ImageCaptureCtx *>(arg);
    uint16_t session_id = ctx->session_id;
    delete ctx;

    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

    uint32_t t_cmd = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] cmd received t=0ms");

    bsp_lcd_set_camera_status("Remote capture...");

#if APP_AUDIO_FEATURES_ENABLE
    g_radio.pause_audio_capture();
    vTaskDelay(pdMS_TO_TICKS(APP_AUDIO_FRAME_MS + 5));
#endif

    // Snapshot pre-encoded Opus ring buffer (tx_task already stopped, no race)
    uint8_t *opus_buf = nullptr;
    size_t opus_len = 0;
    if (g_audio_clip_enabled) {
        opus_buf = static_cast<uint8_t *>(
            heap_caps_malloc(APP_AUDIO_CLIP_MAX_OPUS_BYTES, MALLOC_CAP_SPIRAM));
        if (opus_buf) {
            opus_len = g_radio.snapshot_opus(opus_buf, APP_AUDIO_CLIP_MAX_OPUS_BYTES);
            ESP_LOGI(TAG, "opus snapshot: %u bytes", static_cast<unsigned>(opus_len));
        }
    }

#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend for image capture: %s", esp_err_to_name(audio_e));
    }
#endif

    esp_err_t e = ESP_OK;
#if APP_CAMERA_NODE_LCD_ENABLE
    e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for image capture: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
#if APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
        g_radio.resume_audio_capture();
#endif
        heap_caps_free(opus_buf);
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }
#endif

    uint32_t t_cam_start = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] camera init start +%lums", static_cast<unsigned long>(t_cam_start - t_cmd));

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);

    uint32_t t_cam_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] capture done +%lums (camera=%lums) %lux%lu %u bytes",
             static_cast<unsigned long>(t_cam_done - t_cmd),
             static_cast<unsigned long>(t_cam_done - t_cam_start),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned>(len));

    // LCD reinit and audio resume deferred until after transmission

    if (capture_e != ESP_OK || !frame) {
#if APP_CAMERA_NODE_LCD_ENABLE
        bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
#endif
        bsp_lcd_set_camera_status("Capture failed");
        heap_caps_free(opus_buf);
        g_radio.resume_audio_capture();
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // JPEG encode
    uint8_t *jpeg = nullptr;
    size_t jpeg_len = 0;
    uint32_t t_jpeg_start = static_cast<uint32_t>(esp_log_timestamp());
    e = g_radio.image_xfer().encode_frame(frame, len, width, height, pixfmt, &jpeg, &jpeg_len);
    heap_caps_free(frame);
    uint32_t t_jpeg_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] JPEG done +%lums (jpeg=%lums) %u bytes",
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_jpeg_start),
             static_cast<unsigned>(jpeg_len));

    if (e != ESP_OK || !jpeg) {
#if APP_CAMERA_NODE_LCD_ENABLE
        bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
        bsp_audio_resume();
#endif
        bsp_lcd_set_camera_status("JPEG encode failed");
        heap_caps_free(opus_buf);
        g_radio.resume_audio_capture();
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // --- Build blob: [4-byte jpeg_len][jpeg][opus] and send once ---
    bool has_opus = (opus_buf && opus_len > 0);
    uint8_t *blob;
    size_t blob_len;

    if (has_opus) {
        blob_len = 4 + jpeg_len + opus_len;
        blob = static_cast<uint8_t *>(
            heap_caps_malloc(blob_len, MALLOC_CAP_SPIRAM));
        if (!blob) {
            ESP_LOGE(TAG, "blob alloc failed: %u bytes", static_cast<unsigned>(blob_len));
            heap_caps_free(jpeg);
            heap_caps_free(opus_buf);
#if APP_CAMERA_NODE_LCD_ENABLE
            bsp_lcd_reinit_after_camera();
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
            bsp_audio_resume();
#endif
            bsp_lcd_set_camera_status("Alloc failed");
            g_radio.resume_audio_capture();
            g_capture_busy = false;
            vTaskDelete(nullptr);
            return;
        }
        blob[0] = static_cast<uint8_t>(jpeg_len);
        blob[1] = static_cast<uint8_t>(jpeg_len >> 8);
        blob[2] = static_cast<uint8_t>(jpeg_len >> 16);
        blob[3] = static_cast<uint8_t>(jpeg_len >> 24);
        memcpy(&blob[4], jpeg, jpeg_len);
        memcpy(&blob[4 + jpeg_len], opus_buf, opus_len);
    } else {
        blob_len = jpeg_len;
        blob = jpeg;
        jpeg = nullptr;
    }
    heap_caps_free(jpeg);
    heap_caps_free(opus_buf);

    uint16_t total_frags = static_cast<uint16_t>(
        (blob_len + APP_IMAGE_FRAGMENT_DATA_SIZE - 1) / APP_IMAGE_FRAGMENT_DATA_SIZE);
    uint16_t tx_session = has_opus ? (session_id | APP_AUDIO_SESSION_FLAG) : session_id;
    ESP_LOGI(TAG, "sending blob: %u pkts, %u bytes (jpeg=%u opus=%u)",
             total_frags, static_cast<unsigned>(blob_len),
             static_cast<unsigned>(jpeg_len), static_cast<unsigned>(opus_len));

    g_radio.send_image(blob, blob_len, tx_session);

    // send_image owns and frees blob; this wait only sequences post-transfer cleanup.
    bool tx_done = false;
    uint32_t wait_start = xTaskGetTickCount();
    while (xTaskGetTickCount() - wait_start < pdMS_TO_TICKS(30000)) {
        if (!g_radio.image_tx_busy()) {
            tx_done = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    uint32_t t_tx_done = static_cast<uint32_t>(esp_log_timestamp());
    ESP_LOGI(TAG, "[TIMING] total +%lums | img_prep=%lu tx=%lums",
             static_cast<unsigned long>(t_tx_done - t_cmd),
             static_cast<unsigned long>(t_jpeg_done - t_cmd),
             static_cast<unsigned long>(t_tx_done - t_jpeg_done));

    if (!tx_done) {
        // If TX is still active, skip cleanup that could race the transfer task.
        ESP_LOGW(TAG, "image tx still active after 30s; skipping post-tx cleanup (tx task finishes on its own)");
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    // Reinit LCD now that transmission is done
#if APP_CAMERA_NODE_LCD_ENABLE
    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after tx: %s", esp_err_to_name(lcd_e));
    }
#endif
#if APP_CAMERA_NODE_LCD_ENABLE && APP_AUDIO_FEATURES_ENABLE
    bsp_audio_resume();
#endif
    char status[48];
    snprintf(status, sizeof(status), "Done %u pkts", total_frags);
    bsp_lcd_set_camera_status(status);

    g_radio.resume_audio_capture();

#if APP_VOICE_ALARM_ENABLE
    if (g_voice_alarm_enabled && session_id >= 0xC000) {
        ESP_LOGI(TAG, "trigger capture done, playing voice alarm");
        // Keep the node awake during alarm playback, then allow CAD sleep again.
        g_radio.audio_playback_begin();
        uint8_t prev_vol = APP_VOICE_ALARM_VOLUME_PERCENT;
        bsp_audio_set_volume(prev_vol);
        play_audio_clip(warning_voice_opus, warning_voice_opus_len);
        bsp_audio_set_volume(100);
        g_radio.audio_playback_end();
    }
#endif

    g_capture_busy = false;
    vTaskDelete(nullptr);
}

// Callback: device A receives ImageCmd from B
ImageCmdAckStatus on_image_capture_request(uint16_t session_id)
{
    if (g_app_mode != AppMode::camera) {
        ESP_LOGW(TAG, "ImageCmd rejected: not in camera mode (session=%u)", session_id);
        g_radio.notify_capture_dropped();
        return ImageCmdAckStatus::rejected;
    }

    // A retransmit of the already dispatched session is not a new request.
    // Report DUPLICATE so the gateway keeps waiting for the existing image.
    static uint16_t s_last_dispatched_session = 0;
    static bool s_have_dispatched = false;
    if (s_have_dispatched && session_id == s_last_dispatched_session) {
        ESP_LOGW(TAG, "ImageCmd duplicate: session=%u", session_id);
        return ImageCmdAckStatus::duplicate;
    }

    if (g_capture_busy) {
        ESP_LOGW(TAG, "ImageCmd busy: session=%u active_session=%u",
                 session_id, s_have_dispatched ? s_last_dispatched_session : 0);
        // The keep-awake guard belongs to the active capture; do not clear it
        // merely because a second request arrived while that capture is busy.
        return ImageCmdAckStatus::busy;
    }

    static uint32_t s_last_node_capture_ms = 0;
    uint32_t capture_start_ms = 0;
    if (g_audio_clip_enabled) {
        capture_start_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (s_last_node_capture_ms != 0 &&
            (capture_start_ms - s_last_node_capture_ms) < APP_AUDIO_CAPTURE_COOLDOWN_MS) {
            ESP_LOGW(TAG, "ImageCmd cooldown: session=%u", session_id);
            g_radio.notify_capture_dropped();
            return ImageCmdAckStatus::cooldown;
        }
    }

    g_capture_busy = true;

    // Low power: hold the node awake through the capture + push.
    g_radio.notify_capture_starting();

    auto *ctx = new (std::nothrow) ImageCaptureCtx{ session_id };
    if (!ctx) {
        g_capture_busy = false;
        g_radio.notify_capture_dropped();
        ESP_LOGE(TAG, "image capture context alloc failed");
        return ImageCmdAckStatus::rejected;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(image_capture_task, "img_cap",
                                            APP_IMAGE_TASK_STACK_BYTES, ctx,
                                            APP_IMAGE_TASK_PRIORITY, nullptr,
                                            APP_IMAGE_TASK_CORE);
    if (ok != pdPASS) {
        delete ctx;
        g_capture_busy = false;
        g_radio.notify_capture_dropped();
        ESP_LOGE(TAG, "image capture task create failed");
        return ImageCmdAckStatus::rejected;
    }

    s_last_dispatched_session = session_id;
    s_have_dispatched = true;
    if (g_audio_clip_enabled) {
        s_last_node_capture_ms = capture_start_ms;
    }
    return ImageCmdAckStatus::accepted;
}

// Callback: device B receives complete image
void play_audio_clip(const uint8_t *opus_packed, size_t total_len)
{
    OpusCodec clip_codec;
    if (clip_codec.init() != ESP_OK) {
        ESP_LOGE(TAG, "play_audio_clip: codec init failed");
        return;
    }
    bsp_audio_pa_enable(true);

    size_t pos = 0;
    int16_t pcm[APP_AUDIO_FRAME_SAMPLES];
    int16_t stereo[APP_AUDIO_FRAME_SAMPLES * 2];
    uint32_t frames_played = 0;

    while (pos < total_len) {
        uint8_t flen = opus_packed[pos++];
        if (flen == 0 || pos + flen > total_len) break;
        int decoded = clip_codec.decode(&opus_packed[pos], flen, pcm, APP_AUDIO_FRAME_SAMPLES);
        pos += flen;
        if (decoded <= 0) continue;
        for (int i = 0; i < decoded; i++) {
            stereo[2 * i] = pcm[i];
            stereo[2 * i + 1] = pcm[i];
        }
        size_t written = 0;
        bsp_audio_write(stereo, decoded * 2 * sizeof(int16_t), &written);
        frames_played++;
    }
    ESP_LOGI(TAG, "audio clip played: %lu frames", static_cast<unsigned long>(frames_played));
    bsp_audio_pa_enable(false);
}

void on_gw_image_presented(uint16_t session_id, bool displayed)
{
    if (!g_image_ui_pending || session_id != g_image_ui_pending_session) {
        ESP_LOGW(TAG, "unexpected image UI ack: session=%u pending=%u active=%d",
                 session_id, g_image_ui_pending_session, g_image_ui_pending);
        return;
    }

    g_image_ui_pending = false;
    ESP_LOGI(TAG, "image UI complete: sid=%u displayed=%d",
             session_id, displayed);
}

void process_image_rx_work(ImageRxWork *work)
{
    if (!work || !work->raw || work->raw_len == 0) {
        if (work && work->raw) heap_caps_free(work->raw);
        g_image_rx_processing = false;
        return;
    }

    uint8_t *raw = work->raw;
    const size_t raw_len = work->raw_len;
    const uint16_t sid = work->session_id;
    const bool has_audio = (sid & APP_AUDIO_SESSION_FLAG) != 0;
    const int64_t worker_started_us = esp_timer_get_time();

    const uint8_t *jpeg_data = raw;
    size_t jpeg_len = raw_len;
    const uint8_t *opus_data = nullptr;
    size_t opus_len = 0;
    if (has_audio && raw_len > 4) {
        uint32_t jlen = static_cast<uint32_t>(raw[0])
                      | (static_cast<uint32_t>(raw[1]) << 8)
                      | (static_cast<uint32_t>(raw[2]) << 16)
                      | (static_cast<uint32_t>(raw[3]) << 24);
        if (jlen <= raw_len - 4) {
            jpeg_data = &raw[4];
            jpeg_len = jlen;
            opus_data = &raw[4 + jlen];
            opus_len = raw_len - 4 - jlen;
        }
    }
    ESP_LOGI(TAG, "transfer worker: %u bytes (jpeg=%u opus=%u), queue=%lums",
             static_cast<unsigned>(raw_len),
             static_cast<unsigned>(jpeg_len),
             static_cast<unsigned>(opus_len),
             static_cast<unsigned long>((worker_started_us - work->queued_at_us) / 1000));

    bool image_ready = false;
    const char *image_failure = "JPEG decode failed";
    uint8_t *rgb565 = nullptr;
    uint32_t decoded_width = 0;
    uint32_t decoded_height = 0;
    jpeg_dec_config_t dec_cfg = DEFAULT_JPEG_DEC_CONFIG();
    dec_cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;

    jpeg_dec_handle_t decoder = nullptr;
    jpeg_error_t jerr = jpeg_dec_open(&dec_cfg, &decoder);
    if (jerr != JPEG_ERR_OK || !decoder) {
        ESP_LOGE(TAG, "jpeg_dec_open failed: %d", jerr);
    } else {
        jpeg_dec_io_t io = {};
        io.inbuf = const_cast<unsigned char *>(jpeg_data);
        io.inbuf_len = static_cast<int>(jpeg_len);

        jpeg_dec_header_info_t header = {};
        jerr = jpeg_dec_parse_header(decoder, &io, &header);
        if (jerr != JPEG_ERR_OK) {
            ESP_LOGE(TAG, "jpeg_dec_parse_header failed: %d", jerr);
        } else if (header.width != APP_IMAGE_TX_WIDTH ||
                   header.height != APP_IMAGE_TX_HEIGHT) {
            ESP_LOGE(TAG, "reject JPEG size %ux%u, expected %ux%u",
                     header.width, header.height,
                     APP_IMAGE_TX_WIDTH, APP_IMAGE_TX_HEIGHT);
            image_failure = "Bad image size";
        } else {
            int outbuf_len = 0;
            jerr = jpeg_dec_get_outbuf_len(decoder, &outbuf_len);
            if (jerr != JPEG_ERR_OK || outbuf_len <= 0) {
                ESP_LOGE(TAG, "jpeg_dec_get_outbuf_len failed: %d len=%d",
                         jerr, outbuf_len);
            } else {
                rgb565 = static_cast<uint8_t *>(
                    jpeg_calloc_align(static_cast<size_t>(outbuf_len), 16));
                if (!rgb565) {
                    ESP_LOGE(TAG, "RGB565 alloc failed: %d bytes", outbuf_len);
                    image_failure = "Display memory low";
                } else {
                    io.outbuf = rgb565;
                    jerr = jpeg_dec_process(decoder, &io);
                    if (jerr == JPEG_ERR_OK) {
                        decoded_width = header.width;
                        decoded_height = header.height;
                    } else {
                        ESP_LOGE(TAG, "jpeg_dec_process failed: %d", jerr);
                    }
                }
            }
        }
        jpeg_dec_close(decoder);
    }

    if (rgb565 && decoded_width != 0 && decoded_height != 0) {
        g_image_ui_pending_session = sid;
        g_image_ui_pending = true;
        if (ui_gw_rx_complete(sid,
                              reinterpret_cast<uint16_t *>(rgb565),
                              decoded_width,
                              decoded_height,
                              static_cast<uint32_t>(raw_len),
                              work->transfer_ms)) {
            rgb565 = nullptr;
            image_ready = true;
        } else {
            g_image_ui_pending = false;
            image_failure = "UI QUEUE FULL";
        }
    }
    if (rgb565) jpeg_free_align(rgb565);

    if (!image_ready) {
        ui_gw_rx_failed(image_failure);
        heap_caps_free(raw);
        g_image_rx_processing = false;
        return;
    }

    ESP_LOGI(TAG, "image display queued: sid=%u, processing=%lums",
             sid,
             static_cast<unsigned long>((esp_timer_get_time() - worker_started_us) / 1000));
    g_image_rx_processing = false;
    image_store_save(jpeg_data, jpeg_len, opus_data, opus_len, sid);
    if (opus_data && opus_len > 0) {
        play_audio_clip(opus_data, opus_len);
    }
    heap_caps_free(raw);
}

void image_rx_worker_task(void *arg)
{
    (void)arg;
    ImageRxWork work = {};
    while (true) {
        if (xQueueReceive(g_image_rx_work_queue, &work, portMAX_DELAY) == pdTRUE) {
            process_image_rx_work(&work);
        }
    }
}

esp_err_t start_image_rx_worker()
{
    if (g_image_rx_work_queue) return ESP_OK;

    g_image_rx_work_queue = xQueueCreate(1, sizeof(ImageRxWork));
    if (!g_image_rx_work_queue) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(image_rx_worker_task, "image_rx",
                                            APP_IMAGE_RX_TASK_STACK_BYTES, nullptr,
                                            APP_IMAGE_RX_TASK_PRIORITY, nullptr,
                                            APP_IMAGE_RX_TASK_CORE);
    if (ok != pdPASS) {
        vQueueDelete(g_image_rx_work_queue);
        g_image_rx_work_queue = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void on_image_rx_complete(ImageTransfer *xfer)
{
    ESP_LOGI(TAG, "on_image_rx_complete: xfer=%p complete=%d",
             xfer, xfer ? xfer->rx_complete() : -1);
    if (!xfer || !xfer->rx_complete()) return;

    g_image_rx_processing = true;
    if (g_audio_clip_enabled) {
        s_last_gw_capture_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    }

    const uint16_t sid = xfer->rx_session_id();
    const uint32_t transfer_ms = g_radio.last_transfer_ms();
    uint8_t *raw = nullptr;
    size_t raw_len = 0;
    esp_err_t e = xfer->rx_reassemble(&raw, &raw_len);
    if (e != ESP_OK || !raw) {
        ESP_LOGE(TAG, "rx_reassemble failed: %d", e);
        ui_gw_rx_failed(e == ESP_ERR_NO_MEM ? "RX MEMORY ERROR" : "REASSEMBLE ERROR");
        xfer->rx_reset();
        g_image_rx_processing = false;
        return;
    }

    xfer->rx_reset();
    const ImageRxWork work = {
        .raw = raw,
        .raw_len = raw_len,
        .session_id = sid,
        .transfer_ms = transfer_ms,
        .queued_at_us = esp_timer_get_time(),
    };
    if (!g_image_rx_work_queue ||
        xQueueSend(g_image_rx_work_queue, &work, 0) != pdTRUE) {
        g_image_rx_processing = false;
        heap_caps_free(raw);
        ESP_LOGE(TAG, "image processing queue unavailable/full");
        ui_gw_rx_failed("PROCESS QUEUE FULL");
        return;
    }
    ESP_LOGI(TAG, "image processing queued: sid=%u, %u bytes",
             sid, static_cast<unsigned>(raw_len));
}

void camera_capture_task(void *arg)
{
    (void)arg;
    uint8_t *frame = nullptr;
    size_t len = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t pixfmt = 0;

#if APP_AUDIO_FEATURES_ENABLE
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.suspend();
    }
#endif
    esp_err_t audio_e = bsp_audio_suspend();
    if (audio_e != ESP_OK) {
        ESP_LOGW(TAG, "audio suspend before camera: %s", esp_err_to_name(audio_e));
    }
#endif
    bsp_lcd_set_camera_status("Preparing camera...");
    esp_err_t e = bsp_lcd_release_for_camera();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "release lcd for camera: %s", esp_err_to_name(e));
        bsp_lcd_reinit_after_camera();
        bsp_lcd_set_camera_status("LCD release failed");
#if APP_AUDIO_FEATURES_ENABLE
        if ((e = bsp_audio_resume()) != ESP_OK) {
            ESP_LOGW(TAG, "audio resume after LCD release failure: %s", esp_err_to_name(e));
        }
#if APP_RADIO_FEATURES_ENABLE
        if (g_radio_active) {
            g_radio.resume();
        }
#endif
#endif
        g_capture_busy = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_err_t capture_e = g_camera_uart.capture_frame(&frame, &len, &width, &height, &pixfmt);
    ESP_LOGI(TAG, "capture result=%s frame=%p len=%u %lux%lu fourcc=0x%08lx",
             esp_err_to_name(capture_e), frame, static_cast<unsigned>(len),
             static_cast<unsigned long>(width),
             static_cast<unsigned long>(height),
             static_cast<unsigned long>(pixfmt));

    esp_err_t lcd_e = bsp_lcd_reinit_after_camera();
    if (lcd_e != ESP_OK) {
        ESP_LOGE(TAG, "lcd reinit after camera: %s", esp_err_to_name(lcd_e));
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_audio_resume()) != ESP_OK) {
        ESP_LOGW(TAG, "audio resume after camera: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    if (g_radio_active) {
        g_radio.resume();
    }
#endif
#endif

    if (capture_e == ESP_OK && (pixfmt == 0x56595559 || pixfmt == 0x59565955 ||
                                pixfmt == 0x55595659 || pixfmt == 0x59555956)) {
        if (bsp_lcd_show_yuv422_photo(frame, width, height, pixfmt) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK && pixfmt == 0x59455247) { // 'GREY'
        if (bsp_lcd_show_gray_photo(frame, width, height) == ESP_OK) {
            bsp_lcd_set_camera_status("Captured. Touch capture to retake");
        } else {
            bsp_lcd_set_camera_status("Display photo failed");
        }
    } else if (capture_e == ESP_OK) {
        char status[64];
        snprintf(status, sizeof(status), "Unsupported pixel 0x%08lx",
                 static_cast<unsigned long>(pixfmt));
        bsp_lcd_set_camera_status(status);
    } else {
        bsp_lcd_clear_camera_photo();
        char status[64];
        const char *short_name = short_error_name(capture_e);
        if (short_name) {
            snprintf(status, sizeof(status), "Fail:%s", short_name);
        } else {
            snprintf(status, sizeof(status), "Fail:0x%lx",
                     static_cast<unsigned long>(capture_e));
        }
        bsp_lcd_set_camera_status(status);
    }

    heap_caps_free(frame);
    g_capture_busy = false;
    vTaskDelete(nullptr);
}

void on_lcd_capture(void *user)
{
    (void)user;
    if (g_app_mode == AppMode::radio) {
        bsp_lcd_set_camera_status("Radio mode. Press K5 for camera mode");
        return;
    }
    if (g_capture_busy) {
        bsp_lcd_set_camera_status("Capture already running");
        return;
    }
    g_capture_busy = true;
    BaseType_t ok = xTaskCreatePinnedToCore(camera_capture_task,
                                            "touch_capture",
                                            APP_CAMERA_TASK_STACK_BYTES,
                                            nullptr,
                                            APP_CAMERA_TASK_PRIORITY + 3,
                                            nullptr,
                                            APP_CAMERA_TASK_CORE);
    if (ok != pdPASS) {
        g_capture_busy = false;
        bsp_lcd_set_camera_status("Capture task start failed");
    }
}

// Radio mode: progress callback for UI update during image RX
void on_image_rx_progress(uint16_t session_id, uint16_t received,
                          uint16_t total, int16_t rssi)
{
    if (g_app_mode == AppMode::radio) {
        if (received == 0) {
            ui_gw_rx_begin(session_id, total);
            return;
        }
        ui_gw_rx_progress(received, total, rssi);
    }
}

void on_image_rx_error(ImageRxError error)
{
    if (g_app_mode != AppMode::radio) return;

    if (error == ImageRxError::crc_mismatch) {
        ui_gw_rx_crc_error();
    } else if (error == ImageRxError::no_memory) {
        ui_gw_rx_failed("RX MEMORY ERROR");
    } else {
        ui_gw_rx_failed("RX TIMEOUT");
    }
}

void on_image_request_rejected(ImageCmdAckStatus status)
{
    const char *reason = "Camera rejected";
    if (status == ImageCmdAckStatus::busy) {
        reason = "Camera busy, retry";
    } else if (status == ImageCmdAckStatus::cooldown) {
        reason = "Camera cooling down";
    }

    ESP_LOGW(TAG, "image request rejected by node: status=%u",
             static_cast<unsigned>(status));
    if (g_app_mode == AppMode::radio) {
        ui_gw_rx_failed(reason);
    }
}

// Radio mode (gateway): a node reported its battery voltage -> push to UI.
void on_vbat_received(uint16_t vbat_mv)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_update_vbat(vbat_mv);
    }
}

void on_image_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (g_app_mode == AppMode::radio) {
        ui_gw_rx_eot_nack(missing_count, is_first_eot);
    }
}

// Gateway UI capture callback — triggers remote photo via radio

void cooldown_retry_cb(void *arg)
{
    if (g_radio.image_busy() || g_image_rx_processing || g_image_ui_pending) {
        ESP_LOGW(TAG, "cooldown retry ignored: transfer/display in progress");
        return;
    }
    ESP_LOGI(TAG, "Cooldown expired, auto-triggering capture");
    s_last_gw_capture_ms = (uint32_t)(esp_timer_get_time() / 1000);
    image_store_abort_transfer();
    (void)g_radio.trigger_image_capture();
}

bool on_gw_capture(void)
{
    // A transfer is already running — don't preempt it (that deadlocks the
    // half-duplex radio) and don't abort the HTTP download. Ignore the request.
    if (g_radio.image_busy() || g_image_rx_processing ||
        g_image_ui_pending) {
        ESP_LOGW(TAG, "UI capture: transfer in progress, ignoring");
        return false;
    }

    if (g_audio_clip_enabled) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (s_last_gw_capture_ms != 0 &&
            (now - s_last_gw_capture_ms) < APP_AUDIO_CAPTURE_COOLDOWN_MS) {
            uint32_t remaining_ms = APP_AUDIO_CAPTURE_COOLDOWN_MS - (now - s_last_gw_capture_ms);
            ESP_LOGW(TAG, "UI capture: audio cooldown, auto-retry in %lums",
                     (unsigned long)remaining_ms);
            if (!g_cooldown_retry_timer) {
                const esp_timer_create_args_t args = {
                    .callback = cooldown_retry_cb,
                    .arg = nullptr,
                    .dispatch_method = ESP_TIMER_TASK,
                    .name = "cooldown_retry",
                };
                esp_timer_create(&args, &g_cooldown_retry_timer);
            }
            esp_timer_stop(g_cooldown_retry_timer);
            esp_timer_start_once(g_cooldown_retry_timer, (uint64_t)remaining_ms * 1000);
            return false;
        }
        s_last_gw_capture_ms = now;
    }
    if (g_cooldown_retry_timer) {
        esp_timer_stop(g_cooldown_retry_timer);
    }
    ESP_LOGI(TAG, "UI capture: trigger remote photo");
    image_store_abort_transfer();
    return g_radio.trigger_image_capture();
}

// Gateway UI: user left the transfer page — abort the in-progress RX so the
// gateway stops requesting/receiving this image (the node's TX self-aborts once
// its ACKs stop). Also drop any partial store-side transfer.
void on_gw_rx_abort(void)
{
    ESP_LOGI(TAG, "UI: left transfer page, aborting image RX");
    g_radio.abort_image_rx();
    image_store_abort_transfer();
}

// Gateway UI: true while an image request/RX is in progress. The UI uses this to
// ignore the capture key mid-transfer (no new request is started).
bool on_gw_query_busy(void)
{
    return g_radio.image_busy() || g_image_rx_processing ||
           g_image_ui_pending || g_radio.frequency_change_busy();
}

// Gateway UI interval change callback — sends config to camera node
bool on_gw_interval_change(uint32_t interval_sec)
{
    ESP_LOGI(TAG, "UI interval change: %lus", static_cast<unsigned long>(interval_sec));
    return g_radio.send_config(APP_CFG_KEY_INTERVAL, interval_sec);
}

bool on_gw_audio_clip_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI audio clip: %s", enable ? "on" : "off");
    // Commit local state only on success so the UI switch stays the single
    // source of truth (mismatch would desync the capture cooldown logic).
    bool ok = g_radio.send_config(APP_CFG_KEY_AUDIO_CLIP, enable);
    if (ok) {
        g_audio_clip_enabled = (enable != 0);
    }
    return ok;
}

bool on_gw_sound_trigger_change(uint32_t level)
{
    ESP_LOGI(TAG, "UI sound trigger: %lu", static_cast<unsigned long>(level));
    return g_radio.send_config(APP_CFG_KEY_SOUND_TRIGGER, level);
}

bool on_gw_pir_trigger_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI PIR trigger: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_PIR_TRIGGER, enable);
}

bool on_gw_voice_alarm_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI voice alarm: %s", enable ? "on" : "off");
    return g_radio.send_config(APP_CFG_KEY_VOICE_ALARM, enable);
}

bool on_gw_low_power_change(uint32_t enable)
{
    ESP_LOGI(TAG, "UI low power: %s", enable ? "on" : "off");
    // Send first (reads current g_low_power_enabled to decide whether to send
    // the wakeup preamble), then commit local state only on success so the UI
    // switch stays the single source of truth.
    bool ok = g_radio.send_config(APP_CFG_KEY_LOW_POWER, enable);
    if (ok) {
        g_low_power_enabled = (enable != 0);
        if (enable) {
            // Low-power mode disables the gateway audio-clip path for the current session.
            g_audio_clip_enabled = false;
        }
    }
    return ok;
}

void on_frequency_change_result(bool success, uint32_t frequency_hz)
{
    if (success) {
        (void)save_frequency_index(frequency_index_from_hz(frequency_hz));
    }
    ui_gw_frequency_result(success, frequency_hz);
    ESP_LOGW(TAG, "frequency change finished: %s hz=%lu",
             success ? "OK" : "FAIL", static_cast<unsigned long>(frequency_hz));
}

bool on_gw_frequency_change(uint32_t frequency_hz)
{
    ESP_LOGW(TAG, "frequency button request: hz=%lu",
             static_cast<unsigned long>(frequency_hz));
    uint32_t current_hz = g_radio.current_frequency_hz();
    if (frequency_hz == current_hz) {
        uint8_t current_index = frequency_index_from_hz(current_hz);
        uint8_t next_index =
            static_cast<uint8_t>((current_index + 1U) % APP_FLRC_FREQUENCY_PRESET_COUNT);
        frequency_hz = kFrequencyPresetsHz[next_index];
        ESP_LOGW(TAG, "frequency UI was stale, using next radio preset: index=%u hz=%lu",
                 next_index, static_cast<unsigned long>(frequency_hz));
    }
    return g_radio.request_frequency_change(frequency_hz);
}

void on_wifi_prov_request(void)
{
    if (wifi_mgr_get_state() == WIFI_MGR_CONNECTED ||
        wifi_mgr_get_state() == WIFI_MGR_PROVISIONING) {
        ESP_LOGW(TAG, "WiFi already active, ignoring prov request");
        return;
    }
    ESP_LOGI(TAG, "WiFi provisioning requested");
    esp_err_t err = wifi_mgr_start_provisioning();
    if (err == ESP_OK) {
        httpd_handle_t h = wifi_mgr_get_httpd();
        if (h) {
            image_store_register_httpd(h);
        }
        const char *name = wifi_mgr_get_service_name();
        char payload[200];
        snprintf(payload, sizeof(payload),
            "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"softap\"}",
            name, wifi_mgr_get_ap_password());
        ui_gw_show_qr(payload);
    } else {
        ESP_LOGE(TAG, "start provisioning failed: %s", esp_err_to_name(err));
    }
}

void on_wifi_disconnect_request(void)
{
    ESP_LOGI(TAG, "WiFi disconnect requested");
    wifi_mgr_disconnect();
}

void on_wifi_state_change(wifi_mgr_state_t state)
{
    const char *str = "Disconnected";
    switch (state) {
    case WIFI_MGR_CONNECTING:    str = "Connecting..."; break;
    case WIFI_MGR_CONNECTED:     str = "Connected"; break;
    case WIFI_MGR_PROVISIONING:  str = "Provisioning..."; break;
    default: break;
    }
    ui_gw_wifi_update(str, wifi_mgr_get_ssid(), wifi_mgr_get_rssi());

    if (state == WIFI_MGR_CONNECTED) {
        wifi_mgr_ensure_httpd();
        httpd_handle_t h = wifi_mgr_get_httpd();
        if (h) {
            image_store_register_httpd(h);
        }
        image_store_start_sntp();
    }
}

void on_button(bsp_btn_id_t id, bool pressed, void *user)
{
    (void)user;

    // In radio mode, route all keys to the gateway UI
    if (g_app_mode == AppMode::radio) {
        // K6 long press (>1.5s) → switch mode (keep as escape hatch)
        if (id == BSP_BTN_PTT) {
            if (pressed) {
                g_ptt_press_time_us = esp_timer_get_time();
                g_ptt_held_long = false;
                if (g_ptt_timer) {
                    esp_timer_start_once(g_ptt_timer, 1500000); // 1.5s for mode switch
                }
            } else {
                if (g_ptt_timer) {
                    esp_timer_stop(g_ptt_timer);
                }
                if (g_ptt_held_long) {
                    switch_mode_and_restart();
                } else {
                    ui_gw_key_event(id, true);
                }
                g_ptt_held_long = false;
            }
            return;
        }
        ui_gw_key_event(id, pressed);
        return;
    }

    // Camera mode binds only K5/USER1; voice and volume handlers remain unbound.
    if (id == BSP_BTN_USER1) {
        if (pressed) {
            switch_mode_and_restart();
        }
        return;
    }
}

// Console command: "version" -> print firmware version and FLRC TX power.
int cmd_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "firmware version: V%s", esp_app_get_description()->version);
    ESP_LOGI(TAG, "FLRC TX power: %d dBm", APP_FLRC_TX_POWER_DBM);
    return 0;
}

// Start a REPL on the console (USB-Serial-JTAG) so the user can type commands
// like "version" over the same serial port the logs come out of.
void start_console()
{
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "lr2021>";

    const esp_console_cmd_t version_cmd = {
        .command = "version",
        .help = "Print firmware version and FLRC TX power",
        .hint = nullptr,
        .func = &cmd_version,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr,
    };

    esp_err_t e = esp_console_cmd_register(&version_cmd);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "register version cmd: %s", esp_err_to_name(e));
        return;
    }

    esp_console_dev_usb_serial_jtag_config_t dev_cfg =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    if ((e = esp_console_new_repl_usb_serial_jtag(&dev_cfg, &repl_cfg, &repl)) != ESP_OK) {
        ESP_LOGE(TAG, "console repl init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = esp_console_start_repl(repl)) != ESP_OK) {
        ESP_LOGE(TAG, "console repl start: %s", esp_err_to_name(e));
    }
}
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Lierda L-LRMAM36-FANN4-DK01 booting");
    esp_log_level_set("RALF_LR20XX", ESP_LOG_WARN);
    // Suppress noisy but harmless HTTP server warnings: /favicon.ico 404s
    // (browser auto-requests it) and recv errno 104 (client closed connection).
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);

    esp_err_t e;
    init_nvs();
    g_app_mode = load_app_mode();
    ESP_LOGI(TAG, "app mode: %s", mode_name(g_app_mode));

    ESP_ERROR_CHECK(bsp_i2c_init());

    printf("PSRAM free: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("PSRAM total: %d\n", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));

#if APP_CAMERA_LCD_BRINGUP
    bsp_i2c_scan();
    if ((e = g_camera_uart.init()) != ESP_OK) {
        ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_init()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        return;
    }
    if ((e = bsp_lcd_show_test_pattern()) != ESP_OK) {
        ESP_LOGE(TAG, "lcd test pattern: %s", esp_err_to_name(e));
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(800));
    if ((e = bsp_lcd_start_camera_ui(on_lcd_capture, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "camera ui start: %s", esp_err_to_name(e));
        return;
    }
    ESP_LOGI(TAG, "V02 camera/LCD validation UI ready: ST7789V3 %ux%u, SP0A39 DVP %ux%u",
             APP_LCD_H_RES, APP_LCD_V_RES,
             APP_CAMERA_SENSOR_WIDTH, APP_CAMERA_SENSOR_HEIGHT);
    return;
#endif

#if APP_CAMERA_ONLY_BRINGUP
    ESP_LOGW(TAG, "camera-only bring-up: skipping CON6 detect, LED, audio, LR2021 radio, buttons, LCD, chime");
#if APP_CAMERA_UART_ENABLE
    if ((e = g_camera_uart.start()) != ESP_OK) {
        ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "camera-only SP0A39 one-frame capture: MCLK GPIO%d, PWDN IOEXP P%d",
             BSP_SP0A39_MCLK_GPIO, BSP_SP0A39_PWDN_IOEXP_PIN);
#else
    ESP_LOGW(TAG, "APP_CAMERA_UART_ENABLE is disabled");
#endif
    return;
#endif

    bsp_i2c_scan();

#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_led_init()) != ESP_OK) {
        ESP_LOGE(TAG, "led init: %s", esp_err_to_name(e));
    }
    if (g_app_mode == AppMode::radio) {
        if ((e = bsp_audio_init_playback_only(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init (playback): %s", esp_err_to_name(e));
        }
    } else {
        if ((e = bsp_audio_init(APP_AUDIO_SAMPLE_RATE_HZ)) != ESP_OK) {
            ESP_LOGE(TAG, "audio init: %s", esp_err_to_name(e));
        }
    }
    if ((e = g_audio.init()) != ESP_OK) {
        ESP_LOGE(TAG, "audio diagnostics init: %s", esp_err_to_name(e));
    }
#if APP_RADIO_FEATURES_ENABLE
    {
        // Drive LR2021 NRST high before radio initialization so the first reset is valid.
        // This prevents BUSY from remaining high during PRAM initialization.
        {
            gpio_config_t nrst_conf = {};
            nrst_conf.pin_bit_mask = 1ULL << CONFIG_LR2021_NRST_GPIO;
            nrst_conf.mode         = GPIO_MODE_OUTPUT;
            nrst_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
            nrst_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            nrst_conf.intr_type    = GPIO_INTR_DISABLE;
            gpio_config(&nrst_conf);
            gpio_set_level((gpio_num_t)CONFIG_LR2021_NRST_GPIO, 1);
        }

        bool radio_ok = true;
        uint8_t frequency_index = load_frequency_index();
        if (!g_radio.set_initial_frequency(kFrequencyPresetsHz[frequency_index])) {
            ESP_LOGW(TAG, "invalid saved frequency preset, using default");
            frequency_index = 0;
            (void)g_radio.set_initial_frequency(kFrequencyPresetsHz[0]);
        }
        ESP_LOGI(TAG, "NVS frequency preset: index=%u hz=%lu", frequency_index,
                 static_cast<unsigned long>(kFrequencyPresetsHz[frequency_index]));
        if (g_app_mode == AppMode::radio) {
            if ((e = g_radio.init_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init (gateway): %s", esp_err_to_name(e));
                radio_ok = false;
            }
        } else {
            if ((e = g_radio.init()) != ESP_OK) {
                ESP_LOGE(TAG, "radio init: %s", esp_err_to_name(e));
                radio_ok = false;
            }
        }
        if (radio_ok) {
            g_radio.set_image_capture_cb(on_image_capture_request);
            g_radio.set_image_request_rejected_cb(on_image_request_rejected);
            g_radio.set_image_rx_complete_cb(on_image_rx_complete);
            g_radio.set_image_rx_progress_cb(on_image_rx_progress);
            g_radio.set_image_rx_error_cb(on_image_rx_error);
            g_radio.set_vbat_received_cb(on_vbat_received);
            g_radio.set_image_rx_eot_cb(on_image_rx_eot_nack);
            g_radio.set_config_received_cb(on_config_received);
            g_radio.set_frequency_committed_cb(on_frequency_committed);
            g_radio.set_frequency_change_result_cb(on_frequency_change_result);
            g_radio.set_low_power_standby_cb(on_low_power_standby);
        }
        if (g_app_mode == AppMode::radio && radio_ok) {
            if ((e = start_image_rx_worker()) != ESP_OK) {
                ESP_LOGE(TAG, "image RX worker start: %s", esp_err_to_name(e));
                radio_ok = false;
            }
        }
        if (g_app_mode == AppMode::radio && radio_ok) {
#if APP_RADIO_TASKS_ENABLE
            if ((e = g_radio.start_gateway()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (gateway): %s", esp_err_to_name(e));
            } else {
                g_radio_active = true;
                nvs_handle_t gw_nvs;
                if (nvs_open("ui_gw", NVS_READONLY, &gw_nvs) == ESP_OK) {
                    uint8_t lp = 0;
                    nvs_get_u8(gw_nvs, "lowpwr", &lp);
                    g_low_power_enabled = (lp != 0);
                    nvs_close(gw_nvs);
                }
            }
#else
            ESP_LOGW(TAG, "radio initialized but tasks/RX disabled for camera isolation");
#endif
        } else if (g_app_mode == AppMode::camera && radio_ok) {
            if ((e = g_radio.start()) != ESP_OK) {
                ESP_LOGE(TAG, "radio task start (camera mode): %s", esp_err_to_name(e));
            }
            g_radio.enable_opus_preenc(true);
            ESP_LOGI(TAG, "camera mode: radio initialized for image transfer");
            g_capture_interval_sec = load_capture_interval();
            g_audio_clip_enabled = load_config_u8("audio_clip", 0) != 0;
            g_radio.set_sound_trigger_level(load_config_u8("snd_trig", 0));
            g_radio.set_pir_enabled(load_config_u8("pir", 0) != 0);
            g_voice_alarm_enabled = load_config_u8("alarm", 0) != 0;
            g_low_power_enabled = load_config_u8("lowpwr", 0) != 0;
            ESP_LOGI(TAG, "NVS: audio=%d snd=%d pir=%d alarm=%d lowpwr=%d",
                     g_audio_clip_enabled, load_config_u8("snd_trig", 0),
                     load_config_u8("pir", 0), g_voice_alarm_enabled, g_low_power_enabled);
            start_auto_capture_timer();
            update_camera_timer_status();
            start_countdown_timer();

            // PIR sensor on GPIO12: delay 5s then arm high-level trigger
            // (allow residual touch IC signals to settle after power-on)
            gpio_config_t pir_cfg = {
                .pin_bit_mask = 1ULL << APP_PIR_GPIO,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&pir_cfg);
            gpio_install_isr_service(0); // OK if already installed
            ESP_LOGI(TAG, "PIR: GPIO%d configured, arming in 5s...", APP_PIR_GPIO);
            const esp_timer_create_args_t pir_arm_args = {
                .callback = pir_arm_timer_cb,
                .arg = &g_radio,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "pir_arm",
                .skip_unhandled_events = true,
            };
            esp_timer_handle_t pir_arm_timer = nullptr;
            esp_timer_create(&pir_arm_args, &pir_arm_timer);
            esp_timer_start_once(pir_arm_timer, 5000000ULL); // 5s
        }
    }
#else
    ESP_LOGW(TAG, "radio feature disabled for camera/audio isolation");
#endif
#endif
    if (g_app_mode == AppMode::camera) {
        if ((e = g_camera_uart.init()) != ESP_OK) {
            ESP_LOGE(TAG, "camera init: %s", esp_err_to_name(e));
        }
#if APP_CAMERA_UART_ENABLE
        if ((e = g_camera_uart.start()) != ESP_OK) {
            ESP_LOGE(TAG, "camera uart start: %s", esp_err_to_name(e));
        }
#endif
    }
    if (g_app_mode != AppMode::camera) {
        if ((e = bsp_lcd_init()) != ESP_OK) {
            ESP_LOGE(TAG, "lcd init: %s", esp_err_to_name(e));
        } else if (g_app_mode == AppMode::radio) {
            if ((e = bsp_lcd_start_gateway_ui()) != ESP_OK) {
                ESP_LOGE(TAG, "gateway ui start: %s", esp_err_to_name(e));
            } else {
                ui_gw_set_capture_cb(on_gw_capture);
                ui_gw_set_interval_cb(on_gw_interval_change);
                ui_gw_set_audio_clip_cb(on_gw_audio_clip_change);
                ui_gw_set_sound_trigger_cb(on_gw_sound_trigger_change);
                ui_gw_set_pir_trigger_cb(on_gw_pir_trigger_change);
                ui_gw_set_voice_alarm_cb(on_gw_voice_alarm_change);
                ui_gw_set_low_power_cb(on_gw_low_power_change);
                ui_gw_set_frequency_cb(on_gw_frequency_change);
                ui_gw_set_current_frequency(g_radio.current_frequency_hz());
                ui_gw_set_wifi_prov_cb(on_wifi_prov_request);
                ui_gw_set_wifi_disconnect_cb(on_wifi_disconnect_request);
                ui_gw_set_rx_abort_cb(on_gw_rx_abort);
                ui_gw_set_busy_cb(on_gw_query_busy);
                ui_gw_set_image_presented_cb(on_gw_image_presented);

                wifi_mgr_set_state_cb(on_wifi_state_change);
                wifi_mgr_init();
                image_store_init();
                image_store_restore_time();
                {
                    httpd_handle_t h = wifi_mgr_get_httpd();
                    if (h) {
                        image_store_register_httpd(h);
                        // Captive portal: DNS hijack + 404->gallery redirect so
                        // clients on the open AP stay connected and auto-open
                        // the gallery instead of disconnecting for "no internet".
                        captive_portal_start(h);
                    }
                }

                // Sync audio clip state from gateway UI NVS
                {
                    nvs_handle_t h;
                    uint8_t val = 0;
                    if (nvs_open("ui_gw", NVS_READONLY, &h) == ESP_OK) {
                        nvs_get_u8(h, "audio", &val);
                        nvs_close(h);
                    }
                    g_audio_clip_enabled = (val != 0);
                }
            }
        }
    }
#if APP_AUDIO_FEATURES_ENABLE
    if ((e = bsp_button_init(on_button, nullptr)) != ESP_OK) {
        ESP_LOGE(TAG, "btn init: %s", esp_err_to_name(e));
    }

    // Create PTT long-press timer for K6 short/long press detection
    const esp_timer_create_args_t ptt_timer_args = {
        .callback = ptt_long_press_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ptt_long",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&ptt_timer_args, &g_ptt_timer);

    // Battery / external-supply voltage monitor: read VBAT_ADC (GPIO11) every
    // 15 s and cache the result. Radio path grabs the cached value to embed in
    // ImageStart and periodic broadcast packets.
    if ((e = bsp_vbat_monitor_start(0)) != ESP_OK) {
        ESP_LOGW(TAG, "vbat monitor start: %s", esp_err_to_name(e));
    }

    g_audio.play_startup_chime();

    ESP_LOGI(TAG, "audio config: %u Hz local record/playback",
             APP_AUDIO_SAMPLE_RATE_HZ);
#if APP_RADIO_FEATURES_ENABLE
#if APP_RADIO_TASKS_ENABLE
    ESP_LOGI(TAG, "voice config: Opus %u Hz, %u ms, %d bps CBR; FLRC %lu Hz, %lu bps",
             APP_AUDIO_SAMPLE_RATE_HZ, APP_AUDIO_FRAME_MS, APP_OPUS_BITRATE_BPS,
             g_radio.current_frequency_hz(), APP_FLRC_BITRATE_BPS);
#else
    ESP_LOGW(TAG, "FLRC radio init only; RX/TX tasks disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "FLRC voice disabled in this build");
#endif
#else
    ESP_LOGW(TAG, "audio/radio/button features disabled");
#endif
    ESP_LOGI(TAG, "display: ST7789T3 %ux%u camera capture UI",
             APP_LCD_H_RES, APP_LCD_V_RES);
    ESP_LOGI(TAG, "K5: switch camera/radio mode. K6/PTT: FLRC voice in radio mode. K4=vol+, K3=vol-");

    ESP_LOGI(TAG, "firmware version: V%s", esp_app_get_description()->version);
    ESP_LOGI(TAG, "FLRC TX power: %d dBm", APP_FLRC_TX_POWER_DBM);

    // Console: type "version" to reprint version + TX power at any time.
    start_console();
}
