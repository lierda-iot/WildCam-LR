#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_PAGE_IMAGE = 0,
    UI_PAGE_RX,
    UI_PAGE_LINK,
    UI_PAGE_CONFIG,
    UI_PAGE_QR,
    UI_PAGE_COUNT
} ui_page_t;

typedef bool (*ui_gw_capture_cb_t)(void);
typedef bool (*ui_gw_interval_cb_t)(uint32_t interval_sec);
typedef bool (*ui_gw_audio_clip_cb_t)(uint32_t enable);
typedef bool (*ui_gw_sound_trigger_cb_t)(uint32_t level);
typedef bool (*ui_gw_pir_trigger_cb_t)(uint32_t enable);
typedef bool (*ui_gw_voice_alarm_cb_t)(uint32_t enable);
typedef bool (*ui_gw_low_power_cb_t)(uint32_t enable);
typedef bool (*ui_gw_frequency_cb_t)(uint32_t frequency_hz);
typedef void (*ui_gw_wifi_prov_cb_t)(void);
typedef void (*ui_gw_wifi_disconnect_cb_t)(void);
// Called when the user leaves the transfer (RX) page: abort the current RX.
typedef void (*ui_gw_rx_abort_cb_t)(void);
// Returns true while an image transfer (request or RX) is already in progress.
// The UI queries this before handling capture, navigation keys, or gestures so
// an active transfer cannot be aborted or covered by a stale page transition.
typedef bool (*ui_gw_busy_cb_t)(void);
// Called after an owned RGB565 event is discarded, or after the final LCD DMA
// flush for the image page completes. Queue eviction can invoke the discarded
// path from an event producer task; it must not block or call LVGL.
typedef void (*ui_gw_image_presented_cb_t)(uint16_t session_id, bool displayed);

esp_err_t ui_gw_init(void);
void ui_gw_key_event(bsp_btn_id_t key, bool pressed);

// RX state APIs are non-blocking event posts. LVGL objects are updated by
// the gateway UI timer from lv_timer_handler(), never by the radio task.
void ui_gw_rx_begin(uint16_t session_id, uint16_t total_frags);
void ui_gw_rx_progress(uint16_t received, uint16_t total, int16_t rssi);
void ui_gw_rx_crc_error(void);
// On success, ownership of rgb565 transfers to the LVGL task, which releases
// it with jpeg_free_align() after copying it to the canvas. On failure, the
// caller retains ownership and must release it.
bool ui_gw_rx_complete(uint16_t session_id, uint16_t *rgb565,
                       uint32_t w, uint32_t h,
                       uint32_t jpeg_size, uint32_t elapsed_ms);
void ui_gw_rx_failed(const char *reason);

void ui_gw_rx_eot_nack(uint16_t missing_count, bool is_first_eot);

void ui_gw_set_capture_cb(ui_gw_capture_cb_t cb);
void ui_gw_set_interval_cb(ui_gw_interval_cb_t cb);
void ui_gw_set_audio_clip_cb(ui_gw_audio_clip_cb_t cb);
void ui_gw_set_sound_trigger_cb(ui_gw_sound_trigger_cb_t cb);
void ui_gw_set_pir_trigger_cb(ui_gw_pir_trigger_cb_t cb);
void ui_gw_set_voice_alarm_cb(ui_gw_voice_alarm_cb_t cb);
void ui_gw_set_low_power_cb(ui_gw_low_power_cb_t cb);
void ui_gw_set_frequency_cb(ui_gw_frequency_cb_t cb);
void ui_gw_set_wifi_prov_cb(ui_gw_wifi_prov_cb_t cb);
void ui_gw_set_wifi_disconnect_cb(ui_gw_wifi_disconnect_cb_t cb);
void ui_gw_set_rx_abort_cb(ui_gw_rx_abort_cb_t cb);
void ui_gw_set_busy_cb(ui_gw_busy_cb_t cb);
void ui_gw_set_image_presented_cb(ui_gw_image_presented_cb_t cb);

void ui_gw_wifi_update(const char *state_str, const char *ssid, int8_t rssi);
void ui_gw_show_qr(const char *payload);
void ui_gw_hide_qr(void);

/* Non-blocking update of the node battery voltage display (mV).
 * Call with 0 to hide. */
void ui_gw_update_vbat(uint16_t vbat_mv);
void ui_gw_set_current_frequency(uint32_t frequency_hz);
void ui_gw_frequency_result(bool success, uint32_t frequency_hz);

#ifdef __cplusplus
}
#endif
