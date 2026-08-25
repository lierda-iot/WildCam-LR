#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Shared I2C0 bus + TCA9554A GPIO expander -------------------- */
esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_reinit(void);
i2c_master_bus_handle_t bsp_i2c_bus(void);

/* Probe each 7-bit address on I2C0 and log the ones that ACK. Handy for
 * bringing up a new board. Returns ESP_OK if the scan ran; the caller can
 * read the log output to see which addresses responded. */
esp_err_t bsp_i2c_scan(void);

typedef enum {
    BSP_CON6_PERIPHERAL_LCD_ST7789 = 0,
    BSP_CON6_PERIPHERAL_CAMERA,
} bsp_con6_peripheral_t;

/* Detect what is attached to CON6.  Camera has priority when its I2C address
 * ACKs; otherwise the connector is treated as the ST7789T3 LCD variant. */
esp_err_t bsp_con6_detect(bsp_con6_peripheral_t *out_peripheral);

/* Set a single pin on the TCA9554A expander (configures it as output if it
 * is not already). Pins used by the BSP: P0/P1/P2 = RGB LED, P3 = LCD reset,
 * P6 = PA enable. */
esp_err_t bsp_ioexp_set_pin(uint8_t pin, bool level);

/* ---------- LCD (ST7789V3 on V02 20-pin FPC) ---------------------------- */
typedef void (*bsp_lcd_capture_cb_t)(void *user);

esp_err_t bsp_lcd_init(void);
esp_err_t bsp_lcd_release_for_camera(void);
esp_err_t bsp_lcd_reinit_after_camera(void);
esp_err_t bsp_lcd_show_test_pattern(void);
esp_err_t bsp_lcd_start_lvgl_demo(void);
esp_err_t bsp_lcd_start_camera_ui(bsp_lcd_capture_cb_t cb, void *user);
esp_err_t bsp_lcd_start_gateway_ui(void);
SemaphoreHandle_t bsp_lcd_get_lvgl_lock(void);
/* Return the sequence number assigned to the next LVGL refresh whose final
 * LCD transfer has not started yet. The token becomes complete only after the
 * final color-transfer DMA callback for that refresh has run. */
uint32_t bsp_lcd_next_frame_token(void);
bool bsp_lcd_frame_token_complete(uint32_t token);
esp_err_t bsp_lcd_set_camera_status(const char *text);
esp_err_t bsp_lcd_clear_camera_photo(void);
esp_err_t bsp_lcd_show_gray_photo(const uint8_t *gray,
                                  uint32_t width,
                                  uint32_t height);
esp_err_t bsp_lcd_show_yuv422_photo(const uint8_t *yuv422,
                                    uint32_t width,
                                    uint32_t height,
                                    uint32_t pixelformat);
esp_err_t bsp_lcd_show_rgb565_photo(const uint16_t *rgb565,
                                    uint32_t width,
                                    uint32_t height);

/* ---------- RGB LED ----------------------------------------------------- */
esp_err_t bsp_led_init(void);
void      bsp_led_set(bool r, bool g, bool b);

/* ---------- Audio (ES8311 codec + CST8302A PA + I2S) -------------------- */
esp_err_t bsp_audio_init(uint32_t sample_rate_hz);
esp_err_t bsp_audio_init_playback_only(uint32_t sample_rate_hz);
esp_err_t bsp_audio_pa_enable(bool on);                  /* PA enable via P6 */
esp_err_t bsp_audio_suspend(void);
esp_err_t bsp_audio_resume(void);
esp_err_t bsp_audio_set_volume(uint8_t volume_percent);  /* 0..100 for DAC  */
esp_err_t bsp_audio_set_mic_gain_db(uint8_t gain_db);    /* 0..42 PGA       */
esp_err_t bsp_audio_write(const void *buf, size_t bytes, size_t *out_written);
esp_err_t bsp_audio_read (void       *buf, size_t bytes, size_t *out_read);

/* ---------- Buttons ----------------------------------------------------- */
typedef enum {
    BSP_BTN_BOOT = 0,   /* GPIO0 strap button (K2)     */
    BSP_BTN_USER1,      /* K5 -> ~1.65 V on KEY_ADC    */
    BSP_BTN_VOL_DN,     /* K3 -> ~1.11 V               */
    BSP_BTN_PTT,        /* K6 -> ~0.82 V               */
    BSP_BTN_VOL_UP,     /* K4 -> ~2.41 V               */
    BSP_BTN_COUNT,
} bsp_btn_id_t;

typedef void (*bsp_btn_cb_t)(bsp_btn_id_t id, bool pressed, void *user);

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user);

/* Read the divider-compensated supply voltage from GPIO11/ADC2 in millivolts.
 * Returns a negative esp_err_t when ADC2 is unavailable. */
int bsp_vbat_read_mv(void);

/* Return the most recent successful voltage reading cached by bsp_vbat_read_mv(),
 * or 0 if no valid reading has been taken yet. Safe to call from the radio path
 * without touching ADC2 (lock-free read). */
uint16_t bsp_vbat_get_cached(void);

/* Start silent periodic voltage sampling; 0 selects the 15-second default.
 * Low-power nodes sample during CAD wake windows instead. */
esp_err_t bsp_vbat_monitor_start(uint32_t period_ms);

#ifdef __cplusplus
}
#endif
