/* BOOT uses GPIO0; K6, K5, K4, and K3 share the GPIO5 ADC ladder.
 * A 50 Hz task debounces the keys and reports state changes. */

#include "bsp.h"

#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "bsp_btn";

#define POLL_PERIOD_MS          20
#define DEBOUNCE_COUNT          2

#define ADC_UNIT                BSP_KEY_ADC_UNIT
#define ADC_CHAN                BSP_KEY_ADC_CHANNEL
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BITWIDTH            ADC_BITWIDTH_DEFAULT

/* Expected voltages (mV) per button. */
typedef struct {
    bsp_btn_id_t id;
    int          mv;
} adc_btn_t;

static const adc_btn_t s_adc_btns[] = {
    { BSP_BTN_PTT,    820  },   /* K6: hold to send FLRC voice */
    { BSP_BTN_VOL_DN, 1110 },   /* K3 */
    { BSP_BTN_USER1,  1650 },   /* K5: hold to record, release to play */
    { BSP_BTN_VOL_UP, 2410 },   /* K4 */
};

/* Maximum accepted error from the closest button target voltage. */
#define ADC_MAX_ERROR_MV        250
/* Above this threshold, no button is pressed. */
#define ADC_IDLE_THRESHOLD_MV   2900

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cal;
static bool                      s_cal_valid;

static bsp_btn_cb_t s_cb;
static void        *s_cb_user;

/* Debounced state per button id. */
static bool    s_btn_state  [BSP_BTN_COUNT];
static uint8_t s_btn_streak [BSP_BTN_COUNT];
static bool    s_btn_candidate[BSP_BTN_COUNT];

static int read_key_mv(void)
{
    int raw;
    if (adc_oneshot_read(s_adc, ADC_CHAN, &raw) != ESP_OK) return -1;
    if (s_cal_valid) {
        int mv;
        if (adc_cali_raw_to_voltage(s_cal, raw, &mv) == ESP_OK) return mv;
    }
    /* Fallback: raw * 3300 / 4095. Good enough for debouncing. */
    return raw * 3300 / ((1 << 12) - 1);
}

static bsp_btn_id_t classify_adc(int mv)
{
    if (mv < 0 || mv >= ADC_IDLE_THRESHOLD_MV) return BSP_BTN_COUNT;

    bsp_btn_id_t closest = BSP_BTN_COUNT;
    int closest_error = ADC_MAX_ERROR_MV + 1;
    for (size_t i = 0; i < sizeof(s_adc_btns) / sizeof(s_adc_btns[0]); i++) {
        int error = abs(mv - s_adc_btns[i].mv);
        if (error < closest_error) {
            closest = s_adc_btns[i].id;
            closest_error = error;
        } else if (error == closest_error) {
            closest = BSP_BTN_COUNT;
        }
    }
    return closest;
}

static void fire(bsp_btn_id_t id, bool pressed)
{
    ESP_LOGD(TAG, "btn %d %s", id, pressed ? "down" : "up");
    if (s_cb) s_cb(id, pressed, s_cb_user);
}

static void update_button(bsp_btn_id_t id, bool candidate)
{
    if (candidate == s_btn_candidate[id]) {
        if (s_btn_streak[id] < 0xFF) s_btn_streak[id]++;
    } else {
        s_btn_candidate[id] = candidate;
        s_btn_streak[id]    = 1;
    }
    if (s_btn_streak[id] >= DEBOUNCE_COUNT && candidate != s_btn_state[id]) {
        s_btn_state[id] = candidate;
        fire(id, candidate);
    }
}

static void poll_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();

    while (1) {
        /* GPIO boot button */
        bool boot_pressed = gpio_get_level(BSP_BOOT_KEY_GPIO) == 0;
        update_button(BSP_BTN_BOOT, boot_pressed);

        /* ADC ladder: at most one button can be pressed at a time, so we
         * set the active one true and everyone else false. */
        int mv = read_key_mv();
        bsp_btn_id_t active = classify_adc(mv);
        for (size_t i = 0; i < sizeof(s_adc_btns) / sizeof(s_adc_btns[0]); i++) {
            bsp_btn_id_t id = s_adc_btns[i].id;
            update_button(id, id == active);
        }

        vTaskDelayUntil(&next, pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

static esp_err_t adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t u_cfg = { .unit_id = ADC_UNIT };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&u_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t c_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHAN, &c_cfg),
                        TAG, "adc chan");

    /* Calibration is optional; fall back to linear mapping if unavailable. */
    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cal) == ESP_OK) {
        s_cal_valid = true;
    } else {
        ESP_LOGW(TAG, "ADC calibration not available, using raw mapping");
    }
    return ESP_OK;
}

esp_err_t bsp_button_init(bsp_btn_cb_t cb, void *user)
{
    s_cb      = cb;
    s_cb_user = user;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BSP_BOOT_KEY_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio0");
    ESP_RETURN_ON_ERROR(adc_setup(),     TAG, "adc");

    BaseType_t ok = xTaskCreate(poll_task, "bsp_btn", 3072, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
