/* Read VBAT through the GPIO11/ADC2 1:1 divider and return real millivolts.
 * Wi-Fi may block ADC2; failed samples preserve the last cached value. */

#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "bsp_vbat";

#define ADC_UNIT        BSP_VBAT_ADC_UNIT
#define ADC_CHAN        BSP_VBAT_ADC_CHANNEL
#define ADC_ATTEN       ADC_ATTEN_DB_12        /* full 0..3.1 V input range */
#define ADC_BITWIDTH    ADC_BITWIDTH_DEFAULT

#define VBAT_DEFAULT_PERIOD_MS  15000   /* non-low-power maintenance cadence */
#define VBAT_SAMPLES            8       /* raw conversions averaged per read */

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cal;
static bool                      s_cal_valid;
static bool                      s_ready;

/* Latest good reading in mV (real supply voltage, divider already undone).
 * Updated by every successful bsp_vbat_read_mv(); read lock-free via
 * bsp_vbat_get_cached() from the radio path. 0 = no valid reading yet. */
static volatile uint16_t s_cached_mv;

static esp_err_t vbat_adc_setup(void)
{
    if (s_ready) return ESP_OK;

    adc_oneshot_unit_init_cfg_t u_cfg = { .unit_id = ADC_UNIT };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&u_cfg, &s_adc), TAG, "adc unit");

    adc_oneshot_chan_cfg_t c_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHAN, &c_cfg),
                        TAG, "adc chan");

    /* Calibration is optional; fall back to a linear mapping if unavailable. */
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

    s_ready = true;
    return ESP_OK;
}

int bsp_vbat_read_mv(void)
{
    if (vbat_adc_setup() != ESP_OK) return -1;

    /* Average VBAT_SAMPLES raw conversions to smooth out ADC noise. Individual
     * reads may fail (Wi-Fi owning ADC2); we average only the ones that
     * succeed and report failure only if every sample failed. */
    int64_t   raw_sum = 0;
    int       ok      = 0;
    esp_err_t last_e  = ESP_OK;

    for (int i = 0; i < VBAT_SAMPLES; i++) {
        int raw;
        esp_err_t e = adc_oneshot_read(s_adc, ADC_CHAN, &raw);
        if (e == ESP_OK) {
            raw_sum += raw;
            ok++;
        } else {
            last_e = e;   /* ESP_ERR_TIMEOUT almost always means Wi-Fi owns ADC2 */
        }
    }
    if (ok == 0) return -last_e;

    int raw_avg = (int)(raw_sum / ok);

    int pin_mv;
    if (s_cal_valid) {
        if (adc_cali_raw_to_voltage(s_cal, raw_avg, &pin_mv) != ESP_OK) {
            pin_mv = raw_avg * 3300 / ((1 << 12) - 1);
        }
    } else {
        pin_mv = raw_avg * 3300 / ((1 << 12) - 1);
    }

    /* Undo the on-board 1M/1M divider to recover the real supply voltage. */
    int mv = pin_mv * BSP_VBAT_DIVIDER_RATIO;

    /* Refresh the cache so radio callers can grab it without touching ADC2. */
    s_cached_mv = (uint16_t)mv;
    return mv;
}

uint16_t bsp_vbat_get_cached(void)
{
    return s_cached_mv;
}

static void vbat_task(void *arg)
{
    uint32_t period_ms = (uint32_t)(uintptr_t)arg;
    TickType_t next = xTaskGetTickCount();

    // Silently sample every period_ms to keep bsp_vbat_get_cached() fresh for
    // the radio path (ImageStart first packet + periodic broadcast). No console
    // printing — the value reaches the gateway over the air, not the serial log.
    while (1) {
        (void)bsp_vbat_read_mv();
        vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms));
    }
}

esp_err_t bsp_vbat_monitor_start(uint32_t period_ms)
{
    if (period_ms == 0) period_ms = VBAT_DEFAULT_PERIOD_MS;
    ESP_RETURN_ON_ERROR(vbat_adc_setup(), TAG, "adc");

    BaseType_t ok = xTaskCreate(vbat_task, "bsp_vbat", 3072,
                                (void *)(uintptr_t)period_ms, 4, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
