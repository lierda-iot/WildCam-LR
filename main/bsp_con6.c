#include "bsp.h"

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#include "app_config.h"

static const char *TAG = "bsp_con6";

static esp_err_t con6_start_camera_mclk_for_detect(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = APP_SP0A39_MCLK_HZ,
        .clk_cfg = LEDC_USE_APB_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "detect mclk timer");

    ledc_channel_config_t ch = {
        .gpio_num = BSP_SP0A39_MCLK_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 1,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "detect mclk channel");
    return ESP_OK;
}

static esp_err_t con6_prepare_for_detect(void)
{
    ESP_RETURN_ON_ERROR(con6_start_camera_mclk_for_detect(), TAG, "camera detect mclk");
    ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, true), TAG, "pwdn high");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, false), TAG, "pwdn low");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t bsp_con6_detect(bsp_con6_peripheral_t *out_peripheral)
{
    if (!out_peripheral) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    ESP_RETURN_ON_ERROR(con6_prepare_for_detect(), TAG, "prepare");

    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t probe = i2c_master_probe(bus, BSP_I2C_ADDR_SP0A39, 50);
    if (probe != ESP_OK) {
        *out_peripheral = BSP_CON6_PERIPHERAL_LCD_ST7789;
        ESP_LOGI(TAG, "SP0A39 addr 0x%02X not present; selecting ST7789T3 LCD",
                 BSP_I2C_ADDR_SP0A39);
        return ESP_OK;
    }

    *out_peripheral = BSP_CON6_PERIPHERAL_CAMERA;
    ESP_LOGI(TAG, "SP0A39 addr 0x%02X ACK; selecting camera", BSP_I2C_ADDR_SP0A39);
    return ESP_OK;
}
