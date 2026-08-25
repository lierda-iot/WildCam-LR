#include "bsp.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lvgl.h"

#include "app_config.h"

static const char *TAG = "bsp_lcd";

/* LR2021 uses SPI2 inside the module; keep the external LCD on SPI3. */
#define BSP_LCD_SPI_HOST SPI3_HOST
#define LCD_FLUSH_TIMEOUT_MS 200U
#define LCD_FLUSH_WATCHDOG_PERIOD_MS 25U

static esp_lcd_panel_io_handle_t s_lcd_io;
static esp_lcd_panel_handle_t s_lcd_panel;
static bool s_lcd_bus_ready;
static bool s_lcd_ready;
static bool s_lvgl_started;
static bool s_lcd_suspended;
static i2c_master_dev_handle_t s_touch;
static uint8_t s_touch_addr;
static lv_disp_drv_t *s_lvgl_disp_drv;
static SemaphoreHandle_t s_lvgl_lock;
static lv_obj_t *s_camera_status_label;
static lv_obj_t *s_camera_canvas;
static lv_color_t *s_camera_canvas_buf;
static bsp_lcd_capture_cb_t s_capture_cb;
static void *s_capture_user;
static lv_disp_drv_t *volatile s_pending_flush_drv;
static volatile uint32_t s_pending_flush_started_ms;
static volatile uint32_t s_pending_flush_seq;
static volatile int16_t s_pending_flush_y1;
static volatile int16_t s_pending_flush_y2;
static volatile uint32_t s_completed_flush_seq;
static volatile uint32_t s_refresh_started_seq;
static volatile uint32_t s_refresh_completed_seq;
static volatile uint32_t s_monitored_refresh_seq;
static volatile uint32_t s_monitored_last_flush_seq;
static esp_timer_handle_t s_flush_watchdog_timer;

typedef struct {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t len;
    uint16_t delay_ms;
} lcd_init_cmd_t;

static esp_err_t lcd_tx_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(s_lcd_io, ESP_ERR_INVALID_STATE, TAG, "lcd io not ready");
    esp_err_t ret = esp_lcd_panel_io_tx_param(s_lcd_io, cmd, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd_tx_cmd(0x%02X) FAILED: %s", cmd, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t touch_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_touch) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_touch, &reg, 1, data, len, 20);
}

static esp_err_t lcd_send_vendor_init(void)
{
    static const uint8_t madctl[] = {0x08};
    static const uint8_t colmod[] = {0x05};
    static const uint8_t porch[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    static const uint8_t gate[] = {0x35};
    static const uint8_t vcom[] = {0x36};
    static const uint8_t vdv_vrh_en[] = {0x01};
    static const uint8_t vrh[] = {0x13};
    static const uint8_t vdv[] = {0x20};
    static const uint8_t frame_rate[] = {0x0f};
    static const uint8_t gate_ctrl[] = {0xa1};
    static const uint8_t power[] = {0xa4, 0xa1};
    static const uint8_t gamma_pos[] = {
        0xf0, 0x08, 0x0e, 0x09, 0x08, 0x04, 0x2f,
        0x33, 0x45, 0x36, 0x13, 0x12, 0x2a, 0x2d,
    };
    static const uint8_t gamma_neg[] = {
        0xf0, 0x0e, 0x12, 0x0c, 0x0a, 0x15, 0x2e,
        0x32, 0x44, 0x39, 0x17, 0x18, 0x2b, 0x2f,
    };

    static const lcd_init_cmd_t cmds[] = {
        {LCD_CMD_MADCTL, madctl, sizeof(madctl), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
        {0xb2, porch, sizeof(porch), 0},
        {0xb7, gate, sizeof(gate), 0},
        {0xbb, vcom, sizeof(vcom), 0},
        {0xc2, vdv_vrh_en, sizeof(vdv_vrh_en), 0},
        {0xc3, vrh, sizeof(vrh), 0},
        {0xc4, vdv, sizeof(vdv), 0},
        {0xc6, frame_rate, sizeof(frame_rate), 0},
        {0xd6, gate_ctrl, sizeof(gate_ctrl), 0},
        {0xd0, power, sizeof(power), 0},
        {0xe0, gamma_pos, sizeof(gamma_pos), 0},
        {0xe1, gamma_neg, sizeof(gamma_neg), 0},
        {LCD_CMD_COLMOD, colmod, sizeof(colmod), 0},
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); ++i) {
        ESP_RETURN_ON_ERROR(lcd_tx_cmd(cmds[i].cmd, cmds[i].data, cmds[i].len),
                            TAG, "lcd cmd 0x%02x", cmds[i].cmd);
        if (cmds[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }

    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_SLPOUT, NULL, 0), TAG, "SLPOUT");
    vTaskDelay(pdMS_TO_TICKS(120));
    ESP_RETURN_ON_ERROR(lcd_tx_cmd(LCD_CMD_DISPON, NULL, 0), TAG, "DISPON");
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

static esp_err_t lcd_reset_gpio(void)
{
    esp_err_t err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_RST_PIN, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "lcd reset via IO expander unavailable: %s",
                 esp_err_to_name(err));
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_IO_EXP_LCD_RST_PIN, true),
                        TAG, "lcd reset high");
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(APP_LCD_LVGL_TICK_MS);
}

static esp_err_t lcd_draw_rgb565_bitmap(uint32_t x0, uint32_t y0,
                                        uint32_t x1, uint32_t y1,
                                        const uint16_t *pixels)
{
    ESP_RETURN_ON_FALSE(s_lcd_ready && pixels, ESP_ERR_INVALID_STATE, TAG,
                        "lcd not ready");
    ESP_RETURN_ON_FALSE(x1 > x0 && y1 > y0 &&
                        x1 <= APP_LCD_H_RES && y1 <= APP_LCD_V_RES,
                        ESP_ERR_INVALID_ARG, TAG, "invalid draw area");

    return esp_lcd_panel_draw_bitmap(s_lcd_panel, x0, y0, x1, y1, pixels);
}

static void lcd_flush_watchdog_cb(void *arg)
{
    (void)arg;

    lv_disp_drv_t *drv = s_pending_flush_drv;
    if (drv == NULL) {
        return;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t started_ms = s_pending_flush_started_ms;
    if ((uint32_t)(now_ms - started_ms) < LCD_FLUSH_TIMEOUT_MS) {
        return;
    }

    uint32_t seq = s_pending_flush_seq;
    int16_t y1 = s_pending_flush_y1;
    int16_t y2 = s_pending_flush_y2;

    /* Confirm that the same transfer is still pending. A late completion may
     * race with this timer, but a transfer that exceeded the deadline is no
     * longer safe to recover by releasing LVGL's draw buffer. */
    if (s_pending_flush_drv != drv || s_pending_flush_seq != seq) {
        return;
    }

    ESP_LOGE(TAG, "LCD flush timeout: seq=%lu y=%d..%d elapsed=%lums; restarting",
             (unsigned long)seq, y1, y2,
             (unsigned long)(now_ms - started_ms));
    esp_restart();
}

static esp_err_t lcd_flush_watchdog_start(void)
{
    if (s_flush_watchdog_timer != NULL) {
        return ESP_OK;
    }

    const esp_timer_create_args_t args = {
        .callback = lcd_flush_watchdog_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lcd_flush_wd",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_flush_watchdog_timer),
                        TAG, "create LCD flush watchdog");

    esp_err_t err = esp_timer_start_periodic(
        s_flush_watchdog_timer,
        (uint64_t)LCD_FLUSH_WATCHDOG_PERIOD_MS * 1000U);
    if (err != ESP_OK) {
        esp_timer_delete(s_flush_watchdog_timer);
        s_flush_watchdog_timer = NULL;
        return err;
    }
    return ESP_OK;
}

static bool sequence_reached(uint32_t current, uint32_t target)
{
    return (int32_t)(current - target) >= 0;
}

static void mark_flush_completed(uint32_t flush_seq)
{
    s_completed_flush_seq = flush_seq;

    uint32_t refresh_seq = s_monitored_refresh_seq;
    uint32_t last_flush_seq = s_monitored_last_flush_seq;
    if (refresh_seq != 0 && sequence_reached(flush_seq, last_flush_seq)) {
        s_refresh_completed_seq = refresh_seq;
    }
}

static bool lcd_color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io,
                                    esp_lcd_panel_io_event_data_t *event_data,
                                    void *user_ctx)
{
    (void)panel_io;
    (void)event_data;
    (void)user_ctx;

    lv_disp_drv_t *drv = s_pending_flush_drv;
    if (drv != NULL) {
        uint32_t flush_seq = s_pending_flush_seq;
        s_pending_flush_drv = NULL;
        lv_disp_flush_ready(drv);
        mark_flush_completed(flush_seq);
    }
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_map)
{
    uint32_t flush_seq = s_pending_flush_seq + 1U;
    s_pending_flush_seq = flush_seq;

    if (s_lcd_suspended || !s_lcd_ready) {
        lv_disp_flush_ready(drv);
        mark_flush_completed(flush_seq);
        return;
    }
    uint32_t pixel_count = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    uint16_t *px = (uint16_t *)color_map;
    for (uint32_t i = 0; i < pixel_count; i++) {
        px[i] = (px[i] >> 8) | (px[i] << 8);
    }
    s_pending_flush_started_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_pending_flush_y1 = area->y1;
    s_pending_flush_y2 = area->y2;
    s_pending_flush_drv = drv;
    esp_err_t err = lcd_draw_rgb565_bitmap(area->x1, area->y1,
                                           area->x2 + 1, area->y2 + 1,
                                           (const uint16_t *)color_map);
    if (err != ESP_OK) {
        if (s_pending_flush_drv == drv) {
            s_pending_flush_drv = NULL;
        }
        ESP_LOGE(TAG, "lvgl flush failed: %s", esp_err_to_name(err));
        lv_disp_flush_ready(drv);
        mark_flush_completed(flush_seq);
    }
}

static void lvgl_monitor_cb(lv_disp_drv_t *drv, uint32_t time_ms,
                            uint32_t pixel_count)
{
    (void)drv;
    (void)time_ms;
    (void)pixel_count;

    uint32_t refresh_seq = s_refresh_started_seq + 1U;
    uint32_t last_flush_seq = s_pending_flush_seq;

    /* Publish the final flush sequence before the refresh sequence. The DMA
     * callback can then close the same refresh whether it runs just before or
     * just after this monitor callback. */
    s_monitored_last_flush_seq = last_flush_seq;
    s_monitored_refresh_seq = refresh_seq;
    s_refresh_started_seq = refresh_seq;

    if (sequence_reached(s_completed_flush_seq, last_flush_seq)) {
        s_refresh_completed_seq = refresh_seq;
    }
}

static esp_err_t touch_reset(void)
{
    gpio_config_t intr = {
        .pin_bit_mask = 1ULL << BSP_LCD_TOUCH_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&intr), TAG, "touch int gpio");

    esp_err_t err = bsp_ioexp_set_pin(BSP_IO_EXP_TP_RST_PIN, false);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(bsp_ioexp_set_pin(BSP_IO_EXP_TP_RST_PIN, true),
                            TAG, "touch reset high");
        vTaskDelay(pdMS_TO_TICKS(80));
    } else {
        ESP_LOGW(TAG, "touch reset via IO expander unavailable: %s",
                 esp_err_to_name(err));
    }
    return ESP_OK;
}

static esp_err_t touch_attach(void)
{
    if (s_touch) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(touch_reset(), TAG, "touch reset");
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) {
        ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
        bus = bsp_i2c_bus();
    }
    if (!bus) {
        return ESP_ERR_INVALID_STATE;
    }

    static const uint8_t candidates[] = {
        BSP_I2C_ADDR_TOUCH_FT6206,
        BSP_I2C_ADDR_TOUCH_CST816,
        BSP_I2C_ADDR_TOUCH_CST816_ALT,
    };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        uint8_t addr = candidates[i];
        esp_err_t pe = i2c_master_probe(bus, addr, 50);
        if (pe != ESP_OK) {
            ESP_LOGI(TAG, "touch addr 0x%02X not present (%s)",
                     addr, esp_err_to_name(pe));
            continue;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = BSP_I2C0_FREQ_HZ,
        };
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_touch),
                            TAG, "touch add");
        s_touch_addr = addr;
        ESP_LOGI(TAG, "touch controller selected at 0x%02X (FT6206/CST816-compatible)",
                 s_touch_addr);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "no supported touch controller found on I2C");
    return ESP_ERR_NOT_FOUND;
}

static bool touch_read_point(lv_coord_t *x, lv_coord_t *y)
{
    uint8_t buf[5] = {};
    if (touch_read_reg(0x02, buf, sizeof(buf)) != ESP_OK) {
        return false;
    }

    uint8_t points = buf[0] & 0x0f;
    if (points == 0 || points > 2) {
        return false;
    }

    uint16_t raw_x = ((uint16_t)(buf[1] & 0x0f) << 8) | buf[2];
    uint16_t raw_y = ((uint16_t)(buf[3] & 0x0f) << 8) | buf[4];
    int32_t adj_x = (int32_t)raw_x - APP_LCD_X_GAP;
    int32_t adj_y = (int32_t)raw_y - APP_LCD_Y_GAP;
    if (adj_x < 0 || adj_y < 0 ||
        adj_x >= APP_LCD_H_RES || adj_y >= APP_LCD_V_RES) {
        return false;
    }

    *x = (lv_coord_t)adj_x;
    *y = (lv_coord_t)adj_y;
    return true;
}

static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    static lv_coord_t last_x;
    static lv_coord_t last_y;

    if (s_touch && touch_read_point(&last_x, &last_y)) {
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = last_x;
    data->point.y = last_y;
}

static void lvgl_create_demo_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "LR2021 Radio");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "ST7789T3 + LVGL + Touch");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xa7b0be), 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 42);

    lv_obj_t *bar = lv_bar_create(scr);
    lv_obj_set_size(bar, 190, 16);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 76);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 72, LV_ANIM_OFF);

    lv_obj_t *slider = lv_slider_create(scr);
    lv_obj_set_width(slider, 190);
    lv_obj_align(slider, LV_ALIGN_TOP_MID, 0, 118);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 35, LV_ANIM_OFF);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 42);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 158);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Touch");
    lv_obj_center(btn_label);

    lv_obj_t *status = lv_label_create(scr);
    lv_label_set_text(status, s_touch ? "Touch: ready" : "Touch: not found");
    lv_obj_set_style_text_color(status, lv_color_hex(0xd9e6f2), 0);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -18);
}

static void camera_btn_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_capture_cb) {
        s_capture_cb(s_capture_user);
    }
}

static esp_err_t lvgl_create_camera_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    s_camera_status_label = lv_label_create(scr);
    lv_label_set_text(s_camera_status_label, s_touch ? "Touch capture to take a photo" : "Touch not found");
    lv_obj_set_style_text_color(s_camera_status_label, lv_color_hex(0xd9e6f2), 0);
    lv_obj_set_style_bg_color(s_camera_status_label, lv_color_hex(0x101820), 0);
    lv_obj_set_style_bg_opa(s_camera_status_label, LV_OPA_COVER, 0);
    lv_obj_set_size(s_camera_status_label, APP_LCD_H_RES - 16, 20);
    lv_label_set_long_mode(s_camera_status_label, LV_LABEL_LONG_DOT);
    lv_obj_align(s_camera_status_label, LV_ALIGN_TOP_MID, 0, 10);

    const size_t canvas_pixels = APP_LCD_H_RES * APP_LCD_PHOTO_PREVIEW_H;
    if (!s_camera_canvas_buf) {
        s_camera_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_camera_canvas_buf) {
            s_camera_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                                   MALLOC_CAP_8BIT);
        }
        ESP_RETURN_ON_FALSE(s_camera_canvas_buf, ESP_ERR_NO_MEM, TAG,
                            "camera canvas alloc");
    }
    for (size_t i = 0; i < canvas_pixels; ++i) {
        s_camera_canvas_buf[i] = lv_color_hex(0x18232d);
    }

    s_camera_canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(s_camera_canvas, s_camera_canvas_buf,
                         APP_LCD_H_RES, APP_LCD_PHOTO_PREVIEW_H,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(s_camera_canvas, LV_ALIGN_TOP_MID, 0, 36);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 156, 42);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_add_event_cb(btn, camera_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Capture");
    lv_obj_center(btn_label);

    return ESP_OK;
}

static void lvgl_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
        }
        lv_timer_handler();
        if (s_lvgl_lock) {
            xSemaphoreGiveRecursive(s_lvgl_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(APP_LCD_LVGL_TASK_DELAY_MS));
    }
}

esp_err_t bsp_lcd_init(void)
{
    if (s_lcd_ready) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    esp_err_t bl_err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, false);
    if (bl_err != ESP_OK) {
        ESP_LOGW(TAG, "backlight off via IO expander unavailable: %s",
                 esp_err_to_name(bl_err));
    }

    if (!s_lcd_bus_ready) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = BSP_LCD_SPI_MOSI_GPIO,
            .miso_io_num = -1,
            .sclk_io_num = BSP_LCD_SPI_SCLK_GPIO,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS * sizeof(uint16_t),
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                            TAG, "spi bus");
        s_lcd_bus_ready = true;
        ESP_LOGI(TAG, "SPI3 bus initialized: SCLK=%d MOSI=%d",
                 BSP_LCD_SPI_SCLK_GPIO, BSP_LCD_SPI_MOSI_GPIO);
    }

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = BSP_LCD_SPI_CS_GPIO,
        .dc_gpio_num = BSP_LCD_SPI_DC_GPIO,
        .spi_mode = 0,
        .pclk_hz = APP_LCD_SPI_PCLK_HZ,
        .trans_queue_depth = APP_LCD_SPI_QUEUE_DEPTH,
        .on_color_trans_done = lcd_color_trans_done_cb,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST,
                                                 &io_cfg, &s_lcd_io),
                        TAG, "panel io");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(s_lcd_io, &panel_cfg, &s_lcd_panel),
                        TAG, "st7789 panel");

    esp_err_t ret;
    ret = lcd_reset_gpio();
    ESP_LOGI(TAG, "lcd_reset_gpio: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd hw reset");

    ret = esp_lcd_panel_reset(s_lcd_panel);
    ESP_LOGI(TAG, "esp_lcd_panel_reset: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd sw reset");

    ret = esp_lcd_panel_init(s_lcd_panel);
    ESP_LOGI(TAG, "esp_lcd_panel_init: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd init");

    vTaskDelay(pdMS_TO_TICKS(120));

    ret = esp_lcd_panel_set_gap(s_lcd_panel, APP_LCD_X_GAP, APP_LCD_Y_GAP);
    ESP_LOGI(TAG, "esp_lcd_panel_set_gap: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd gap");

    ret = esp_lcd_panel_invert_color(s_lcd_panel, true);
    ESP_LOGI(TAG, "esp_lcd_panel_invert_color: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd invert");

    ret = esp_lcd_panel_disp_on_off(s_lcd_panel, true);
    ESP_LOGI(TAG, "esp_lcd_panel_disp_on_off: %s", esp_err_to_name(ret));
    ESP_RETURN_ON_ERROR(ret, TAG, "lcd on");
    bl_err = bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, true);
    if (bl_err != ESP_OK) {
        ESP_LOGW(TAG, "backlight on via IO expander unavailable: %s",
                 esp_err_to_name(bl_err));
    }

    ESP_RETURN_ON_ERROR(lcd_flush_watchdog_start(), TAG,
                        "start LCD flush watchdog");
    s_lcd_ready = true;
    ESP_LOGI(TAG, "ST7789V3 LCD ready: %ux%u, SPI3 sclk=%d mosi=%d dc=%d cs=%d te=%d bl=P%d rst=P%d",
             APP_LCD_H_RES, APP_LCD_V_RES, BSP_LCD_SPI_SCLK_GPIO,
             BSP_LCD_SPI_MOSI_GPIO, BSP_LCD_SPI_DC_GPIO, BSP_LCD_SPI_CS_GPIO,
             BSP_LCD_TE_GPIO, BSP_IO_EXP_LCD_BL_PIN, BSP_IO_EXP_LCD_RST_PIN);
    return ESP_OK;
}

esp_err_t bsp_lcd_release_for_camera(void)
{
    s_lcd_suspended = true;
    vTaskDelay(pdMS_TO_TICKS(40));

    if (s_lcd_panel) {
        esp_lcd_panel_disp_on_off(s_lcd_panel, false);
        esp_lcd_panel_del(s_lcd_panel);
        s_lcd_panel = NULL;
        s_lcd_ready = false;
    }
    if (s_lcd_io) {
        esp_lcd_panel_io_del(s_lcd_io);
        s_lcd_io = NULL;
        s_lcd_ready = false;
    }
    if (s_lcd_bus_ready) {
        esp_err_t err = spi_bus_free(BSP_LCD_SPI_HOST);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "spi bus free failed: %s", esp_err_to_name(err));
        } else {
            s_lcd_bus_ready = false;
        }
    }
    bsp_ioexp_set_pin(BSP_IO_EXP_LCD_BL_PIN, false);
    return ESP_OK;
}

esp_err_t bsp_lcd_reinit_after_camera(void)
{
    ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd reinit");
    s_lcd_suspended = false;
    if (s_lvgl_lock) {
        xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
        lv_obj_invalidate(lv_scr_act());
        xSemaphoreGiveRecursive(s_lvgl_lock);
    }
    return ESP_OK;
}

esp_err_t bsp_lcd_show_test_pattern(void)
{
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    const size_t rows = APP_LCD_TEST_PATTERN_ROWS;
    const size_t pixels = APP_LCD_H_RES * rows;
    uint16_t *line = heap_caps_malloc(pixels * sizeof(uint16_t),
                                      MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!line) {
        return ESP_ERR_NO_MEM;
    }

    static const uint16_t colors[] = {
        0x00f8, 0xe007, 0x1f00, 0xe0ff, 0xff07, 0x1ff8, 0xffff, 0x0000,
    };
    for (uint32_t y = 0; y < APP_LCD_V_RES; y += rows) {
        uint32_t draw_rows = APP_LCD_V_RES - y;
        if (draw_rows > rows) {
            draw_rows = rows;
        }
        for (uint32_t row = 0; row < draw_rows; ++row) {
            for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
                uint32_t band = (x * (sizeof(colors) / sizeof(colors[0]))) / APP_LCD_H_RES;
                line[row * APP_LCD_H_RES + x] = colors[band];
            }
        }
        esp_err_t err = lcd_draw_rgb565_bitmap(0, y, APP_LCD_H_RES,
                                               y + draw_rows, line);
        if (y == 0) {
            ESP_LOGI(TAG, "first draw_bitmap (y=0..%lu): %s", (unsigned long)draw_rows, esp_err_to_name(err));
        }
        if (err != ESP_OK) {
            heap_caps_free(line);
            ESP_RETURN_ON_ERROR(err, TAG, "draw test");
        }
    }

    heap_caps_free(line);
    ESP_LOGI(TAG, "LCD test pattern drawn");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_lvgl_demo(void)
{
    if (s_lvgl_started) {
        return ESP_OK;
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lv_color_t *buf2 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = APP_LCD_H_RES;
    disp_drv.ver_res = APP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    s_lvgl_disp_drv = &disp_drv;
    lv_disp_drv_register(&disp_drv);

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    lvgl_create_demo_ui();

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "LVGL demo started%s", s_touch ? " with touch" : "");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_camera_ui(bsp_lcd_capture_cb_t cb, void *user)
{
    if (s_lvgl_started) {
        s_capture_cb = cb;
        s_capture_user = user;
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
            esp_err_t err = lvgl_create_camera_ui();
            xSemaphoreGiveRecursive(s_lvgl_lock);
            return err;
        }
        return ESP_OK;
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lv_color_t *buf2 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = APP_LCD_H_RES;
    disp_drv.ver_res = APP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    s_lvgl_disp_drv = &disp_drv;
    lv_disp_drv_register(&disp_drv);

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv);
    }

    s_capture_cb = cb;
    s_capture_user = user;
    ESP_RETURN_ON_ERROR(lvgl_create_camera_ui(), TAG, "camera ui");

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "camera LVGL UI started%s", s_touch ? " with touch" : "");
    return ESP_OK;
}

esp_err_t bsp_lcd_start_gateway_ui(void)
{
    if (s_lvgl_started) {
        extern esp_err_t ui_gw_init(void);
        if (s_lvgl_lock) {
            xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
            esp_err_t err = ui_gw_init();
            xSemaphoreGiveRecursive(s_lvgl_lock);
            return err;
        }
        return ui_gw_init();
    }
    if (!s_lcd_ready) {
        ESP_RETURN_ON_ERROR(bsp_lcd_init(), TAG, "lcd init");
    }

    if (!s_lvgl_lock) {
        s_lvgl_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lvgl_lock, ESP_ERR_NO_MEM, TAG, "lvgl lock");
    }

    lv_init();

    const size_t pixels = APP_LCD_H_RES * APP_LCD_LVGL_BUFFER_ROWS;
    lv_color_t *buf1 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lv_color_t *buf2 = heap_caps_malloc(pixels * sizeof(lv_color_t),
                                        MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t draw_buf_gw;
    lv_disp_draw_buf_init(&draw_buf_gw, buf1, buf2, pixels);

    static lv_disp_drv_t disp_drv_gw;
    lv_disp_drv_init(&disp_drv_gw);
    disp_drv_gw.hor_res = APP_LCD_H_RES;
    disp_drv_gw.ver_res = APP_LCD_V_RES;
    disp_drv_gw.flush_cb = lvgl_flush_cb;
    disp_drv_gw.monitor_cb = lvgl_monitor_cb;
    disp_drv_gw.draw_buf = &draw_buf_gw;
    s_lvgl_disp_drv = &disp_drv_gw;
    lv_disp_drv_register(&disp_drv_gw);

    esp_err_t touch_err = touch_attach();
    if (touch_err == ESP_OK) {
        static lv_indev_drv_t indev_drv_gw;
        lv_indev_drv_init(&indev_drv_gw);
        indev_drv_gw.type = LV_INDEV_TYPE_POINTER;
        indev_drv_gw.read_cb = lvgl_touch_read_cb;
        lv_indev_drv_register(&indev_drv_gw);
    }

    extern esp_err_t ui_gw_init(void);
    ESP_RETURN_ON_ERROR(ui_gw_init(), TAG, "gateway ui");

    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG,
                        "lvgl tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer,
                                                 APP_LCD_LVGL_TICK_MS * 1000U),
                        TAG, "lvgl tick start");

    BaseType_t ok = xTaskCreatePinnedToCore(lvgl_task, "lvgl",
                                            APP_LCD_LVGL_TASK_STACK_BYTES, NULL,
                                            APP_LCD_LVGL_TASK_PRIORITY, NULL,
                                            APP_LCD_LVGL_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_lvgl_started = true;
    ESP_LOGI(TAG, "Gateway LVGL UI started");
    return ESP_OK;
}

SemaphoreHandle_t bsp_lcd_get_lvgl_lock(void)
{
    return s_lvgl_lock;
}

uint32_t bsp_lcd_next_frame_token(void)
{
    return s_refresh_started_seq + 1U;
}

bool bsp_lcd_frame_token_complete(uint32_t token)
{
    return sequence_reached(s_refresh_completed_seq, token);
}

esp_err_t bsp_lcd_set_camera_status(const char *text)
{
    if (!s_lvgl_started || !s_camera_status_label || !text) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    lv_label_set_text(s_camera_status_label, text);
    lv_obj_invalidate(s_camera_status_label);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_clear_camera_photo(void)
{
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    const size_t canvas_pixels = APP_LCD_H_RES * APP_LCD_PHOTO_PREVIEW_H;
    for (size_t i = 0; i < canvas_pixels; ++i) {
        s_camera_canvas_buf[i] = lv_color_hex(0x18232d);
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_show_gray_photo(const uint8_t *gray,
                                  uint32_t width,
                                  uint32_t height)
{
    ESP_RETURN_ON_FALSE(gray && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid gray frame");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint8_t *src = gray + src_y * width;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            uint8_t v = src[src_x];
            dst[x] = lv_color_make(v, v, v);
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

static inline uint8_t yuv_clamp(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static bool yuv422_get_pair(const uint8_t *p,
                            uint32_t pixelformat,
                            uint8_t *y0,
                            uint8_t *u,
                            uint8_t *y1,
                            uint8_t *v)
{
    switch (pixelformat) {
    case 0x56595559: // 'YUYV'
        *y0 = p[0]; *u = p[1]; *y1 = p[2]; *v = p[3];
        return true;
    case 0x59565955: // 'UYVY'
        *u = p[0]; *y0 = p[1]; *v = p[2]; *y1 = p[3];
        return true;
    case 0x55595659: // 'YVYU'
        *y0 = p[0]; *v = p[1]; *y1 = p[2]; *u = p[3];
        return true;
    case 0x59555956: // 'VYUY'
        *v = p[0]; *y0 = p[1]; *u = p[2]; *y1 = p[3];
        return true;
    default:
        return false;
    }
}

esp_err_t bsp_lcd_show_rgb565_photo(const uint16_t *rgb565,
                                    uint32_t width,
                                    uint32_t height)
{
    ESP_RETURN_ON_FALSE(rgb565 && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid rgb565 frame");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint16_t *src = rgb565 + src_y * width;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            dst[x].full = src[src_x];
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}

esp_err_t bsp_lcd_show_yuv422_photo(const uint8_t *yuv422,
                                    uint32_t width,
                                    uint32_t height,
                                    uint32_t pixelformat)
{
    ESP_RETURN_ON_FALSE(yuv422 && width && height, ESP_ERR_INVALID_ARG, TAG,
                        "invalid yuv frame");
    ESP_RETURN_ON_FALSE(pixelformat == 0x56595559 || pixelformat == 0x59565955 ||
                        pixelformat == 0x55595659 || pixelformat == 0x59555956,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported yuv fourcc");
    ESP_RETURN_ON_FALSE(s_lvgl_started && s_camera_canvas && s_camera_canvas_buf,
                        ESP_ERR_INVALID_STATE, TAG, "camera canvas not ready");

    xSemaphoreTakeRecursive(s_lvgl_lock, portMAX_DELAY);
    for (uint32_t y = 0; y < APP_LCD_PHOTO_PREVIEW_H; ++y) {
        uint32_t src_y = (y * height) / APP_LCD_PHOTO_PREVIEW_H;
        const uint8_t *row = yuv422 + src_y * width * 2;
        lv_color_t *dst = s_camera_canvas_buf + y * APP_LCD_H_RES;
        for (uint32_t x = 0; x < APP_LCD_H_RES; ++x) {
            uint32_t src_x = (x * width) / APP_LCD_H_RES;
            uint32_t pair = src_x & ~1u;
            uint8_t y0 = 0;
            uint8_t u = 0;
            uint8_t y1 = 0;
            uint8_t v = 0;
            yuv422_get_pair(row + pair * 2, pixelformat, &y0, &u, &y1, &v);
            uint8_t lum = (src_x & 1) ? y1 : y0;
            int c = lum - 16;
            int d = u - 128;
            int e = v - 128;
            uint8_t r = yuv_clamp((298 * c + 409 * e + 128) >> 8);
            uint8_t g = yuv_clamp((298 * c - 100 * d - 208 * e + 128) >> 8);
            uint8_t b = yuv_clamp((298 * c + 516 * d + 128) >> 8);
            dst[x] = lv_color_make(r, g, b);
        }
    }
    lv_obj_invalidate(s_camera_canvas);
    xSemaphoreGiveRecursive(s_lvgl_lock);
    return ESP_OK;
}
