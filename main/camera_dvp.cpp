#include "camera_uart.hpp"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "app_config.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "bsp.h"

#include "esp_cache.h"
#include "sp0a39_regs.h"

namespace {

constexpr const char *TAG = "camera_dvp";
constexpr TickType_t kPwdnSettleTicks = pdMS_TO_TICKS(100);
constexpr TickType_t kResetSettleTicks = pdMS_TO_TICKS(120);
constexpr uint32_t kFourccGrey = 0x59455247; // 'GREY'
constexpr uint32_t kFourccUyvy = 0x59565955; // 'UYVY'
constexpr uint32_t kFourccYuyv = 0x56595559; // 'YUYV'
constexpr uint32_t kFourccVyuy = 0x59555956; // 'VYUY'
constexpr uint32_t kFrameBytes = APP_CAMERA_FRAME_BYTES;
constexpr size_t kCaptureDmaBufferCount = 4;
constexpr int kCaptureFrameNumber = 2;

#if APP_CAMERA_COLOR_ENABLE
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_YUV422;
constexpr uint32_t kOutputPixelformat = kFourccUyvy;
#else
constexpr uint32_t kDvpCaptureWidth = APP_CAMERA_SENSOR_WIDTH;
constexpr cam_ctlr_color_t kDvpInputColor = CAM_CTLR_COLOR_GRAY8;
constexpr uint32_t kOutputPixelformat = kFourccGrey;
#endif

struct dvp_cb_ctx {
    uint8_t *buffers[kCaptureDmaBufferCount];
    uint8_t *safe_buffer;
    size_t buflen;
    volatile size_t received;
    volatile int frame_count;
    size_t next_buffer;
    volatile size_t last_received;
    SemaphoreHandle_t done_sem;
    volatile int capture_target;
    volatile uint8_t *captured_buffer;
};

static dvp_cb_ctx s_dvp_ctx;

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                       esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    if (ctx->capture_target > 0 && ctx->frame_count + 1 >= ctx->capture_target) {
        trans->buffer = ctx->safe_buffer;
        trans->buflen = ctx->buflen;
        return true;
    }
    if (ctx->captured_buffer != nullptr && ctx->capture_target == 0) {
        trans->buffer = ctx->safe_buffer;
        trans->buflen = ctx->buflen;
        return true;
    }
    trans->buffer = ctx->buffers[ctx->next_buffer];
    trans->buflen = ctx->buflen;
    ctx->next_buffer = (ctx->next_buffer + 1) % kCaptureDmaBufferCount;
    return trans->buffer != nullptr;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle,
                                        esp_cam_ctlr_trans_t *trans, void *user_data)
{
    dvp_cb_ctx *ctx = static_cast<dvp_cb_ctx *>(user_data);
    ctx->frame_count = ctx->frame_count + 1;
    ctx->last_received = trans->received_size;
    if (ctx->capture_target > 0 && ctx->frame_count >= ctx->capture_target) {
        ctx->received = trans->received_size;
        ctx->captured_buffer = static_cast<uint8_t *>(trans->buffer);
        ctx->capture_target = 0;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(ctx->done_sem, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    return false;
}

i2c_master_dev_handle_t s_sensor_dev = nullptr;

esp_err_t sensor_i2c_attach()
{
    if (s_sensor_dev) return ESP_OK;
    i2c_master_bus_handle_t bus = bsp_i2c_bus();
    if (!bus) return ESP_ERR_INVALID_STATE;

    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = APP_SP0A39_I2C_ADDR;
    cfg.scl_speed_hz = BSP_I2C0_FREQ_HZ;
    return i2c_master_bus_add_device(bus, &cfg, &s_sensor_dev);
}

void sensor_i2c_detach()
{
    if (s_sensor_dev) {
        i2c_master_bus_rm_device(s_sensor_dev);
        s_sensor_dev = nullptr;
    }
}

esp_err_t sensor_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_sensor_dev, buf, 2, 50);
}

esp_err_t sensor_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_sensor_dev, &reg, 1, val, 1, 50);
}

esp_err_t sensor_write_regs()
{
    const size_t count = sizeof(s_sp0a39_regs) / sizeof(s_sp0a39_regs[0]);
    for (size_t i = 0; i < count; i++) {
        esp_err_t ret = sensor_write_reg(s_sp0a39_regs[i][0], s_sp0a39_regs[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "reg write failed at [%u] 0x%02x=0x%02x: %s",
                     (unsigned)i, s_sp0a39_regs[i][0], s_sp0a39_regs[i][1],
                     esp_err_to_name(ret));
            return ret;
        }
    }
    ESP_LOGI(TAG, "SP0A39 register init done (%u regs)", (unsigned)count);
    return ESP_OK;
}

esp_err_t sensor_read_id()
{
    ESP_RETURN_ON_ERROR(sensor_write_reg(0xfd, 0x00), TAG, "page select");
    uint8_t id_h = 0, id_l = 0;
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x00, &id_h), TAG, "read id_h");
    ESP_RETURN_ON_ERROR(sensor_read_reg(0x01, &id_l), TAG, "read id_l");
    ESP_LOGI(TAG, "SP0A39 chip ID: 0x%02X%02X", id_h, id_l);
    return ESP_OK;
}

void log_sensor_output_regs()
{
    uint8_t p0_1c = 0;
    uint8_t p0_30 = 0;
    uint8_t p0_31 = 0;
    uint8_t p1_32 = 0;
    uint8_t p1_34 = 0;
    uint8_t p1_35 = 0;
    uint8_t p1_36 = 0;
    if (sensor_write_reg(0xfd, 0x00) == ESP_OK) {
        sensor_read_reg(0x1c, &p0_1c);
        sensor_read_reg(0x30, &p0_30);
        sensor_read_reg(0x31, &p0_31);
    }
    if (sensor_write_reg(0xfd, 0x01) == ESP_OK) {
        sensor_read_reg(0x32, &p1_32);
        sensor_read_reg(0x34, &p1_34);
        sensor_read_reg(0x35, &p1_35);
        sensor_read_reg(0x36, &p1_36);
    }
    sensor_write_reg(0xfd, 0x00);
    ESP_LOGI(TAG, "SP0A39 output regs: P0:1c=0x%02x P0:30=0x%02x P0:31=0x%02x P1:32=0x%02x P1:34=0x%02x P1:35=0x%02x P1:36=0x%02x",
             p0_1c, p0_30, p0_31, p1_32, p1_34, p1_35, p1_36);
}

void log_gpio_diagnostics()
{
    int vsync_changes = 0;
    int hsync_changes = 0;
    int pclk_changes = 0;
    int last_vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
    int last_hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
    int last_pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
    for (int i = 0; i < 100000; i++) {
        int vsync = gpio_get_level(BSP_SP0A39_VSYNC_GPIO);
        int hsync = gpio_get_level(BSP_SP0A39_HSYNC_GPIO);
        int pclk = gpio_get_level(BSP_SP0A39_PCLK_GPIO);
        if (vsync != last_vsync) {
            vsync_changes++;
            last_vsync = vsync;
        }
        if (hsync != last_hsync) {
            hsync_changes++;
            last_hsync = hsync;
        }
        if (pclk != last_pclk) {
            pclk_changes++;
            last_pclk = pclk;
        }
    }
    ESP_LOGI(TAG, "GPIO diagnostics: VSYNC changes=%d HSYNC changes=%d PCLK changes=%d HSYNC level=%d",
             vsync_changes, hsync_changes, pclk_changes, gpio_get_level(BSP_SP0A39_HSYNC_GPIO));
}

} // namespace

esp_err_t CameraUartStreamer::init()
{
    if (initialized_) return ESP_OK;
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");

    // Configure DVP data pins
    configure_camera_pins();

    // Create DVP controller first — it provides XCLK to the sensor
    esp_err_t ret = ensure_dvp_ready();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DVP create failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }

    // Now sensor has XCLK, wake it and configure registers
    set_pwdn(false);
    vTaskDelay(pdMS_TO_TICKS(200));
    ret = reset_sensor();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor reset failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = sensor_i2c_attach();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor i2c failed: %s", esp_err_to_name(ret));
        initialized_ = true;
        return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = sensor_read_id();
    if (ret == ESP_OK) {
        ret = sensor_write_regs();
    }
    if (ret == ESP_OK) {
        sensor_configured_ = true;
        sensor_awake_ = true;
        ESP_LOGI(TAG, "sensor initialized, DVP ready");
    } else {
        ESP_LOGW(TAG, "sensor regs failed: %s", esp_err_to_name(ret));
    }

    // Sensor configured — now start DVP permanently
    s_dvp_ctx.capture_target = 0;
    ret = esp_cam_ctlr_start(cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_start failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DVP running permanently");
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::start()
{
    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::set_pwdn(bool asserted)
{
    return bsp_ioexp_set_pin(BSP_SP0A39_PWDN_IOEXP_PIN, asserted);
}

esp_err_t CameraUartStreamer::reset_sensor()
{
    gpio_config_t reset = {};
    reset.pin_bit_mask = 1ULL << BSP_SP0A39_RESET_GPIO;
    reset.mode = GPIO_MODE_OUTPUT;
    reset.pull_up_en = GPIO_PULLUP_DISABLE;
    reset.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&reset), TAG, "camera reset gpio");
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(BSP_SP0A39_RESET_GPIO, 1);
    vTaskDelay(kResetSettleTicks);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::configure_camera_pins()
{
    uint64_t mask = 0;
    mask |= 1ULL << BSP_SP0A39_PCLK_GPIO;
    mask |= 1ULL << BSP_SP0A39_VSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_HSYNC_GPIO;
    mask |= 1ULL << BSP_SP0A39_D0_GPIO;
    mask |= 1ULL << BSP_SP0A39_D1_GPIO;
    mask |= 1ULL << BSP_SP0A39_D2_GPIO;
    mask |= 1ULL << BSP_SP0A39_D3_GPIO;
    mask |= 1ULL << BSP_SP0A39_D4_GPIO;
    mask |= 1ULL << BSP_SP0A39_D5_GPIO;
    mask |= 1ULL << BSP_SP0A39_D6_GPIO;
    mask |= 1ULL << BSP_SP0A39_D7_GPIO;
    gpio_config_t input_conf = {};
    input_conf.pin_bit_mask = mask;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&input_conf), TAG, "gpio input config");
    ESP_LOGI(TAG, "camera DVP pins configured as inputs");
    return ESP_OK;
}

esp_err_t CameraUartStreamer::power_down()
{
    if (dvp_ready_ && cam_handle_) {
        // Stop, disable, and delete the DVP controller so the next init can reclaim its GPIOs.
        esp_err_t se = esp_cam_ctlr_stop(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam stop: %s", esp_err_to_name(se));
        se = esp_cam_ctlr_disable(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam disable: %s", esp_err_to_name(se));
        se = esp_cam_ctlr_del(cam_handle_);
        if (se != ESP_OK) ESP_LOGW(TAG, "cam del: %s", esp_err_to_name(se));
        cam_handle_ = nullptr;
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            if (frame_bufs_[i]) { heap_caps_free(frame_bufs_[i]); frame_bufs_[i] = nullptr; }
        }
        if (safe_buf_) { heap_caps_free(safe_buf_); safe_buf_ = nullptr; }
        dvp_ready_ = false;
    }
    set_pwdn(true);
    gpio_reset_pin(BSP_SP0A39_RESET_GPIO);
    sensor_i2c_detach();
    sensor_configured_ = false;
    sensor_awake_ = false;
    return ESP_OK;
}

esp_err_t CameraUartStreamer::soft_power_down()
{
    set_pwdn(true);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::low_power_standby()
{
    // Release everything (DVP controller, DMA buffers, XCLK, sensor power) and
    // clear initialized_ so the next capture_frame()->init() rebuilds cleanly.
    esp_err_t ret = power_down();
    initialized_ = false;
    ESP_LOGI(TAG, "camera in low power standby (released)");
    return ret;
}

esp_err_t CameraUartStreamer::ensure_dvp_ready()
{
    if (dvp_ready_ && cam_handle_) return ESP_OK;

    esp_cam_ctlr_dvp_pin_config_t pins = {};
    pins.data_width = CAM_CTLR_DATA_WIDTH_8;
    pins.data_io[0] = BSP_SP0A39_D0_GPIO;
    pins.data_io[1] = BSP_SP0A39_D1_GPIO;
    pins.data_io[2] = BSP_SP0A39_D2_GPIO;
    pins.data_io[3] = BSP_SP0A39_D3_GPIO;
    pins.data_io[4] = BSP_SP0A39_D4_GPIO;
    pins.data_io[5] = BSP_SP0A39_D5_GPIO;
    pins.data_io[6] = BSP_SP0A39_D6_GPIO;
    pins.data_io[7] = BSP_SP0A39_D7_GPIO;
    pins.vsync_io = BSP_SP0A39_VSYNC_GPIO;
    pins.de_io = BSP_SP0A39_HSYNC_GPIO;
    pins.pclk_io = BSP_SP0A39_PCLK_GPIO;
    pins.xclk_io = BSP_SP0A39_MCLK_GPIO;

    esp_cam_ctlr_dvp_config_t dvp_cfg = {};
    dvp_cfg.ctlr_id = 0;
    dvp_cfg.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_cfg.h_res = kDvpCaptureWidth;
    dvp_cfg.v_res = APP_CAMERA_SENSOR_HEIGHT;
    dvp_cfg.input_data_color_type = kDvpInputColor;
    dvp_cfg.pin = &pins;
    dvp_cfg.xclk_freq = APP_SP0A39_MCLK_HZ;
    dvp_cfg.dma_burst_size = 64;
    dvp_cfg.bk_buffer_dis = 1;

    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_cfg, &cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_new_dvp_ctlr failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
        frame_bufs_[i] = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam_handle_, kFrameBytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
        if (!frame_bufs_[i]) {
            ESP_LOGE(TAG, "frame buffer alloc failed");
            for (size_t j = 0; j < kCaptureDmaBufferCount; ++j) {
                if (frame_bufs_[j]) { heap_caps_free(frame_bufs_[j]); frame_bufs_[j] = nullptr; }
            }
            esp_cam_ctlr_del(cam_handle_);
            cam_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }

    // Set up persistent context
    for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
        s_dvp_ctx.buffers[i] = frame_bufs_[i];
    }
    s_dvp_ctx.buflen = kFrameBytes;

    // Parking buffer used after a capture so the saved frame buffer is not reused.
    if (!safe_buf_) {
        safe_buf_ = static_cast<uint8_t *>(
            esp_cam_ctlr_alloc_buffer(cam_handle_, kFrameBytes,
                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
        if (!safe_buf_) {
            ESP_LOGE(TAG, "safe buffer alloc failed");
            for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
                if (frame_bufs_[i]) { heap_caps_free(frame_bufs_[i]); frame_bufs_[i] = nullptr; }
            }
            esp_cam_ctlr_del(cam_handle_);
            cam_handle_ = nullptr;
            return ESP_ERR_NO_MEM;
        }
    }
    s_dvp_ctx.safe_buffer = safe_buf_;

    if (!capture_sem_) {
        capture_sem_ = xSemaphoreCreateBinary();
    }
    s_dvp_ctx.done_sem = capture_sem_;

    // Register callbacks once (before enable)
    esp_cam_ctlr_evt_cbs_t cbs = {};
    cbs.on_get_new_trans = on_get_new_trans;
    cbs.on_trans_finished = on_trans_finished;
    ret = esp_cam_ctlr_register_event_callbacks(cam_handle_, &cbs, &s_dvp_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register cbs failed: %s", esp_err_to_name(ret));
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            if (frame_bufs_[i]) { heap_caps_free(frame_bufs_[i]); frame_bufs_[i] = nullptr; }
        }
        esp_cam_ctlr_del(cam_handle_);
        cam_handle_ = nullptr;
        return ret;
    }

    ret = esp_cam_ctlr_enable(cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_enable failed: %s", esp_err_to_name(ret));
        for (size_t i = 0; i < kCaptureDmaBufferCount; ++i) {
            if (frame_bufs_[i]) { heap_caps_free(frame_bufs_[i]); frame_bufs_[i] = nullptr; }
        }
        esp_cam_ctlr_del(cam_handle_);
        cam_handle_ = nullptr;
        return ret;
    }

    dvp_ready_ = true;
    ESP_LOGI(TAG, "DVP controller ready, XCLK on GPIO%d", BSP_SP0A39_MCLK_GPIO);
    return ESP_OK;
}

esp_err_t CameraUartStreamer::capture_frame(uint8_t **out_data,
                                            size_t *out_len,
                                            uint32_t *out_width,
                                            uint32_t *out_height,
                                            uint32_t *out_pixelformat)
{
    ESP_RETURN_ON_FALSE(out_data && out_len && out_width && out_height && out_pixelformat,
                        ESP_ERR_INVALID_ARG, TAG, "invalid args");
    *out_data = nullptr;
    *out_len = 0;
    *out_width = 0;
    *out_height = 0;
    *out_pixelformat = 0;

    ESP_RETURN_ON_ERROR(init(), TAG, "init");
    if (!dvp_ready_) {
        ESP_LOGE(TAG, "DVP not ready");
        return ESP_ERR_INVALID_STATE;
    }

    // Capture the second incoming frame, skipping the first for AE/AWB settling.
    s_dvp_ctx.received = 0;
    s_dvp_ctx.captured_buffer = nullptr;
    xQueueReset(capture_sem_);
    s_dvp_ctx.capture_target = s_dvp_ctx.frame_count + kCaptureFrameNumber;

    bool got_frame = xSemaphoreTake(capture_sem_, pdMS_TO_TICKS(5000)) == pdTRUE;

    if (got_frame && s_dvp_ctx.received >= kFrameBytes && s_dvp_ctx.captured_buffer) {
        ESP_LOGI(TAG, "captured frame: %u bytes (capture frame=%d, skipped=%d)",
                 (unsigned)s_dvp_ctx.received, kCaptureFrameNumber,
                 kCaptureFrameNumber - 1);

        uint8_t *copy = static_cast<uint8_t *>(
            heap_caps_malloc(kFrameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!copy) {
            copy = static_cast<uint8_t *>(
                heap_caps_malloc(kFrameBytes, MALLOC_CAP_8BIT));
        }
        if (!copy) {
            ESP_LOGE(TAG, "frame copy alloc failed");
            return ESP_ERR_NO_MEM;
        }
        esp_cache_msync((void *)s_dvp_ctx.captured_buffer, kFrameBytes,
                        ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        memcpy(copy, (void *)s_dvp_ctx.captured_buffer, kFrameBytes);

        s_dvp_ctx.captured_buffer = nullptr;

        *out_data = copy;
        *out_len = kFrameBytes;
        *out_width = APP_CAMERA_SENSOR_WIDTH;
        *out_height = APP_CAMERA_SENSOR_HEIGHT;
        *out_pixelformat = kOutputPixelformat;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "capture failed: got=%d frames=%d last=%u received=%u expected=%u",
             got_frame ? 1 : 0, s_dvp_ctx.frame_count, (unsigned)s_dvp_ctx.last_received,
             (unsigned)s_dvp_ctx.received, (unsigned)kFrameBytes);
    return got_frame ? ESP_ERR_INVALID_SIZE : ESP_ERR_TIMEOUT;
}
