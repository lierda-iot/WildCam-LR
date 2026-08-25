#include "bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

static const char *TAG = "bsp_i2c";

/* TCA9554A register map */
#define TCA9554_REG_INPUT       0x00
#define TCA9554_REG_OUTPUT      0x01
#define TCA9554_REG_POLARITY    0x02
#define TCA9554_REG_CONFIG      0x03   /* 1 = input, 0 = output */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_ioexp;
static uint8_t s_ioexp_addr;

/* Cached expander state. Config defaults to all inputs. */
static uint8_t s_ioexp_cfg    = 0xFF;
static uint8_t s_ioexp_output = 0x00;
static SemaphoreHandle_t s_ioexp_lock;

static esp_err_t ioexp_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_ioexp, buf, sizeof(buf), 100);
}

static esp_err_t ioexp_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_ioexp, &reg, 1, val, 1, 100);
}

/* Quick hardware check on the two I2C pins. We temporarily configure them as
 * plain GPIOs and look at what the pull-ups deliver. If SDA/SCL can't rest
 * high, the I2C driver has no chance of completing a START. */
static void bus_pin_sanity(void)
{
    const gpio_num_t scl = BSP_I2C0_SCL_GPIO;
    const gpio_num_t sda = BSP_I2C0_SDA_GPIO;

    gpio_reset_pin(scl);
    gpio_reset_pin(sda);

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << scl) | (1ULL << sda),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
    ets_delay_us(200);
    int scl_idle = gpio_get_level(scl);
    int sda_idle = gpio_get_level(sda);

    /* Drive low briefly to check the drivers work, then release. */
    gpio_set_direction(scl, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(sda, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(scl, 0);
    gpio_set_level(sda, 0);
    ets_delay_us(200);
    int scl_lo = gpio_get_level(scl);
    int sda_lo = gpio_get_level(sda);
    gpio_set_direction(scl, GPIO_MODE_INPUT);
    gpio_set_direction(sda, GPIO_MODE_INPUT);
    ets_delay_us(500);
    int scl_rel = gpio_get_level(scl);
    int sda_rel = gpio_get_level(sda);

    ESP_LOGI(TAG, "pin check: SCL(%d) idle=%d low=%d released=%d",
             scl, scl_idle, scl_lo, scl_rel);
    ESP_LOGI(TAG, "pin check: SDA(%d) idle=%d low=%d released=%d",
             sda, sda_idle, sda_lo, sda_rel);
    if (scl_idle == 0 || sda_idle == 0) {
        ESP_LOGE(TAG, "  -> line stuck low at idle, check pull-up resistors "
                      "R33/R34 (2.2K to VDD_3V3) and anything shorting the bus");
    }
    if (scl_rel == 0 || sda_rel == 0) {
        ESP_LOGE(TAG, "  -> line does not recover after release, most likely "
                      "no pull-up at all");
    }

    /* Leave pins floating so the I2C driver can claim them cleanly. */
    gpio_reset_pin(scl);
    gpio_reset_pin(sda);
}

esp_err_t bsp_i2c_init(void)
{
    if (s_bus) return ESP_OK;

    bus_pin_sanity();

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = -1,                    /* auto pick a free port */
        .sda_io_num        = BSP_I2C0_SDA_GPIO,
        .scl_io_num        = BSP_I2C0_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

i2c_master_bus_handle_t bsp_i2c_bus(void)
{
    return s_bus;
}

esp_err_t bsp_i2c_scan(void)
{
    if (!s_bus) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "scanning I2C0...");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  device ACK @ 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "scan done, %d device(s)", found);
    return ESP_OK;
}

static esp_err_t ioexp_attach(void)
{
    if (s_ioexp) return ESP_OK;
    if (!s_bus) return ESP_ERR_INVALID_STATE;

    uint8_t addr = BSP_I2C_ADDR_IO_EXPANDER;
    esp_err_t probe = i2c_master_probe(s_bus, addr, 50);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "TCA9554A configured address 0x%02X did not ACK; scanning 0x38..0x3F",
                 addr);
        bool found = false;
        for (uint8_t candidate = 0x38; candidate <= 0x3F; ++candidate) {
            if (candidate == BSP_I2C_ADDR_TOUCH_FT6206) {
                ESP_LOGI(TAG, "skip 0x%02X while scanning TCA9554A: FT6206 touch address",
                         candidate);
                continue;
            }
            if (i2c_master_probe(s_bus, candidate, 50) == ESP_OK) {
                addr = candidate;
                found = true;
                ESP_LOGI(TAG, "TCA9554A candidate ACK @ 0x%02X", addr);
                break;
            }
            ESP_LOGI(TAG, "TCA9554A candidate 0x%02X not present", candidate);
        }
        if (!found) {
            ESP_LOGE(TAG, "no TCA9554A ACK in 0x38..0x3F after skipping FT6206 touch address");
            return ESP_ERR_NOT_FOUND;
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = BSP_I2C0_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_ioexp),
                        TAG, "ioexp add");
    s_ioexp_addr = addr;

    if (!s_ioexp_lock) {
        s_ioexp_lock = xSemaphoreCreateMutex();
        if (!s_ioexp_lock) return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ioexp_write(TCA9554_REG_OUTPUT, s_ioexp_output);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCA9554A write failed (%s) at 0x%02X",
                 esp_err_to_name(err), s_ioexp_addr);
        i2c_master_bus_rm_device(s_ioexp);
        s_ioexp = NULL;
        return err;
    }
    err = ioexp_write(TCA9554_REG_CONFIG, s_ioexp_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCA9554A config failed (%s) at 0x%02X",
                 esp_err_to_name(err), s_ioexp_addr);
        i2c_master_bus_rm_device(s_ioexp);
        s_ioexp = NULL;
        return err;
    }
    ESP_LOGI(TAG, "TCA9554A attached at 0x%02X", s_ioexp_addr);
    return ESP_OK;
}

esp_err_t bsp_i2c_reinit(void)
{
    if (s_ioexp) {
        i2c_master_bus_rm_device(s_ioexp);
        s_ioexp = NULL;
    }

    if (s_bus) {
        esp_err_t err = i2c_del_master_bus(s_bus);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2c del bus failed: %s", esp_err_to_name(err));
            i2c_master_bus_reset(s_bus);
            return err;
        }
        s_bus = NULL;
    }

    return bsp_i2c_init();
}

esp_err_t bsp_ioexp_set_pin(uint8_t pin, bool level)
{
    if (pin >= 8) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(ioexp_attach(), TAG, "attach");

    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_ioexp_lock, portMAX_DELAY);

    uint8_t mask = 1U << pin;
    bool cfg_dirty = (s_ioexp_cfg & mask) != 0;   /* was input -> output   */
    uint8_t new_out = level ? (s_ioexp_output | mask) : (s_ioexp_output & ~mask);

    if (new_out != s_ioexp_output || cfg_dirty) {
        err = ioexp_write(TCA9554_REG_OUTPUT, new_out);
        if (err == ESP_OK) s_ioexp_output = new_out;
    }
    if (err == ESP_OK && cfg_dirty) {
        uint8_t new_cfg = s_ioexp_cfg & ~mask;
        err = ioexp_write(TCA9554_REG_CONFIG, new_cfg);
        if (err == ESP_OK) s_ioexp_cfg = new_cfg;
    }

    if (pin == BSP_IO_EXP_LCD_RST_PIN ||
        pin == BSP_IO_EXP_LCD_BL_PIN ||
        pin == BSP_IO_EXP_TP_RST_PIN ||
        pin == BSP_IO_EXP_PA_MUTE_PIN ||
        pin == BSP_SP0A39_PWDN_IOEXP_PIN) {
        uint8_t input = 0, output = 0, cfg = 0;
        esp_err_t r0 = ioexp_read(TCA9554_REG_INPUT, &input);
        esp_err_t r1 = ioexp_read(TCA9554_REG_OUTPUT, &output);
        esp_err_t r2 = ioexp_read(TCA9554_REG_CONFIG, &cfg);
        if (r0 == ESP_OK && r1 == ESP_OK && r2 == ESP_OK) {
            ESP_LOGI(TAG, "TCA9554 P%d=%d: input=0x%02X output=0x%02X config=0x%02X",
                     pin, level, input, output, cfg);
        } else {
            ESP_LOGW(TAG, "TCA9554 P%d=%d: readback failed input=%s output=%s config=%s",
                     pin, level, esp_err_to_name(r0), esp_err_to_name(r1), esp_err_to_name(r2));
        }
    }

    xSemaphoreGive(s_ioexp_lock);
    return err;
}

/* ---------- RGB LED (common anode, high on expander pin = ON) ----------- */

esp_err_t bsp_led_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c");
    ESP_RETURN_ON_ERROR(ioexp_attach(), TAG, "ioexp");
    bsp_led_set(false, false, false);
    return ESP_OK;
}

void bsp_led_set(bool r, bool g, bool b)
{
    bsp_ioexp_set_pin(BSP_IO_EXP_LED_R_PIN, r);
    bsp_ioexp_set_pin(BSP_IO_EXP_LED_G_PIN, g);
    bsp_ioexp_set_pin(BSP_IO_EXP_LED_B_PIN, b);
}
