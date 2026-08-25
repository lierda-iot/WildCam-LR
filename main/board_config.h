/* Pin map for the L-LRMAM36-FANN4-DK01 development board.
 * Internal ESP32-S3/LR2021 wiring is configured by esp_lora_driver. */

#pragma once

#include "driver/gpio.h"

/* ---------- UART0 (CP2105 bridge - console/programming) ------------------ */
/* Routed through the module's TX0/RX0 pins; pins are handled by ROM bootloader
 * and the default ESP-IDF UART0 console, no need to set manually. */

/* ---------- UART2 (CP2105 bridge - secondary USB UART) ------------------- */
/* CP2105 ECI channel on USB Type-C1. Use this for high-volume binary streams
 * so UART0 can remain the ESP-IDF console/programming port. */
#define BSP_UART2_TX_GPIO           GPIO_NUM_47
#define BSP_UART2_RX_GPIO           GPIO_NUM_48

/* ---------- I2C0 (ES8311 codec, TCA9554A GPIO expander, touch panel) ----- */
#define BSP_I2C0_SCL_GPIO           GPIO_NUM_18
#define BSP_I2C0_SDA_GPIO           GPIO_NUM_16
#define BSP_I2C0_FREQ_HZ            (100 * 1000)   /* bring-up: slower bus improves margin */

#define BSP_I2C_ADDR_ES8311         0x18   /* Audio codec                     */
#define BSP_I2C_ADDR_IO_EXPANDER    0x39   /* TCA9554A, A0 changed            */
/* Touch panel I2C address depends on the controller fitted on the FPC. */

/* ---------- I2S0 (ES8311 codec audio path) ------------------------------- */
#define BSP_I2S0_MCLK_GPIO          GPIO_NUM_46
#define BSP_I2S0_BCLK_GPIO          GPIO_NUM_1
#define BSP_I2S0_LRCK_GPIO          GPIO_NUM_15
#define BSP_I2S0_DIN_GPIO           GPIO_NUM_45   /* ESP -> codec (SDIN)     */
#define BSP_I2S0_DOUT_GPIO          GPIO_NUM_14   /* codec -> ESP (SDOUT)    */

/* Audio PA (CST8302A) mute/enable is on the GPIO expander P6 (active high). */
#define BSP_IO_EXP_PA_MUTE_PIN      6

/* ---------- ST7789V3 LCD on V02 20-pin FPC ------------------------------- */
/* 4-wire SPI is enough for basic display. Some LCD FPC pins share GPIOs with
 * the 26-pin DVP camera FPC, so the BSP powers the camera down before driving
 * the LCD SPI bus. */
#define BSP_LCD_SPI_SCLK_GPIO       GPIO_NUM_44   /* LCD_CLK                 */
#define BSP_LCD_SPI_MOSI_GPIO       GPIO_NUM_43   /* LCD_MOSI                */
#define BSP_LCD_SPI_DC_GPIO         GPIO_NUM_48   /* LCD_DC                  */
#define BSP_LCD_SPI_CS_GPIO         GPIO_NUM_47   /* LCD_CS                  */
#define BSP_LCD_TE_GPIO             GPIO_NUM_NC   /* TE is not used for bring-up */
#define BSP_LCD_TOUCH_INT_GPIO      GPIO_NUM_12

#define BSP_IO_EXP_LCD_RST_PIN      3
#define BSP_IO_EXP_TP_RST_PIN       4
#define BSP_IO_EXP_LCD_BL_PIN       5

#define BSP_I2C_ADDR_TOUCH_CST816   0x15
#define BSP_I2C_ADDR_TOUCH_CST816_ALT 0x2A
#define BSP_I2C_ADDR_TOUCH_FT6206   0x38

/* ---------- Capacitive touch panel (shares I2C0) ------------------------- */
#define BSP_TP_INT_GPIO             BSP_LCD_TOUCH_INT_GPIO

/* ---------- SP0A39 DVP camera on V02 26-pin FPC -------------------------- */
/* PWDN is active high, controlled via IO expander P7. */
#define BSP_SP0A39_VSYNC_GPIO       GPIO_NUM_7
#define BSP_SP0A39_HSYNC_GPIO       GPIO_NUM_6
#define BSP_SP0A39_MCLK_GPIO        GPIO_NUM_3
#define BSP_SP0A39_PCLK_GPIO        GPIO_NUM_8
#define BSP_SP0A39_D0_GPIO          GPIO_NUM_2
#define BSP_SP0A39_D1_GPIO          GPIO_NUM_9
#define BSP_SP0A39_D2_GPIO          GPIO_NUM_13
#define BSP_SP0A39_D3_GPIO          GPIO_NUM_4
#define BSP_SP0A39_D4_GPIO          GPIO_NUM_43
#define BSP_SP0A39_D5_GPIO          GPIO_NUM_44
#define BSP_SP0A39_D6_GPIO          GPIO_NUM_48
#define BSP_SP0A39_D7_GPIO          GPIO_NUM_47
#define BSP_SP0A39_RESET_GPIO       GPIO_NUM_10
#define BSP_SP0A39_PWDN_IOEXP_PIN   7
#define BSP_I2C_ADDR_SP0A39         0x21

/* ---------- RGB status LED (driven by the GPIO expander) ----------------- */
#define BSP_IO_EXP_LED_G_PIN        0
#define BSP_IO_EXP_LED_R_PIN        1
#define BSP_IO_EXP_LED_B_PIN        2

/* ---------- User buttons ------------------------------------------------- */
/* K1 is reset and K2 is GPIO0/BOOT. GPIO5 reads the button ladder:
 * K6=PTT, K5=USER1, K4=VOL+, and K3=VOL-. */
#define BSP_KEY_ADC_GPIO            GPIO_NUM_5
#define BSP_KEY_ADC_UNIT            ADC_UNIT_1
#define BSP_KEY_ADC_CHANNEL         ADC_CHANNEL_4   /* GPIO5 = ADC1_CH4       */

#define BSP_BOOT_KEY_GPIO           GPIO_NUM_0

/* GPIO11 reads VBAT through a 1:1 divider. ADC2 reads may time out while Wi-Fi is active. */
#define BSP_VBAT_ADC_GPIO           GPIO_NUM_11
#define BSP_VBAT_ADC_UNIT           ADC_UNIT_2
#define BSP_VBAT_ADC_CHANNEL        ADC_CHANNEL_0   /* GPIO11 = ADC2_CH0      */
#define BSP_VBAT_DIVIDER_RATIO      2               /* R114/R115 = 1M/1M      */

/* LR2021 wiring: SPI2, NSS=GPIO39, SCLK=GPIO40, MOSI=GPIO41, MISO=GPIO42,
 * BUSY=GPIO17, NRST=GPIO38, and DIO7=GPIO21. These pins are internal. */
