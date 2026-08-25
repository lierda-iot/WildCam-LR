#pragma once

/* Application-level configuration for audio, codec, radio, and PTT. */

/* ----- Audio capture/playback ------------------------------------------------ */

#define APP_AUDIO_FEATURES_ENABLE       1
#define APP_RADIO_FEATURES_ENABLE       1
#define APP_RADIO_HW_INIT_ENABLE        1
#define APP_RADIO_AUTO_RX_ENABLE        1
#define APP_RADIO_TASKS_ENABLE          1

/* Set to 0 to disable STA/provisioning — AP-only gallery mode */
#define APP_WIFI_STA_ENABLE             0

/* ES8311/I2S sample rate used by both the microphone and speaker paths. */
#define APP_AUDIO_SAMPLE_RATE_HZ        16000U

/* The voice path is mono; stereo I2S samples are mixed down before encoding. */
#define APP_AUDIO_CHANNELS              1U

/* PCM sample format passed to the Opus encoder and produced by the decoder. */
#define APP_AUDIO_BITS_PER_SAMPLE       16U

/* 10 ms reduces per-frame encode blocking and makes packet loss less audible. */
#define APP_AUDIO_FRAME_MS              10U

/* Number of mono PCM samples in one Opus frame at APP_AUDIO_SAMPLE_RATE_HZ. */
#define APP_AUDIO_FRAME_SAMPLES         ((APP_AUDIO_SAMPLE_RATE_HZ * APP_AUDIO_FRAME_MS) / 1000U)

/* Number of bytes in one mono 16-bit PCM frame before Opus compression. */
#define APP_AUDIO_FRAME_BYTES           (APP_AUDIO_FRAME_SAMPLES * (APP_AUDIO_BITS_PER_SAMPLE / 8U))

/* Size used for blocking I2S reads/writes in the diagnostic local audio path. */
#define APP_AUDIO_IO_CHUNK_BYTES        2048U

/* ----- Opus voice codec ------------------------------------------------------ */

/* Low-delay mode avoids the heavier SILK VOIP path that was hitting WDT. */
#define APP_OPUS_APPLICATION            OPUS_APPLICATION_RESTRICTED_LOWDELAY

/* 16 kHz speech has enough FLRC budget; 24 kbps improves low-delay quality. */
#define APP_OPUS_BITRATE_BPS            24000

/* Fixed bitrate makes packet sizing and radio scheduling easier to debug. */
#define APP_OPUS_USE_VBR                0

/* Keep encoder CPU bounded on ESP32-S3; raise only after WDT/headroom tests. */
#define APP_OPUS_COMPLEXITY             0

/* Keep DTX off for the first bring-up so packet timing remains predictable. */
#define APP_OPUS_USE_DTX                0

/* Reserve enough room for a 20 ms voice packet at the target bitrate plus slack. */
#define APP_OPUS_MAX_PACKET_BYTES       96U

/* ----- FLRC radio link ------------------------------------------------------- */

/* Default center frequency and ten fixed pseudo-random channel presets. */
#define APP_FLRC_FREQUENCY_HZ           921470000UL
#define APP_FLRC_FREQUENCY_PRESET_COUNT 10U
#define APP_FLRC_FREQUENCY_PRESETS_HZ   { \
    921470000UL, 922430000UL, 923480000UL, 924390000UL, 925490000UL, \
    926470000UL, 927500000UL, 928440000UL, 929420000UL, 930500000UL  \
}
#define APP_FREQUENCY_CONFIRM_TIMEOUT_MS 500U
#define APP_FREQUENCY_CONFIRM_RETRIES   3U
#define APP_FREQUENCY_ACK_SETTLE_MS     200U
#define APP_FREQUENCY_ROLLBACK_MS       2000U

/* LR2021 FLRC high-rate mode requested for this project.
 * Informational only (used in logs): the upgraded driver selects the rate via
 * the raw_bit_rate enum below, which already implies the paired bandwidth. */
#define APP_FLRC_BITRATE_BPS            2600000UL

/* Double-sided FLRC bandwidth paired with 2.6 Mbps. Informational only now;
 * bandwidth is bundled into APP_FLRC_RAW_BIT_RATE in the upgraded driver. */
#define APP_FLRC_BANDWIDTH_HZ           2666000UL

/* Raw bit-rate enum for the upgraded LR2021 driver. This replaces the old
 * br_in_bps/bw_dsb_in_hz numeric fields; 2.6 Mbps maps to the value below. */
#define APP_FLRC_RAW_BIT_RATE           RAL_FLRC_RAW_BIT_RATE_2_600_MBPS

/* Preamble length enum for the upgraded driver (was preamble_len_in_bits=32). */
#define APP_FLRC_PREAMBLE_LEN           RAL_FLRC_PREAMBLE_LENGTH_32_BITS

/* TX power for bench tests. 22 dBm saturates nearby receivers; use 10 for close range. */
#define APP_FLRC_TX_POWER_DBM           22

/* Use FEC during early tests; switch to RAL_FLRC_CR_1_1 only after range tests. */
#define APP_FLRC_CODING_RATE            RAL_FLRC_CR_3_4

/* BT=0.5 keeps spectrum cleaner than no shaping at high FLRC data rates. */
#define APP_FLRC_PULSE_SHAPE            RAL_FLRC_PULSE_SHAPE_BT_05

/* Use 511-byte FLRC packets for image bursts; voice remains frame-limited.
 * RX buffers must accept the full payload to prevent fragment truncation. */
#define APP_FLRC_MAX_PAYLOAD_BYTES      511U

/* FLRC BURST per-packet on-air payload for image bulk transfer. 2x511 = 1022B
 * fits the 1024-byte FIFO, letting us prefill 2 packets before the first TX. */
#define APP_FLRC_BURST_PAYLOAD_LEN      511U

/* Pack several 10 ms Opus frames per FLRC packet to reduce TX overhead while
 * keeping each radio packet to about 50 ms of audio. */
#define APP_FLRC_OPUS_FRAMES_PER_PACKET 5U

/* RX timeout used by the packet receiver before it re-arms listening. */
#define APP_FLRC_RX_TIMEOUT_MS          100U

/* Extra gap after each voice TX packet. Keep at 0 for continuous 20 ms audio. */
#define APP_FLRC_VOICE_TX_GAP_MS        0U

/* Log one voice frame every N packets to avoid flooding the serial console. */
#define APP_VOICE_LOG_EVERY_N           25U

/* Poll period for the RAC engine task. Keep well below one audio frame so TX
 * done/RX done is handled without stretching the 20 ms voice cadence. */
#define APP_RADIO_TASK_POLL_MS          2U

/* FreeRTOS priority for the radio engine task. */
#define APP_RADIO_TASK_PRIORITY         4

/* Stack for RAC callbacks plus Opus encode/decode. Opus needs much more than
 * the old ping-only path, so keep this conservative during bring-up. */
#define APP_RADIO_TASK_STACK_BYTES      16384U

/* Keep direct RAL radio control on CPU0. */
#define APP_RADIO_TASK_CORE             0

/* Dedicated sync word for this project's FLRC test/audio packets. */
#define APP_FLRC_SYNC_WORD_0            0x4CU
#define APP_FLRC_SYNC_WORD_1            0x52U
#define APP_FLRC_SYNC_WORD_2            0x32U
#define APP_FLRC_SYNC_WORD_3            0x31U

/* ----- PTT behavior ---------------------------------------------------------- */

/* Button used as push-to-talk. K6 is the ADC-ladder key near 0.82 V. */
#define APP_PTT_BUTTON                  BSP_BTN_PTT

/* Receiver-side jitter buffer target before starting speaker playback. */
#define APP_RX_JITTER_BUFFER_MS         60U

/* Number of encoded voice frames to queue before starting speaker playback. */
#define APP_RX_JITTER_FRAMES            ((APP_RX_JITTER_BUFFER_MS + APP_AUDIO_FRAME_MS - 1U) / APP_AUDIO_FRAME_MS)

/* Conceal one missing aggregated FLRC packet before resyncing. */
#define APP_RX_MAX_PLC_FRAMES           APP_FLRC_OPUS_FRAMES_PER_PACKET

/* Stop playback if no voice packet arrives within this interval. */
#define APP_RX_AUDIO_TIMEOUT_MS         200U

/* Number of encoded voice packets buffered between radio RX and playback. */
#define APP_VOICE_RX_QUEUE_LEN          12U

/* Number of encoded voice frames buffered between microphone and radio TX. */
#define APP_VOICE_TX_QUEUE_LEN          25U

/* Only print one TX queue overflow warning every N dropped voice frames. */
#define APP_TX_DROP_LOG_EVERY_N         25U

/* Opus decode plus I2S write run here so radio RX can re-arm quickly. */
#define APP_VOICE_PLAY_TASK_PRIORITY    5
#define APP_VOICE_PLAY_TASK_STACK_BYTES 16384U

/* Run Opus decode/playback away from direct radio control. */
#define APP_VOICE_PLAY_TASK_CORE        1

/* Opus encode plus I2S read run here so RAC polling is not blocked by audio. */
#define APP_VOICE_TX_TASK_PRIORITY      5
#define APP_VOICE_TX_TASK_STACK_BYTES   16384U

/* Keep Opus encode away from the radio/control task on CPU0. */
#define APP_VOICE_TX_TASK_CORE          1

/* ----- Audio DSP (noise suppression / voice enhancement) -------------------- */

#define APP_AUDIO_DSP_ENABLE              1
#define APP_AUDIO_DSP_PREEMPH_ALPHA_Q15   31785   /* 0.97 in Q15 — speech HF boost */
#define APP_AUDIO_DSP_NOISE_GATE_THRESH   200     /* RMS threshold to open gate */
#define APP_AUDIO_DSP_NOISE_GATE_ATTACK   3       /* frames above thresh to open */
#define APP_AUDIO_DSP_NOISE_GATE_RELEASE  4       /* smoothing shift (larger = slower) */
#define APP_AUDIO_DSP_NS_FLOOR_ADAPT      6       /* noise floor rise speed shift (larger = slower) */
#define APP_AUDIO_DSP_NS_FLOOR_DECAY      2       /* noise floor drop speed shift (larger = slower) */
#define APP_AUDIO_DSP_NS_MIN_GAIN_Q15     3277    /* minimum suppression gain ~0.10 (full atten floor) */
#define APP_AUDIO_DSP_NS_OVERSUBTRACT     3       /* over-subtraction factor shift (1=2x, 2=4x, 3=8x) */
#define APP_AUDIO_DSP_LIMITER_THRESHOLD   30000   /* soft-clip knee on RX playback */
#define APP_AUDIO_DSP_TX_MUTE_FRAMES      5       /* mute first N frames after PTT to suppress echo tail */
#define APP_AUDIO_DSP_AGC_TARGET_Q15      22000   /* target RMS level in Q15 (~0.67 FS) */
#define APP_AUDIO_DSP_AGC_MAX_GAIN_Q15    (4 << 15) /* max 4x amplification */
#define APP_AUDIO_DSP_AGC_ATTACK_SHIFT    3       /* gain ramp-up speed (larger = slower) */
#define APP_AUDIO_DSP_AGC_RELEASE_SHIFT   5       /* gain ramp-down speed (larger = slower) */

/* ----- Local diagnostic tones ------------------------------------------------ */

#define APP_BEEP_FREQ_HZ                1800
#define APP_BEEP_ON_MS                  140
#define APP_BEEP_GAP_MS                 90
#define APP_BEEP_TAIL_MS                80
#define APP_BEEP_AMP                    12000
#define APP_PA_SETTLE_MS                40
/* Set to 0 to make boot silent. */
#define APP_STARTUP_CHIME_ENABLE        1

/* Short low-amplitude boot chime; avoids the old 5 second square-wave tone. */
#define APP_STARTUP_CHIME_FREQ1_HZ      660
#define APP_STARTUP_CHIME_FREQ2_HZ      880
#define APP_STARTUP_CHIME_TONE_MS       90
#define APP_STARTUP_CHIME_GAP_MS        35
#define APP_STARTUP_CHIME_AMP           3500

/* ----- Camera/LCD bring-up -------------------------------------------------- */

/* ----- Auto-capture timer ------------------------------------------------ */

#define APP_AUTO_CAPTURE_DEFAULT_SEC    300U   /* 5 minutes */
#define APP_AUTO_CAPTURE_MIN_SEC        10U    /* 10 seconds minimum */
#define APP_CFG_KEY_INTERVAL            0x01
#define APP_CFG_KEY_INTER_PACKET        0x02
#define APP_CFG_KEY_AUDIO_CLIP          0x03
#define APP_CFG_KEY_SOUND_TRIGGER       0x04
#define APP_CFG_KEY_PIR_TRIGGER         0x05
#define APP_CFG_KEY_VOICE_ALARM         0x06
#define APP_CFG_KEY_LOW_POWER           0x07
#define APP_CFG_KEY_FREQUENCY           0x08
#define APP_AUDIO_CLIP_DEFAULT_ENABLE   0


#define APP_TRIGGER_COOLDOWN_SEC        15U
#define APP_AUDIO_CAPTURE_COOLDOWN_MS   6000U
#define APP_PIR_GPIO                    GPIO_NUM_12
#define APP_SOUND_TRIGGER_THRESH_LOW    1200
#define APP_SOUND_TRIGGER_THRESH_MED    5000
#define APP_SOUND_TRIGGER_THRESH_HIGH   12000

/* Delay sound-trigger capture so the Opus ring includes post-trigger audio.
 * PIR capture is dispatched immediately. */
#define APP_SOUND_TRIGGER_DELAY_MS      500U

/* ----- Voice alarm (speaker playback on trigger) ------------------------- */

#define APP_VOICE_ALARM_ENABLE          1
#define APP_VOICE_ALARM_VOLUME_PERCENT  80U   /* gateway level 8 → 80% → REG32=0xCC */

/* ----- Audio ring buffer (pre-capture retrospective recording) ----------- */

#define APP_AUDIO_RINGBUF_SECONDS       5U
#define APP_AUDIO_RINGBUF_SAMPLES       (APP_AUDIO_SAMPLE_RATE_HZ * APP_AUDIO_RINGBUF_SECONDS)
#define APP_AUDIO_RINGBUF_BYTES         (APP_AUDIO_RINGBUF_SAMPLES * 2U)
#define APP_AUDIO_CLIP_MAX_OPUS_BYTES   20000U
#define APP_AUDIO_SESSION_FLAG          0x8000U

/* ----- Image transfer over FLRC ------------------------------------------ */

#define APP_IMAGE_JPEG_QUALITY          90
#define APP_IMAGE_TX_WIDTH              240U
#define APP_IMAGE_TX_HEIGHT             320U
#define APP_IMAGE_PRE_ROTATE_WIDTH      320U
#define APP_IMAGE_PRE_ROTATE_HEIGHT     240U
#define APP_IMAGE_FRAGMENT_DATA_SIZE    (APP_FLRC_MAX_PAYLOAD_BYTES - 16U)
#define APP_IMAGE_TX_INTER_PACKET_US    0U
#define APP_IMAGE_RX_TIMEOUT_MS         3000U
#define APP_IMAGE_RX_PROGRESS_INTERVAL_MS 100U
#define APP_IMAGE_NACK_MAX_RETRIES      400U
#define APP_IMAGE_NACK_MAX_INDICES      120U
#define APP_IMAGE_MAX_JPEG_SIZE         (400U * 1024U)
#define APP_IMAGE_EOT_RETRY_COUNT       10U
#define APP_IMAGE_EOT_RETRY_INTERVAL_MS 30U
// Retry ImageStart every 50 ms for up to 30 seconds.
// This interval reduces collisions with the 30 ms ImageCmd retry cycle.
#define APP_IMAGE_START_RETRY_COUNT     600U
#define APP_IMAGE_START_RETRY_INTERVAL_MS 50U
// Retry ImageCmd until ImageCmdAck or ImageStart arrives.
// Use 30 ms normally and 1 second when a LoRa wakeup is required.
#define APP_IMAGE_REQ_RETRY_INTERVAL_MS    30U
#define APP_IMAGE_REQ_RETRY_INTERVAL_LP_MS 1000U
// Fill each low-power request round for the node's 8-second wake window.
// Start new wakeup rounds until the node replies or the request is canceled.
#define APP_IMAGE_REQ_ROUND_MS          8000U
// Low power: the node's FLRC RX wake window. Refreshed on every received packet;
// after this long with zero RX activity the node returns to CAD sleep. Also the
// node TX no-interaction abort budget (no ACK/NACK for this long -> give up).
#define APP_LP_WAKE_WINDOW_MS           8000U
#define APP_IMAGE_TASK_STACK_BYTES      16384U
#define APP_IMAGE_TASK_PRIORITY         3
#define APP_IMAGE_TASK_CORE             1
#define APP_IMAGE_RX_TASK_STACK_BYTES   16384U
#define APP_IMAGE_RX_TASK_PRIORITY      3
#define APP_IMAGE_RX_TASK_CORE          1

/* ----- Camera node LCD (set 0 to skip LCD release/reinit for faster capture) */
#define APP_CAMERA_NODE_LCD_ENABLE      0

/* ----- Camera/LCD bring-up (legacy) -------------------------------------- */

#define APP_CAMERA_LCD_BRINGUP          0

#define APP_CAMERA_UART_ENABLE          0

#define APP_CAMERA_ONLY_BRINGUP         0

#define APP_CON6_FORCE_CAMERA           0

#define APP_CAMERA_UART_BAUD            2000000

/* SP0A39 DVP mode with 24 MHz input clock. */
#define APP_CAMERA_COLOR_ENABLE         1
#define APP_SP0A39_MCLK_HZ              12000000U
#define APP_SP0A39_I2C_ADDR             0x21U

/* LCD_CAM DVP input polarity. Toggle these when the sensor clocks/syncs are
 * visible but DMA captures only blank data. */
#define APP_CAMERA_DVP_VSYNC_INVERT     1
#define APP_CAMERA_DVP_HSYNC_INVERT     1
/* LCD_CAM DVP PCLK sampling edge. 0 = rising, 1 = falling. */
#define APP_CAMERA_DVP_PCLK_INVERT      0

/* VGA 640x480 from DVP. */
#define APP_CAMERA_SENSOR_WIDTH         640U
#define APP_CAMERA_SENSOR_HEIGHT        480U
#define APP_CAMERA_FRAME_BYTES          (APP_CAMERA_SENSOR_WIDTH * APP_CAMERA_SENSOR_HEIGHT * (APP_CAMERA_COLOR_ENABLE ? 2U : 1U))

/* DMA buffer configuration for DVP capture. */
#define APP_CAMERA_DVP_DMA_WINDOW_BYTES (16U * 1024U)
#define APP_CAMERA_DVP_DMA_BUFFER_COUNT 4U

#define APP_CAMERA_UART_CHUNK_BYTES     2048U
/* esp_video SPI preprocessing runs synchronously in the capture task. Keep this
 * one-shot bring-up task at idle priority so task WDT IDLE1 can still run. */
#define APP_CAMERA_TASK_PRIORITY        0
#define APP_CAMERA_TASK_STACK_BYTES     12288U
#define APP_CAMERA_TASK_CORE            1

/* ----- ST7789V3 LCD --------------------------------------------------------- */

#define APP_LCD_H_RES                   240U
#define APP_LCD_V_RES                   320U
#define APP_LCD_X_GAP                   0
#define APP_LCD_Y_GAP                   0
#define APP_LCD_SPI_PCLK_HZ             (10U * 1000U * 1000U)
#define APP_LCD_SPI_QUEUE_DEPTH         10U
#define APP_LCD_TEST_PATTERN_ROWS       20U
#define APP_LCD_LVGL_BUFFER_ROWS        40U
#define APP_LCD_LVGL_TICK_MS            2U
#define APP_LCD_LVGL_TASK_DELAY_MS      10U
#define APP_LCD_LVGL_TASK_STACK_BYTES   8192U
#define APP_LCD_LVGL_TASK_PRIORITY      2
#define APP_LCD_LVGL_TASK_CORE          1
#define APP_LCD_PHOTO_PREVIEW_H         180U
