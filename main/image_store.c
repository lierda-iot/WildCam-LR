#include "image_store.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "opus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "img_store";

typedef struct {
    image_meta_t meta;
    uint8_t     *jpeg;
    uint8_t     *opus;
    uint8_t     *pcm;
} image_slot_t;

static image_slot_t s_slots[IMAGE_STORE_MAX_SLOTS];
static int s_head = 0;
static int s_count = 0;
static size_t s_total_bytes = 0;
static volatile bool s_abort = false;
static bool s_sntp_started = false;

static SemaphoreHandle_t s_decode_sem = NULL;
static TaskHandle_t s_decode_task = NULL;

/* Epoch below this is treated as "clock not set yet" (boot-relative). */
#define TIME_VALID_THRESHOLD 1700000000UL

/* Convert boot-relative capture times after the system clock becomes valid.
 * The epoch-to-uptime offset maps stored uptime values to wall-clock time. */
static void adjust_stored_timestamps(uint32_t real_epoch)
{
    uint32_t uptime_now = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (real_epoch < uptime_now) return; /* sanity: real time must exceed uptime */
    uint32_t offset = real_epoch - uptime_now;

    int fixed = 0;
    for (int i = 0; i < IMAGE_STORE_MAX_SLOTS; i++) {
        image_slot_t *s = &s_slots[i];
        if (s->meta.valid && s->meta.timestamp < TIME_VALID_THRESHOLD) {
            s->meta.timestamp += offset;
            fixed++;
        }
    }
    if (fixed > 0) {
        ESP_LOGI(TAG, "adjusted %d photo timestamp(s) to real time (offset=%lu)",
                 fixed, (unsigned long)offset);
    }
}

/* ---- SNTP ---- */

static void sntp_sync_cb(struct timeval *tv)
{
    struct tm t;
    localtime_r(&tv->tv_sec, &t);
    ESP_LOGI(TAG, "SNTP synced: %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    adjust_stored_timestamps((uint32_t)tv->tv_sec);
}

void image_store_start_sntp(void)
{
    if (s_sntp_started) return;
    setenv("TZ", "CST-8", 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_sync_cb);
    esp_sntp_init();
    s_sntp_started = true;
    ESP_LOGI(TAG, "SNTP started");
}

/* ---- Time sync (browser → device) ---- */

/* Set timezone only. Time itself is not persisted across reboots: the clock
 * starts at 0 (1970) on boot and photos store boot-relative timestamps until a
 * real time source (browser sync or SNTP) lets us back-calculate them. */
void image_store_restore_time(void)
{
    setenv("TZ", "CST-8", 1);
    tzset();
}

static esp_err_t handler_synctime(httpd_req_t *req)
{
    char buf[32] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    uint32_t epoch = (uint32_t)strtoul(buf, NULL, 10);
    if (epoch < TIME_VALID_THRESHOLD) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid timestamp");
        return ESP_FAIL;
    }

    /* Back-calculate boot-relative photo timestamps before moving the clock. */
    adjust_stored_timestamps(epoch);

    struct timeval tv = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);

    struct tm t;
    time_t now = (time_t)epoch;
    localtime_r(&now, &t);
    ESP_LOGI(TAG, "time synced from browser: %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

/* ---- Background PCM decode ---- */

static void decode_slot_pcm(image_slot_t *slot)
{
    if (!slot->opus || slot->meta.opus_len == 0) return;
    if (slot->pcm) return;

    const uint8_t *opus_data = slot->opus;
    size_t opus_len = slot->meta.opus_len;

    size_t pos = 0, num_frames = 0;
    while (pos < opus_len) {
        uint8_t flen = opus_data[pos++];
        if (flen == 0 || pos + flen > opus_len) break;
        pos += flen;
        num_frames++;
    }
    if (num_frames == 0) return;

    uint32_t pcm_samples = num_frames * APP_AUDIO_FRAME_SAMPLES;
    size_t pcm_bytes = pcm_samples * 2;

    uint8_t *pcm_buf = (uint8_t *)heap_caps_malloc(pcm_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm_buf) {
        ESP_LOGW(TAG, "PCM alloc failed: %u bytes", (unsigned)pcm_bytes);
        return;
    }

    int err_code = 0;
    OpusDecoder *dec = opus_decoder_create(APP_AUDIO_SAMPLE_RATE_HZ, 1, &err_code);
    if (!dec) {
        heap_caps_free(pcm_buf);
        return;
    }

    int16_t *samples = (int16_t *)pcm_buf;
    pos = 0;
    size_t soff = 0;
    while (pos < opus_len && soff < pcm_samples) {
        uint8_t flen = opus_data[pos++];
        if (flen == 0 || pos + flen > opus_len) break;
        int got = opus_decode(dec, &opus_data[pos], flen, &samples[soff], APP_AUDIO_FRAME_SAMPLES, 0);
        pos += flen;
        if (got > 0) soff += got;
    }
    opus_decoder_destroy(dec);

    /* Normalize to 90% full scale */
    int16_t peak = 0;
    for (size_t i = 0; i < soff; i++) {
        int16_t v = samples[i] < 0 ? -samples[i] : samples[i];
        if (v > peak) peak = v;
    }
    if (peak > 0 && peak < 29490) {
        int32_t gain = 29490 * 256 / peak;
        for (size_t i = 0; i < soff; i++) {
            int32_t s = ((int32_t)samples[i] * gain) >> 8;
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            samples[i] = (int16_t)s;
        }
    }

    size_t actual_pcm_bytes = soff * 2;
    slot->pcm = pcm_buf;
    slot->meta.pcm_len = actual_pcm_bytes;
    s_total_bytes += actual_pcm_bytes;

    /* Free opus data - no longer needed */
    s_total_bytes -= slot->meta.opus_len;
    heap_caps_free(slot->opus);
    slot->opus = NULL;
    slot->meta.opus_len = 0;

    ESP_LOGI(TAG, "PCM decoded: %u samples, peak=%d, %uKB", (unsigned)soff, peak, (unsigned)(actual_pcm_bytes / 1024));
}

static void decode_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_decode_sem, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        for (int i = 0; i < IMAGE_STORE_MAX_SLOTS; i++) {
            image_slot_t *slot = &s_slots[i];
            if (slot->meta.valid && slot->opus && !slot->pcm) {
                int64_t t0 = esp_timer_get_time();
                decode_slot_pcm(slot);
                uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
                ESP_LOGI(TAG, "bg decode slot %d: %lu ms", i, (unsigned long)ms);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
}

/* ---- Storage ---- */

esp_err_t image_store_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));
    s_head = 0;
    s_count = 0;
    s_total_bytes = 0;
    s_abort = false;

    if (!s_decode_sem) {
        s_decode_sem = xSemaphoreCreateBinary();
    }
    if (!s_decode_task) {
        xTaskCreatePinnedToCore(decode_task, "pcm_dec", 8192, NULL, 1, &s_decode_task, 1);
    }

    ESP_LOGI(TAG, "initialized (%d slots, %uKB max)",
             IMAGE_STORE_MAX_SLOTS, IMAGE_STORE_MAX_BYTES / 1024);
    return ESP_OK;
}

static void free_slot(int idx)
{
    image_slot_t *s = &s_slots[idx];
    if (s->jpeg) {
        s_total_bytes -= s->meta.jpeg_len;
        heap_caps_free(s->jpeg);
        s->jpeg = NULL;
    }
    if (s->opus) {
        s_total_bytes -= s->meta.opus_len;
        heap_caps_free(s->opus);
        s->opus = NULL;
    }
    if (s->pcm) {
        s_total_bytes -= s->meta.pcm_len;
        heap_caps_free(s->pcm);
        s->pcm = NULL;
    }
    s->meta.valid = false;
    s->meta.jpeg_len = 0;
    s->meta.opus_len = 0;
    s->meta.pcm_len = 0;
}

esp_err_t image_store_save(const uint8_t *jpeg, size_t jpeg_len,
                           const uint8_t *opus, size_t opus_len,
                           uint16_t session_id)
{
    if (!jpeg || jpeg_len == 0) return ESP_ERR_INVALID_ARG;

    size_t needed = jpeg_len + opus_len;
    if (needed > IMAGE_STORE_MAX_BYTES) {
        ESP_LOGW(TAG, "image too large: %u bytes", (unsigned)needed);
        return ESP_ERR_NO_MEM;
    }

    while (s_total_bytes + needed > IMAGE_STORE_MAX_BYTES && s_count > 0) {
        int oldest = (s_head - s_count + IMAGE_STORE_MAX_SLOTS) % IMAGE_STORE_MAX_SLOTS;
        free_slot(oldest);
        s_count--;
    }

    free_slot(s_head);

    uint8_t *jbuf = (uint8_t *)heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jbuf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for jpeg (%u)", (unsigned)jpeg_len);
        return ESP_ERR_NO_MEM;
    }
    memcpy(jbuf, jpeg, jpeg_len);

    uint8_t *obuf = NULL;
    if (opus && opus_len > 0) {
        obuf = (uint8_t *)heap_caps_malloc(opus_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!obuf) {
            heap_caps_free(jbuf);
            ESP_LOGE(TAG, "PSRAM alloc failed for opus (%u)", (unsigned)opus_len);
            return ESP_ERR_NO_MEM;
        }
        memcpy(obuf, opus, opus_len);
    }

    image_slot_t *slot = &s_slots[s_head];
    slot->jpeg = jbuf;
    slot->opus = obuf;
    slot->pcm = NULL;
    slot->meta.jpeg_len = jpeg_len;
    slot->meta.opus_len = opus_len;
    slot->meta.pcm_len = 0;
    slot->meta.session_id = session_id;
    slot->meta.timestamp = (uint32_t)time(NULL);
    slot->meta.valid = true;
    image_store_start_sntp();

    s_total_bytes += jpeg_len + opus_len;
    s_head = (s_head + 1) % IMAGE_STORE_MAX_SLOTS;
    if (s_count < IMAGE_STORE_MAX_SLOTS) s_count++;

    ESP_LOGI(TAG, "saved image #%d: jpeg=%u opus=%u total_used=%uKB",
             s_count, (unsigned)jpeg_len, (unsigned)opus_len,
             (unsigned)(s_total_bytes / 1024));

    if (obuf && s_decode_sem) {
        xSemaphoreGive(s_decode_sem);
    }

    return ESP_OK;
}

int image_store_count(void)
{
    return s_count;
}

static int logical_to_physical(int index)
{
    if (index < 0 || index >= s_count) return -1;
    return (s_head - s_count + index + IMAGE_STORE_MAX_SLOTS) % IMAGE_STORE_MAX_SLOTS;
}

const image_meta_t *image_store_get_meta(int index)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    return &s_slots[phys].meta;
}

const uint8_t *image_store_get_jpeg(int index, size_t *out_len)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    if (out_len) *out_len = s_slots[phys].meta.jpeg_len;
    return s_slots[phys].jpeg;
}

const uint8_t *image_store_get_opus(int index, size_t *out_len)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    if (!s_slots[phys].opus) return NULL;
    if (out_len) *out_len = s_slots[phys].meta.opus_len;
    return s_slots[phys].opus;
}

void image_store_abort_transfer(void)
{
    s_abort = true;
}

/* ---- HTTP Handlers ---- */

#define CHUNK_SIZE  8192

static esp_err_t send_jpeg_chunked(httpd_req_t *req, const uint8_t *data, size_t len)
{
    s_abort = false;
    httpd_resp_set_type(req, "image/jpeg");

    char cl[16];
    snprintf(cl, sizeof(cl), "%u", (unsigned)len);
    httpd_resp_set_hdr(req, "Content-Length", cl);

    size_t sent = 0;
    while (sent < len) {
        if (s_abort) {
            ESP_LOGW(TAG, "HTTP transfer aborted by capture");
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_OK;
        }
        size_t chunk = (len - sent > CHUNK_SIZE) ? CHUNK_SIZE : (len - sent);
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)&data[sent], chunk);
        if (err != ESP_OK) return err;
        sent += chunk;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_api_images(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    char buf[64];
    httpd_resp_sendstr_chunk(req, "[");

    int count = image_store_count();
    for (int i = 0; i < count; i++) {
        const image_meta_t *m = image_store_get_meta(i);
        if (!m || !m->valid) continue;
        int has_audio = (m->pcm_len > 0 || m->opus_len > 0) ? 1 : 0;
        int n = snprintf(buf, sizeof(buf),
            "%s{\"id\":%d,\"jpeg\":%lu,\"audio\":%d,\"ts\":%lu}",
            (i > 0) ? "," : "",
            i,
            (unsigned long)m->jpeg_len,
            has_audio,
            (unsigned long)m->timestamp);
        httpd_resp_send_chunk(req, buf, n);
    }

    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_img(httpd_req_t *req)
{
    int idx = -1;
    const char *uri = req->uri;
    const char *num = uri + 5;  /* skip "/img/" */
    idx = atoi(num);

    size_t len = 0;
    const uint8_t *data = image_store_get_jpeg(idx, &len);
    if (!data) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Image not found");
        return ESP_FAIL;
    }
    return send_jpeg_chunked(req, data, len);
}

static void write_wav_header(uint8_t *h, uint32_t pcm_bytes)
{
    uint32_t file_size = 36 + pcm_bytes;
    uint32_t sr = APP_AUDIO_SAMPLE_RATE_HZ;
    uint32_t brate = sr * 2;
    memcpy(h, "RIFF", 4);
    h[4]=file_size; h[5]=file_size>>8; h[6]=file_size>>16; h[7]=file_size>>24;
    memcpy(h+8, "WAVEfmt ", 8);
    h[16]=16; h[17]=0; h[18]=0; h[19]=0;
    h[20]=1; h[21]=0;
    h[22]=1; h[23]=0;
    h[24]=sr; h[25]=sr>>8; h[26]=sr>>16; h[27]=sr>>24;
    h[28]=brate; h[29]=brate>>8; h[30]=brate>>16; h[31]=brate>>24;
    h[32]=2; h[33]=0;
    h[34]=16; h[35]=0;
    memcpy(h+36, "data", 4);
    h[40]=pcm_bytes; h[41]=pcm_bytes>>8; h[42]=pcm_bytes>>16; h[43]=pcm_bytes>>24;
}

static esp_err_t handler_audio(httpd_req_t *req)
{
    const char *num = req->uri + 7;  /* skip "/audio/" */
    int idx = atoi(num);
    int phys = logical_to_physical(idx);
    if (phys < 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    }

    image_slot_t *slot = &s_slots[phys];

    /* Prepare PCM: use pre-decoded if available, else decode on the fly */
    const uint8_t *pcm_data = slot->pcm;
    size_t pcm_bytes = slot->meta.pcm_len;
    uint8_t *temp_buf = NULL;

    if (!pcm_data || pcm_bytes == 0) {
        /* Fallback: decode Opus to temporary buffer */
        const uint8_t *opus_data = slot->opus;
        size_t opus_len = slot->meta.opus_len;
        if (!opus_data || opus_len == 0) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Audio not found");
            return ESP_FAIL;
        }

        size_t pos = 0, num_frames = 0;
        while (pos < opus_len) {
            uint8_t flen = opus_data[pos++];
            if (flen == 0 || pos + flen > opus_len) break;
            pos += flen;
            num_frames++;
        }
        if (num_frames == 0) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No frames");
            return ESP_FAIL;
        }

        uint32_t pcm_samples = num_frames * APP_AUDIO_FRAME_SAMPLES;
        pcm_bytes = pcm_samples * 2;

        temp_buf = (uint8_t *)heap_caps_malloc(pcm_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!temp_buf) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
            return ESP_FAIL;
        }

        int err_code = 0;
        OpusDecoder *dec = opus_decoder_create(APP_AUDIO_SAMPLE_RATE_HZ, 1, &err_code);
        if (!dec) {
            heap_caps_free(temp_buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Decoder err");
            return ESP_FAIL;
        }

        int16_t *pcm = (int16_t *)temp_buf;
        pos = 0;
        size_t soff = 0;
        while (pos < opus_len && soff < pcm_samples) {
            uint8_t flen = opus_data[pos++];
            if (flen == 0 || pos + flen > opus_len) break;
            int got = opus_decode(dec, &opus_data[pos], flen, &pcm[soff], APP_AUDIO_FRAME_SAMPLES, 0);
            pos += flen;
            if (got > 0) soff += got;
        }
        opus_decoder_destroy(dec);

        /* Normalize */
        int16_t peak = 0;
        for (size_t i = 0; i < soff; i++) {
            int16_t v = pcm[i] < 0 ? -pcm[i] : pcm[i];
            if (v > peak) peak = v;
        }
        if (peak > 0 && peak < 29490) {
            int32_t gain = 29490 * 256 / peak;
            for (size_t i = 0; i < soff; i++) {
                int32_t s = ((int32_t)pcm[i] * gain) >> 8;
                if (s > 32767) s = 32767;
                if (s < -32768) s = -32768;
                pcm[i] = (int16_t)s;
            }
        }

        pcm_data = temp_buf;
        pcm_bytes = soff * 2;
    }

    /* Assemble complete WAV file in contiguous buffer (header + PCM).
     * This lets httpd_resp_send handle Content-Length and Range requests
     * automatically, so audio progress bar works correctly. */
    size_t wav_size = 44 + pcm_bytes;
    uint8_t *wav_buf = (uint8_t *)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) {
        heap_caps_free(temp_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    write_wav_header(wav_buf, pcm_bytes);
    memcpy(wav_buf + 44, pcm_data, pcm_bytes);
    heap_caps_free(temp_buf);

    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    /* Send the complete buffered WAV with Content-Length so browser duration and seeking work. */
    esp_err_t err = httpd_resp_send(req, (const char *)wav_buf, wav_size);
    heap_caps_free(wav_buf);
    return err;
}

static const char GALLERY_HTML[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>AM36 Gallery</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:system-ui,sans-serif;background:#f4f6f8;color:#333;padding:16px 16px 32px}"
    "header{text-align:center;margin-bottom:20px}"
    "header h1{font-size:1.3em;color:#1a73e8;font-weight:600}"
    "header .sub{font-size:.8em;color:#888;margin-top:4px}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:16px;max-width:1100px;margin:0 auto}"
    ".card{background:#fff;border-radius:12px;overflow:hidden;cursor:pointer;box-shadow:0 2px 8px rgba(0,0,0,.08);transition:transform .15s,box-shadow .15s}"
    ".card:hover{transform:translateY(-2px);box-shadow:0 4px 16px rgba(0,0,0,.12)}"
    ".card img{width:100%;aspect-ratio:3/4;object-fit:contain;display:block;pointer-events:none}"
    ".card .info{padding:10px 12px;font-size:.8em;color:#555;display:flex;justify-content:space-between;align-items:center}"
    ".card .badge{background:#1a73e8;color:#fff;padding:2px 7px;border-radius:10px;font-size:.7em;font-weight:600}"
    ".ov{display:none;position:fixed;inset:0;background:rgba(0,0,0,.92);z-index:100;align-items:center;justify-content:center;flex-direction:column}"
    ".ov.show{display:flex}"
    ".ov img{max-width:90%;max-height:65vh;object-fit:contain;border-radius:6px}"
    ".ov .close{position:absolute;top:14px;right:18px;font-size:2em;color:#fff;cursor:pointer;line-height:1;z-index:2}"
    ".ov .nav{position:absolute;top:50%;transform:translateY(-50%);font-size:2.5em;color:#fff;cursor:pointer;user-select:none;padding:10px;opacity:.7}"
    ".ov .nav:hover{opacity:1}"
    ".ov .prev{left:12px}"
    ".ov .next{right:12px}"
    ".ov .bottom{margin-top:14px;display:flex;flex-direction:column;align-items:center;gap:8px}"
    ".ov .bottom a{color:#8ab4f8;text-decoration:none;font-size:.85em}"
    "audio{width:260px}"
    ".empty{text-align:center;margin-top:60px;color:#999;font-size:.95em}"
    "</style></head><body>"
    "<header><h1>AM36 Gallery</h1><div class='sub' id='info'></div></header>"
    "<div class='grid' id='grid'></div>"
    "<div class='ov' id='ov'>"
    "<span class='close' onclick='closeOv()'>&times;</span>"
    "<span class='nav prev' onclick='navImg(-1)'>&#10094;</span>"
    "<span class='nav next' onclick='navImg(1)'>&#10095;</span>"
    "<img id='ovimg'>"
    "<div class='bottom'>"
    "<audio id='ovaudio' controls style='display:none'></audio>"
    "<a id='ovdl' download>Download</a>"
    "</div></div>"
    "<div class='empty' id='empty'>No images yet</div>"
    "<script>"
    "var grid=document.getElementById('grid');"
    "var ov=document.getElementById('ov');"
    "var ovimg=document.getElementById('ovimg');"
    "var ovaudio=document.getElementById('ovaudio');"
    "var ovdl=document.getElementById('ovdl');"
    "var info=document.getElementById('info');"
    "var empty=document.getElementById('empty');"
    "var imgList=[],curIdx=0;"
    "function fmtT(ts){if(ts<1700000000){var m=Math.floor(ts/60);return'Boot +'+m+'min'}"
    "var d=new Date(ts*1000);function p(n){return(n<10?'0':'')+n}"
    "return d.getFullYear()+'.'+(d.getMonth()+1)+'.'+d.getDate()+' '+p(d.getHours())+':'+p(d.getMinutes())}"
    "function showImg(idx){"
    "if(idx<0)idx=imgList.length-1;if(idx>=imgList.length)idx=0;"
    "curIdx=idx;var m=imgList[idx];"
    "ovimg.src='/img/'+m.id;ovdl.href='/img/'+m.id;"
    "ovaudio.pause();ovaudio.style.display='none';ovaudio.src='';"
    "if(m.audio){ovaudio.src='/audio/'+m.id;ovaudio.style.display='block'}}"
    "function openOv(idx){showImg(idx);ov.classList.add('show')}"
    "function closeOv(){ov.classList.remove('show');ovimg.src='';ovaudio.pause();ovaudio.src=''}"
    "function navImg(dir){showImg(curIdx+dir)}"
    "ov.addEventListener('click',function(e){if(e.target===ov)closeOv()});"
    "document.addEventListener('keydown',function(e){"
    "if(!ov.classList.contains('show'))return;"
    "if(e.key==='Escape')closeOv();"
    "if(e.key==='ArrowLeft')navImg(-1);"
    "if(e.key==='ArrowRight')navImg(1)});"
    "function load(){"
    "fetch('/api/images').then(function(r){return r.json()}).then(function(imgs){"
    "grid.innerHTML='';"
    "if(!imgs.length){empty.style.display='block';info.textContent='';return}"
    "empty.style.display='none';"
    "imgList=imgs.slice().reverse();"
    "info.textContent=imgs.length+' photo'+(imgs.length>1?'s':'');"
    "for(var i=0;i<imgList.length;i++){(function(idx){var m=imgList[idx];"
    "var d=document.createElement('div');d.className='card';"
    "d.onclick=function(){openOv(idx)};"
    "var sz=m.jpeg>1024?(m.jpeg/1024).toFixed(1)+'KB':m.jpeg+'B';"
    "var tm=fmtT(m.ts);"
    "var badge=m.audio?'<span class=\"badge\">AUDIO</span>':'';"
    "d.innerHTML='<img src=\"/img/'+m.id+'\" loading=\"lazy\"><div class=\"info\"><span>'+tm+' &middot; '+sz+'</span>'+badge+'</div>';"
    "grid.appendChild(d)})(i)}})}"
    "fetch('/api/synctime',{method:'POST',body:Math.floor(Date.now()/1000).toString()})"
    ".catch(function(){}).finally(function(){load();setInterval(load,8000)});"
    "</script></body></html>";

static esp_err_t handler_gallery(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, GALLERY_HTML, sizeof(GALLERY_HTML) - 1);
    return ESP_OK;
}

esp_err_t image_store_register_httpd(httpd_handle_t httpd)
{
    if (!httpd) return ESP_ERR_INVALID_ARG;

    const httpd_uri_t uri_gallery = {
        .uri = "/gallery",
        .method = HTTP_GET,
        .handler = handler_gallery,
    };
    const httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handler_gallery,
    };
    const httpd_uri_t uri_api = {
        .uri = "/api/images",
        .method = HTTP_GET,
        .handler = handler_api_images,
    };
    const httpd_uri_t uri_img = {
        .uri = "/img/*",
        .method = HTTP_GET,
        .handler = handler_img,
    };
    const httpd_uri_t uri_audio = {
        .uri = "/audio/*",
        .method = HTTP_GET,
        .handler = handler_audio,
    };
    const httpd_uri_t uri_synctime = {
        .uri = "/api/synctime",
        .method = HTTP_POST,
        .handler = handler_synctime,
    };

    const httpd_uri_t *all[] = {
        &uri_gallery, &uri_api, &uri_img, &uri_audio, &uri_synctime, &uri_root,
    };
    esp_err_t first_err = ESP_OK;
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        esp_err_t e = httpd_register_uri_handler(httpd, all[i]);
        /* Already-registered is fine: this may be called again on reconnect. */
        if (e != ESP_OK && e != ESP_ERR_HTTPD_HANDLER_EXISTS) {
            ESP_LOGE(TAG, "register '%s' failed: %s", all[i]->uri, esp_err_to_name(e));
            if (first_err == ESP_OK) first_err = e;
        }
    }
    if (first_err != ESP_OK) return first_err;

    ESP_LOGI(TAG, "HTTP handlers registered: / /gallery /api/images /img/* /audio/* /api/synctime");
    return ESP_OK;
}
