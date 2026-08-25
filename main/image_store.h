#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_STORE_MAX_SLOTS   20
#define IMAGE_STORE_MAX_BYTES   (6U * 1024U * 1024U)

typedef struct {
    uint32_t timestamp;
    uint32_t jpeg_len;
    uint32_t opus_len;
    uint32_t pcm_len;
    uint16_t session_id;
    bool     valid;
} image_meta_t;

esp_err_t image_store_init(void);
void      image_store_start_sntp(void);
void      image_store_restore_time(void);

esp_err_t image_store_save(const uint8_t *jpeg, size_t jpeg_len,
                           const uint8_t *opus, size_t opus_len,
                           uint16_t session_id);

int       image_store_count(void);
const image_meta_t *image_store_get_meta(int index);

const uint8_t *image_store_get_jpeg(int index, size_t *out_len);
const uint8_t *image_store_get_opus(int index, size_t *out_len);

esp_err_t image_store_register_httpd(httpd_handle_t httpd);

void      image_store_abort_transfer(void);

#ifdef __cplusplus
}
#endif
