#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WIFI_MGR_DISCONNECTED = 0,
    WIFI_MGR_CONNECTING,
    WIFI_MGR_CONNECTED,
    WIFI_MGR_PROVISIONING,
} wifi_mgr_state_t;

typedef void (*wifi_mgr_state_cb_t)(wifi_mgr_state_t state);

esp_err_t wifi_mgr_init(void);
esp_err_t wifi_mgr_start_provisioning(void);
void      wifi_mgr_stop_provisioning(void);
void      wifi_mgr_disconnect(void);

wifi_mgr_state_t wifi_mgr_get_state(void);
const char *wifi_mgr_get_ssid(void);
int8_t     wifi_mgr_get_rssi(void);
const char *wifi_mgr_get_service_name(void);
const char *wifi_mgr_get_ap_password(void);

#include "esp_http_server.h"
httpd_handle_t wifi_mgr_get_httpd(void);
esp_err_t      wifi_mgr_ensure_httpd(void);

void wifi_mgr_set_state_cb(wifi_mgr_state_cb_t cb);

#ifdef __cplusplus
}
#endif
