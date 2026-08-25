#include "wifi_manager.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "wifi_mgr";

#define NVS_NAMESPACE    "wifi_cfg"
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASS     "pass"

static wifi_mgr_state_t s_state = WIFI_MGR_DISCONNECTED;
static wifi_mgr_state_cb_t s_state_cb = NULL;
static char s_ssid[33] = {0};
static char s_service_name[16] = {0};
static char s_ap_password[16] = {0};
static int8_t s_rssi = 0;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_wifi_started = false;
static httpd_handle_t s_httpd = NULL;

static void set_state(wifi_mgr_state_t st)
{
    s_state = st;
    if (s_state_cb) s_state_cb(st);
}

static bool load_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return false;
    esp_err_t e1 = nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_len);
    esp_err_t e2 = nvs_get_str(nvs, NVS_KEY_PASS, pass, &pass_len);
    nvs_close(nvs);
    return (e1 == ESP_OK && e2 == ESP_OK && ssid[0] != '\0');
}

static void save_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASS, pass);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "credentials saved: %s", ssid);
}

static void update_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        s_rssi = ap.rssi;
    }
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_PROV_EVENT) {
        switch (id) {
        case WIFI_PROV_CRED_RECV: {
            wifi_sta_config_t *cfg = (wifi_sta_config_t *)data;
            ESP_LOGI(TAG, "prov cred received: %s", (const char *)cfg->ssid);
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "prov success");
            wifi_prov_mgr_stop_provisioning();
            break;
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGW(TAG, "prov failed, retrying...");
            wifi_prov_mgr_reset_sm_state_on_failure();
            break;
        case WIFI_PROV_END:
            wifi_prov_mgr_deinit();
            ESP_LOGI(TAG, "prov manager deinitialized");
            break;
        default:
            break;
        }
    } else if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t *)data;
            memset(s_ssid, 0, sizeof(s_ssid));
            memcpy(s_ssid, ev->ssid, ev->ssid_len);
            set_state(WIFI_MGR_CONNECTING);
            ESP_LOGI(TAG, "STA connected to %s", s_ssid);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_state == WIFI_MGR_CONNECTING || s_state == WIFI_MGR_CONNECTED) {
                ESP_LOGI(TAG, "STA disconnected, reconnecting...");
                set_state(WIFI_MGR_DISCONNECTED);
                esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_AP_STACONNECTED: {
            wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
            ESP_LOGI(TAG, "AP: station " MACSTR " join, AID=%d", MAC2STR(ev->mac), ev->aid);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            wifi_event_ap_stadisconnected_t *ev = (wifi_event_ap_stadisconnected_t *)data;
            ESP_LOGI(TAG, "AP: station " MACSTR " leave, AID=%d", MAC2STR(ev->mac), ev->aid);
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        update_rssi();

        wifi_config_t wcfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &wcfg) == ESP_OK) {
            save_credentials((const char *)wcfg.sta.ssid, (const char *)wcfg.sta.password);
            memset(s_ssid, 0, sizeof(s_ssid));
            strncpy(s_ssid, (const char *)wcfg.sta.ssid, sizeof(s_ssid) - 1);
        }
        set_state(WIFI_MGR_CONNECTED);
    }
}

static void generate_service_name(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_service_name, sizeof(s_service_name), "AM36_%01X%02X%02X",
             mac[3] & 0x0F, mac[4], mac[5]);
    snprintf(s_ap_password, sizeof(s_ap_password), "AM36%02X%02X%01X",
             mac[0], mac[1], mac[2] >> 4);
}

/* ---- Web config page ---- */

static const char CONFIG_PAGE[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>AM36 WiFi Setup</title>"
    "<style>"
    "body{font-family:sans-serif;max-width:320px;margin:40px auto;padding:0 16px}"
    "h2{text-align:center}"
    "input,button{width:100%;padding:10px;margin:6px 0;box-sizing:border-box;font-size:16px}"
    "button{background:#4CAF50;color:#fff;border:none;border-radius:4px;cursor:pointer}"
    ".msg{text-align:center;color:#333;margin-top:16px}"
    "</style></head><body>"
    "<h2>AM36 WiFi Config</h2>"
    "<form action='/connect' method='post'>"
    "<label>SSID</label><input name='ssid' required maxlength='32'>"
    "<label>Password</label><input name='pass' type='password' maxlength='64'>"
    "<button type='submit'>Connect</button>"
    "</form>"
    "<div class='msg'>Enter your home WiFi credentials</div>"
    "</body></html>";

static const char CONNECT_OK_PAGE[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Connecting...</title>"
    "<style>body{font-family:sans-serif;max-width:320px;margin:40px auto;padding:0 16px;text-align:center}</style>"
    "</head><body>"
    "<h2>Connecting...</h2>"
    "<p>Device is connecting to your WiFi network. This page will become unreachable.</p>"
    "</body></html>";

static esp_err_t config_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CONFIG_PAGE, sizeof(CONFIG_PAGE) - 1);
    return ESP_OK;
}

static int url_decode_char(const char *src)
{
    int hi = 0, lo = 0;
    if (src[0] >= '0' && src[0] <= '9') hi = src[0] - '0';
    else if (src[0] >= 'A' && src[0] <= 'F') hi = src[0] - 'A' + 10;
    else if (src[0] >= 'a' && src[0] <= 'f') hi = src[0] - 'a' + 10;
    else return -1;
    if (src[1] >= '0' && src[1] <= '9') lo = src[1] - '0';
    else if (src[1] >= 'A' && src[1] <= 'F') lo = src[1] - 'A' + 10;
    else if (src[1] >= 'a' && src[1] <= 'f') lo = src[1] - 'a' + 10;
    else return -1;
    return (hi << 4) | lo;
}

static size_t url_decode(char *dst, size_t dst_len, const char *src)
{
    size_t di = 0;
    while (*src && di < dst_len - 1) {
        if (*src == '%' && src[1] && src[2]) {
            int c = url_decode_char(src + 1);
            if (c >= 0) { dst[di++] = (char)c; src += 3; continue; }
        }
        if (*src == '+') { dst[di++] = ' '; src++; continue; }
        dst[di++] = *src++;
    }
    dst[di] = '\0';
    return di;
}

static bool parse_form_field(const char *body, const char *key, char *out, size_t out_len)
{
    size_t klen = strlen(key);
    const char *p = body;
    while ((p = strstr(p, key)) != NULL) {
        if (p == body || *(p - 1) == '&') {
            if (p[klen] == '=') {
                p += klen + 1;
                const char *end = strchr(p, '&');
                size_t vlen = end ? (size_t)(end - p) : strlen(p);
                char encoded[128];
                if (vlen >= sizeof(encoded)) vlen = sizeof(encoded) - 1;
                memcpy(encoded, p, vlen);
                encoded[vlen] = '\0';
                url_decode(out, out_len, encoded);
                return true;
            }
        }
        p += klen;
    }
    return false;
}

static esp_err_t connect_post_handler(httpd_req_t *req)
{
    char body[256] = {0};
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char ssid[33] = {0};
    char pass[65] = {0};
    if (!parse_form_field(body, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing SSID");
        return ESP_FAIL;
    }
    parse_form_field(body, "pass", pass, sizeof(pass));

    ESP_LOGI(TAG, "Web config: SSID=%s", ssid);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CONNECT_OK_PAGE, sizeof(CONNECT_OK_PAGE) - 1);

    save_credentials(ssid, pass);

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid, ssid, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, pass, sizeof(wcfg.sta.password) - 1);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    set_state(WIFI_MGR_CONNECTING);
    esp_wifi_connect();

    return ESP_OK;
}

static esp_err_t start_httpd(void)
{
    if (s_httpd) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* wifi_provisioning softap registers ~5 handlers, wifi_manager adds 2,
     * image_store adds 6, and the captive portal adds 8 OS probe URLs. Keep
     * headroom so none fail with HANDLERS_FULL. */
    cfg.max_uri_handlers = 28;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &cfg), TAG, "httpd start");

    const httpd_uri_t config_page = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
    };
    httpd_register_uri_handler(s_httpd, &config_page);

    const httpd_uri_t connect_uri = {
        .uri = "/connect",
        .method = HTTP_POST,
        .handler = connect_post_handler,
    };
    httpd_register_uri_handler(s_httpd, &connect_uri);

    ESP_LOGI(TAG, "HTTP server started on port %d", cfg.server_port);
    return ESP_OK;
}

/* ---- WiFi hardware ---- */

static esp_err_t wifi_hw_start(void)
{
    if (s_wifi_started) return ESP_OK;

#if APP_WIFI_STA_ENABLE
    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }
#endif
    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    /* SoftAP needs enough RX headroom during the association handshake, or a
     * client repeatedly fails auth/assoc and its slot lingers until timeout.
     * Bumped from 6/12 to 10/16 to fix clients that cannot associate. */
    cfg.static_rx_buf_num = 10;
    cfg.dynamic_rx_buf_num = 16;
    cfg.static_tx_buf_num = 0;
    cfg.dynamic_tx_buf_num = 12;
    cfg.tx_buf_type = 1;
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");

#if APP_WIFI_STA_ENABLE
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "set mode");
#else
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set mode");
#endif

    wifi_config_t ap_cfg = {};
    strncpy((char *)ap_cfg.ap.ssid, s_service_name, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = strlen(s_service_name);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    ap_cfg.ap.max_connection = 4;
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    s_wifi_started = true;
    ESP_LOGI(TAG, "WiFi APSTA started, AP: %s (open)", s_service_name);
    return ESP_OK;
}

esp_err_t wifi_mgr_init(void)
{
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(err));
        return err;
    }

    esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL);

    generate_service_name();

    ESP_RETURN_ON_ERROR(wifi_hw_start(), TAG, "hw start");
    ESP_RETURN_ON_ERROR(start_httpd(), TAG, "httpd");

#if APP_WIFI_STA_ENABLE
    char ssid[33] = {0};
    char pass[65] = {0};
    if (load_credentials(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "saved WiFi: %s, connecting...", ssid);

        wifi_config_t wcfg = {};
        strncpy((char *)wcfg.sta.ssid, ssid, sizeof(wcfg.sta.ssid) - 1);
        strncpy((char *)wcfg.sta.password, pass, sizeof(wcfg.sta.password) - 1);
        esp_wifi_set_config(WIFI_IF_STA, &wcfg);
        esp_wifi_connect();
        set_state(WIFI_MGR_CONNECTING);
    } else {
        ESP_LOGI(TAG, "no saved credentials, AP-only mode");
        set_state(WIFI_MGR_DISCONNECTED);
    }
#else
    ESP_LOGI(TAG, "STA disabled, AP-only mode");
    set_state(WIFI_MGR_DISCONNECTED);
#endif

    return ESP_OK;
}

esp_err_t wifi_mgr_start_provisioning(void)
{
#if !APP_WIFI_STA_ENABLE
    ESP_LOGW(TAG, "STA disabled, provisioning not available");
    return ESP_ERR_NOT_SUPPORTED;
#else
    ESP_RETURN_ON_ERROR(wifi_hw_start(), TAG, "hw start for prov");

    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    ESP_RETURN_ON_ERROR(start_httpd(), TAG, "httpd");

    wifi_prov_scheme_softap_set_httpd_handle(&s_httpd);

    wifi_prov_mgr_deinit();

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_RETURN_ON_ERROR(wifi_prov_mgr_init(config), TAG, "prov mgr init");

    wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
    const char *pop = s_ap_password;

    ESP_LOGI(TAG, "starting SoftAP prov: name=%s (open AP)", s_service_name);

    ESP_RETURN_ON_ERROR(
        wifi_prov_mgr_start_provisioning(security, pop, s_service_name, NULL),
        TAG, "start prov");

    set_state(WIFI_MGR_PROVISIONING);
    return ESP_OK;
#endif
}

void wifi_mgr_stop_provisioning(void)
{
    wifi_prov_mgr_stop_provisioning();
    wifi_prov_mgr_deinit();
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    set_state(WIFI_MGR_DISCONNECTED);
    ESP_LOGI(TAG, "provisioning stopped");
}

void wifi_mgr_disconnect(void)
{
    if (s_state == WIFI_MGR_PROVISIONING) {
        wifi_mgr_stop_provisioning();
    }
    set_state(WIFI_MGR_DISCONNECTED);
    s_ssid[0] = '\0';
    if (s_wifi_started) {
        esp_wifi_disconnect();
    }
    ESP_LOGI(TAG, "WiFi disconnected");
}

wifi_mgr_state_t wifi_mgr_get_state(void)
{
    if (s_state == WIFI_MGR_CONNECTED) update_rssi();
    return s_state;
}

const char *wifi_mgr_get_ssid(void)
{
    return s_ssid;
}

int8_t wifi_mgr_get_rssi(void)
{
    return s_rssi;
}

const char *wifi_mgr_get_service_name(void)
{
    return s_service_name;
}

const char *wifi_mgr_get_ap_password(void)
{
    return s_ap_password;
}

void wifi_mgr_set_state_cb(wifi_mgr_state_cb_t cb)
{
    s_state_cb = cb;
}

httpd_handle_t wifi_mgr_get_httpd(void)
{
    return s_httpd;
}

esp_err_t wifi_mgr_ensure_httpd(void)
{
    return start_httpd();
}
