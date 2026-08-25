#include "captive_portal.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "captive";

#define DNS_PORT        53
#define DNS_MAX_LEN     512
#define DNS_TASK_STACK  3072
#define DNS_TASK_PRIO   3

static TaskHandle_t s_dns_task = NULL;
static volatile bool s_dns_run = false;
static bool s_http_hooked = false;

/* Minimal DNS header, packed to match the wire layout. */
typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qd_count;
    uint16_t an_count;
    uint16_t ns_count;
    uint16_t ar_count;
} dns_header_t;

/* Resolve the current SoftAP IPv4 address (defaults to 192.168.4.1). */
static uint32_t get_ap_ip(void)
{
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ip = {0};
    if (ap && esp_netif_get_ip_info(ap, &ip) == ESP_OK && ip.ip.addr) {
        return ip.ip.addr;
    }
    return inet_addr("192.168.4.1");          /* network byte order */
}

/* Build a DNS response that maps each valid A query to the SoftAP address. */
static int build_dns_reply(const uint8_t *req, int req_len, uint8_t *out, int out_max)
{
    if (req_len < (int)sizeof(dns_header_t) + 5) return 0;

    const dns_header_t *qh = (const dns_header_t *)req;
    /* Only answer standard queries (QR=0, opcode=0) with at least one question. */
    if ((ntohs(qh->flags) & 0x8000) != 0) return 0;
    if (ntohs(qh->qd_count) == 0) return 0;

    /* Walk the QNAME to find where the question section ends. */
    int pos = sizeof(dns_header_t);
    while (pos < req_len && req[pos] != 0) {
        pos += req[pos] + 1;                 /* skip a label */
        if (pos >= req_len) return 0;
    }
    pos += 1;                                /* skip the root label (0x00) */
    pos += 4;                                /* skip QTYPE + QCLASS */
    if (pos > req_len) return 0;

    int q_section = pos - sizeof(dns_header_t);
    int reply_len = sizeof(dns_header_t) + q_section + 16;
    if (reply_len > out_max) return 0;

    memcpy(out, req, pos);                    /* header + original question */

    dns_header_t *rh = (dns_header_t *)out;
    rh->flags    = htons(0x8180);             /* response, recursion available */
    rh->an_count = htons(1);
    rh->ns_count = 0;
    rh->ar_count = 0;

    /* Answer: name pointer -> 0xC00C, type A, class IN, TTL, RDLENGTH=4, IP. */
    uint8_t *a = out + pos;
    a[0] = 0xC0; a[1] = 0x0C;
    a[2] = 0x00; a[3] = 0x01;                 /* type A */
    a[4] = 0x00; a[5] = 0x01;                 /* class IN */
    a[6] = 0x00; a[7] = 0x00; a[8] = 0x00; a[9] = 0x3C; /* TTL 60s */
    a[10] = 0x00; a[11] = 0x04;               /* RDLENGTH 4 */
    uint32_t ip = get_ap_ip();                /* network byte order already */
    memcpy(a + 12, &ip, 4);

    return reply_len;
}

static void dns_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket failed: errno %d", errno);
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(DNS_PORT);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed: errno %d", errno);
        close(sock);
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    /* Block on recv but wake periodically so stop() can end the task. */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "DNS hijack listening on port %d", DNS_PORT);

    uint8_t req[DNS_MAX_LEN];
    uint8_t rep[DNS_MAX_LEN];
    while (s_dns_run) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, req, sizeof(req), 0,
                         (struct sockaddr *)&from, &from_len);
        if (n <= 0) continue;                 /* timeout or error: re-check flag */

        int r = build_dns_reply(req, n, rep, sizeof(rep));
        if (r > 0) {
            sendto(sock, rep, r, 0, (struct sockaddr *)&from, from_len);
        }
    }

    close(sock);
    ESP_LOGI(TAG, "DNS hijack stopped");
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

/* Redirect unknown HTTP paths to the gallery for captive-portal discovery. */
static esp_err_t http_redirect_404(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/gallery");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Redirect known OS connectivity probes to the gallery.
 * Explicit handlers prevent repeated probe and disconnect cycles. */
static esp_err_t probe_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/gallery");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* Connectivity-check URLs used by common client operating systems. */
static const char *const s_probe_urls[] = {
    "/connecttest.txt",             /* Windows 10/11 NCSI */
    "/redirect",                    /* Windows sign-in redirect */
    "/ncsi.txt",                    /* Windows NCSI (legacy) */
    "/generate_204",                /* Android */
    "/gen_204",                     /* Android (older) */
    "/hotspot-detect.html",         /* iOS / macOS */
    "/library/test/success.html",   /* iOS / macOS (captive agent) */
    "/success.txt",                 /* Firefox / NetworkManager */
};

esp_err_t captive_portal_start(httpd_handle_t httpd)
{
    /* Hook the HTTP 404 -> gallery redirect and the explicit probe URLs. */
    if (httpd && !s_http_hooked) {
        esp_err_t e = httpd_register_err_handler(httpd, HTTPD_404_NOT_FOUND,
                                                 http_redirect_404);
        if (e == ESP_OK) {
            s_http_hooked = true;
        } else {
            ESP_LOGW(TAG, "register 404 handler failed: %s", esp_err_to_name(e));
        }

        /* Explicit handlers for OS connectivity probes so Windows reliably
         * detects the captive portal instead of looping on "no internet". */
        for (size_t i = 0; i < sizeof(s_probe_urls) / sizeof(s_probe_urls[0]); i++) {
            httpd_uri_t probe = {
                .uri     = s_probe_urls[i],
                .method  = HTTP_GET,
                .handler = probe_redirect_handler,
            };
            esp_err_t pe = httpd_register_uri_handler(httpd, &probe);
            if (pe != ESP_OK) {
                ESP_LOGW(TAG, "register probe %s failed: %s",
                         s_probe_urls[i], esp_err_to_name(pe));
            }
        }
    }

    /* Start the DNS hijack task once. */
    if (!s_dns_task) {
        s_dns_run = true;
        if (xTaskCreate(dns_task, "dns_hijack", DNS_TASK_STACK, NULL,
                        DNS_TASK_PRIO, &s_dns_task) != pdPASS) {
            s_dns_run = false;
            ESP_LOGE(TAG, "DNS task create failed");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "captive portal active");
    return ESP_OK;
}

void captive_portal_stop(void)
{
    s_dns_run = false;   /* dns_task notices within its 1s recv timeout */
}

