#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Captive portal for the SoftAP gallery.
 * DNS maps all hosts to the AP, and HTTP probe paths redirect to /gallery.
 * Call captive_portal_start() after the AP and HTTP server are running. */
esp_err_t captive_portal_start(httpd_handle_t httpd);

/* Stop the DNS server and free its socket/task. */
void captive_portal_stop(void);

#ifdef __cplusplus
}
#endif
