#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

#define WIFI_WEB_MAX_HANDLERS 8

typedef struct {
    const char *ssid;
    const char *password;
    bool use_static_ip;
    uint8_t ip[4];
    uint8_t gateway[4];
    uint8_t netmask[4];
} wifi_web_config_t;

esp_err_t wifi_web_init(const wifi_web_config_t *config);
esp_err_t wifi_web_add_uri(const char *uri, httpd_method_t method,
                           esp_err_t (*handler)(httpd_req_t *req));
esp_err_t wifi_web_start(void);
esp_err_t wifi_web_stop(void);
esp_err_t wifi_web_get_ip_str(char *buf, size_t len);
