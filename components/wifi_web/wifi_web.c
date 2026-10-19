#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "wifi_web.h"

static const char *TAG = "wifi_web";

typedef struct {
    char uri[64];
    httpd_method_t method;
    esp_err_t (*handler)(httpd_req_t *req);
} wifi_web_uri_t;

static wifi_web_config_t s_config;
static esp_netif_t *s_sta_netif;
static httpd_handle_t s_server;
static wifi_web_uri_t s_uris[WIFI_WEB_MAX_HANDLERS];
static size_t s_uri_count;
static bool s_connected;
static char s_ip_str[16];

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
        s_connected = false;
        s_ip_str[0] = '\0';
        ESP_LOGW(TAG, "WiFi disconnected, reason=%d, retrying...", event->reason);
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_ip4addr_ntoa(&event->ip_info.ip, s_ip_str, sizeof(s_ip_str));
        s_connected = true;
        ESP_LOGI(TAG, "Connected! IP address: %s", s_ip_str);
    }
}

esp_err_t wifi_web_init(const wifi_web_config_t *config)
{
    if (config == NULL || config->ssid == NULL || config->password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;
    s_connected = false;
    s_ip_str[0] = '\0';

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    if (s_config.use_static_ip) {
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(s_sta_netif));

        esp_netif_ip_info_t ip_info = {0};
        esp_netif_set_ip4_addr(&ip_info.ip, s_config.ip[0], s_config.ip[1],
                               s_config.ip[2], s_config.ip[3]);
        esp_netif_set_ip4_addr(&ip_info.gw, s_config.gateway[0], s_config.gateway[1],
                               s_config.gateway[2], s_config.gateway[3]);
        esp_netif_set_ip4_addr(&ip_info.netmask, s_config.netmask[0], s_config.netmask[1],
                               s_config.netmask[2], s_config.netmask[3]);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip_info));

        esp_netif_dns_info_t dns_info = {0};
        dns_info.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_ip4_addr(&dns_info.ip.u_addr.ip4, s_config.gateway[0],
                               s_config.gateway[1], s_config.gateway[2],
                               s_config.gateway[3]);
        ESP_ERROR_CHECK(esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
        },
    };
    strncpy((char *)wifi_config.sta.ssid, s_config.ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, s_config.password,
            sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

esp_err_t wifi_web_add_uri(const char *uri, httpd_method_t method,
                           esp_err_t (*handler)(httpd_req_t *req))
{
    if (uri == NULL || handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_server != NULL) {
        httpd_uri_t item = {
            .uri = uri,
            .method = method,
            .handler = handler,
            .user_ctx = NULL,
        };
        return httpd_register_uri_handler(s_server, &item);
    }

    if (s_uri_count >= WIFI_WEB_MAX_HANDLERS) {
        return ESP_ERR_NO_MEM;
    }

    strncpy(s_uris[s_uri_count].uri, uri, sizeof(s_uris[s_uri_count].uri) - 1);
    s_uris[s_uri_count].method = method;
    s_uris[s_uri_count].handler = handler;
    s_uri_count++;
    return ESP_OK;
}

esp_err_t wifi_web_start(void)
{
    if (s_server != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&s_server, &config));

    for (size_t i = 0; i < s_uri_count; i++) {
        httpd_uri_t item = {
            .uri = s_uris[i].uri,
            .method = s_uris[i].method,
            .handler = s_uris[i].handler,
            .user_ctx = NULL,
        };
        ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &item));
    }

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}

esp_err_t wifi_web_stop(void)
{
    if (s_server != NULL) {
        httpd_stop(s_server);
        s_server = NULL;
    }

    esp_wifi_disconnect();
    esp_wifi_stop();
    return ESP_OK;
}

esp_err_t wifi_web_get_ip_str(char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    strncpy(buf, s_ip_str, len - 1);
    buf[len - 1] = '\0';
    return ESP_OK;
}
