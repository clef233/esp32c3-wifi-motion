idf_component_register(SRCS "wifi_web.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_event esp_netif esp_http_server freertos log)
