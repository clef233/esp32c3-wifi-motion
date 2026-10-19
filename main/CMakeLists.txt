idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_netif wifi_web csi_capture esp_http_server esp_driver_tsens nvs_flash app_update driver freertos log)
