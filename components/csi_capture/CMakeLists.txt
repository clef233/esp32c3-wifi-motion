idf_component_register(SRCS "csi_capture.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif lwip esp_timer freertos log)
