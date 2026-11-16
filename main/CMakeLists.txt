idf_component_register(SRCS "main.c" "ble_monitor.c" "zigbee_monitor.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt esp_event esp_netif esp_timer esp_wifi ieee802154 nvs_flash)
