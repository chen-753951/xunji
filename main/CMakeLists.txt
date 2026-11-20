idf_component_register(SRCS "duoji.c" "main.c" "motor.c" "sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer esp_wifi esp_netif nvs_flash esp_event lwip)
