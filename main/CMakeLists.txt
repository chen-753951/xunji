idf_component_register(SRCS "main.c" "motor.c" "sensor.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer)
