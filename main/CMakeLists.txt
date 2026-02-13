idf_component_register(SRCS "bt_app_core.c"
                            "main.c"
                    PRIV_REQUIRES bt nvs_flash
                    PRIV_REQUIRES esp_adc
                    INCLUDE_DIRS ".")
