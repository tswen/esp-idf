idf_component_register(SRCS "src/host_wifi_default.c"
                            "src/host_wifi_netif.c"
                            "src/esp_host_pkt.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_event
                    PRIV_REQUIRES nvs_flash esp_netif esp_timer
                    )

