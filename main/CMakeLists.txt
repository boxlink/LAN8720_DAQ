idf_component_register(SRCS  "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_event esp_netif log esp_eth ads111x esp-modbus) 
