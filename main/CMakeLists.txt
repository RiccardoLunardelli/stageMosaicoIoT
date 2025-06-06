idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        json
        driver
        mqtt
        lwip
        freertos
        log
        esp_system
        wpa_supplicant
        esp-modbus
)