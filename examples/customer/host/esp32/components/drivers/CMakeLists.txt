set(srcs)
set(include_dirs)

if(CONFIG_TRANSMIT_USE_SDSPI)
    list(APPEND srcs "esp32/sdspi_host/sdspi_host.c" "esp32/sdspi_host/port.c")
    list(APPEND include_dirs esp32/sdspi_host/include)
else()
    list(APPEND srcs "host_serial_bus.c")
    list(APPEND include_dirs include)
    if(CONFIG_TRANSMIT_USE_SPI)
        if(CONFIG_SLAVE_ESP32_SERIES)
            list(APPEND srcs "esp32/spi/spi_master_esp32_series.c")
        endif()

        if(CONFIG_SLAVE_ESP8266)
            list(APPEND srcs "esp32/spi/spi_master_esp8266.c")
        endif()
    else()
        list(APPEND srcs "esp32/sdio/sdio_host_esp32.c"
                         "esp32/sdio/sdio_host_driver/sdio_host_hal.c"
                         "esp32/sdio/sdio_host_driver/port/sdio_driver_port.c")
        list(APPEND include_dirs esp32/sdio/sdio_host_driver/include)
    endif()
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "${include_dirs}"
                    PRIV_REQUIRES sdmmc spi_flash
                    )