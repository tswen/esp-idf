set(srcs  "sdio_host_hal.c")

list(APPEND srcs "port/platform_os.c"
                 "port/sdio_driver_port.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES sdmmc)
