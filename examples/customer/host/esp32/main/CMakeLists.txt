set(srcs)
set(icls)

# common
list(APPEND srcs
    "../../../common/esp_hosted_config.pb-c.c"
    )

# host common
list(APPEND srcs
     "../../host_common/commands.c"
     "../../host_common/transport_pserial.c"
     )

# other
list(APPEND srcs
    "dongle_main.c"
    "CLI_Commands.c"
    "Command_Parse.c"
    "uart.c"
    )

list(APPEND srcs
    "host_driver/platform_wrapper.c"
    "host_driver/net/netdev_api.c"
    "host_driver/net/netdev_stub.c"
    "host_driver/serial/serial_if.c"
    "host_driver/sdspi_driver/sdspi_drv.c"
    "host_driver/wifi/src/host_wifi_default.c"
    "host_driver/wifi/src/host_wifi_netif.c"
    "host_driver/wifi/src/esp_host_pkt.c"
    )

list(APPEND icls
    "../../../common/include"
    "../../host_common/include"
    "host_driver/include"
    "host_driver/wifi/include"
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${icls}
                       )
