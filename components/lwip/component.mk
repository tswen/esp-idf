#
# Component Makefile
#
COMPONENT_SUBMODULES += lwip

COMPONENT_ADD_INCLUDEDIRS := \
	include/apps \
	include/apps/sntp \
	lwip/src/include \
	port/esp32/include \
	port/esp32/include/arch

COMPONENT_SRCDIRS := \
	apps/dhcpserver \
	apps/ping \
	apps/sntp \
	lwip/src/api \
	lwip/src/apps/sntp \
	lwip/src/apps/netbiosns \
	lwip/src/core \
	lwip/src/core/ipv4 \
	lwip/src/core/ipv6 \
	lwip/src/netif \
	port/esp32 \
	port/esp32/freertos \
	port/esp32/hooks \
	port/esp32/netif \
	port/esp32/debug

ifndef CONFIG_IDF_TARGET_ESP32
    COMPONENT_OBJEXCLUDE := port/esp32/netif/ethernetif.o
endif
ifndef CONFIG_VFS_SUPPORT_IO
    COMPONENT_OBJEXCLUDE += port/esp32/vfs_lwip.o
else
    COMPONENT_OBJEXCLUDE += port/esp32/no_vfs_syscalls.o
endif
ifndef CONFIG_OPENTHREAD_ENABLED
    COMPONENT_OBJEXCLUDE += port/esp32/netif/openthreadif.o
endif

ifndef CONFIG_LWIP_ICMP
    COMPONENT_OBJEXCLUDE += apps/ping/esp_ping.o apps/ping/ping.o apps/ping/ping_sock.o
endif

ifdef CONFIG_LWIP_PPP_SUPPORT
    COMPONENT_SRCDIRS += lwip/src/netif/ppp lwip/src/netif/ppp/polarssl
endif

CFLAGS += -Wno-address  # lots of LWIP source files evaluate macros that check address of stack variables

lwip/src/netif/ppp/ppp.o: CFLAGS += -Wno-uninitialized
lwip/src/netif/ppp/pppos.o: CFLAGS += -Wno-implicit-fallthrough
lwip/src/core/tcp.o: CFLAGS += -Wno-type-limits

COMPONENT_ADD_LDFRAGMENTS += linker.lf
