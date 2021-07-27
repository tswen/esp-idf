/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Peter Lawrence
 *
 * influenced by lrndis https://github.com/fetisov/lrndis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*
this appears as either a RNDIS or CDC-ECM USB virtual network adapter; the OS picks its preference

RNDIS should be valid on Linux and Windows hosts, and CDC-ECM should be valid on Linux and macOS hosts

The MCU appears to the host as IP address 192.168.7.1, and provides a DHCP server, DNS server, and web server.
*/
/*
Some smartphones *may* work with this implementation as well, but likely have limited (broken) drivers,
and likely their manufacturer has not tested such functionality.  Some code workarounds could be tried:

The smartphone may only have an ECM driver, but refuse to automatically pick ECM (unlike the OSes above);
try modifying ./examples/devices/net_lwip_webserver/usb_descriptors.c so that CONFIG_ID_ECM is default.

The smartphone may be artificially picky about which Ethernet MAC address to recognize; if this happens, 
try changing the first byte of tud_network_mac_address[] below from 0x02 to 0x00 (clearing bit 1).
*/

// #include "bsp/board.h"
// #include "tusb.h"
#include "esp_err.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

// #include "dhserver.h"
// #include "dnserver.h"
// #include "lwip/init.h"
// #include "lwip/timeouts.h"
// #include "httpd.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

#include "nvs_flash.h"

#include "semphr.h"

#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

static char* TAG = "esp_rndis";
/* lwip context */
// static struct netif netif_data;

/* shared between tud_network_recv_cb() and service_traffic() */

/* this is used by this code, ./class/net/net_driver.c, and usb_descriptors.c */
/* ideally speaking, this should be generated from the hardware's unique ID (if available) */
/* it is suggested that the first byte is 0x02 to indicate a link-local address */
uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};

#ifdef CONFIG_HEAP_TRACING
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM
#endif
/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "tusb.h"

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       NET | VENDOR | MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) | _PID_MAP(NET, 5) )

// String Descriptor Index
enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_INTERFACE,
  STRID_MAC
};

enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

enum
{
  CONFIG_ID_RNDIS = 0,
  CONFIG_ID_ECM   = 1,
  CONFIG_ID_COUNT
};

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) device class
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0101,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = CONFIG_ID_COUNT // multiple configurations
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#define MAIN_CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_RNDIS_DESC_LEN)
#define ALT_CONFIG_TOTAL_LEN     (TUD_CONFIG_DESC_LEN + TUD_CDC_ECM_DESC_LEN)

#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X || CFG_TUSB_MCU == OPT_MCU_LPC40XX
  // LPC 17xx and 40xx endpoint type (bulk/interrupt/iso) are fixed by its number
  // 0 control, 1 In, 2 Bulk, 3 Iso, 4 In etc ...
  #define EPNUM_NET_NOTIF   0x81
  #define EPNUM_NET_OUT     0x02
  #define EPNUM_NET_IN      0x82

#elif CFG_TUSB_MCU == OPT_MCU_SAMG
  // SAMG doesn't support a same endpoint number with different direction IN and OUT
  //    e.g EP1 OUT & EP1 IN cannot exist together
  #define EPNUM_NET_NOTIF   0x81
  #define EPNUM_NET_OUT     0x02
  #define EPNUM_NET_IN      0x83

#else
  #define EPNUM_NET_NOTIF   0x81
  #define EPNUM_NET_OUT     0x02
  #define EPNUM_NET_IN      0x82
#endif

bool s_wifi_is_connected = false;
static uint8_t const rndis_configuration[] =
{
  // Config number (index+1), interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_RNDIS+1, ITF_NUM_TOTAL, 0, MAIN_CONFIG_TOTAL_LEN, 0, 100),

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_RNDIS_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, EPNUM_NET_NOTIF, 8, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE),
};

static uint8_t const ecm_configuration[] =
{
  // Config number (index+1), interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_ECM+1, ITF_NUM_TOTAL, 0, ALT_CONFIG_TOTAL_LEN, 0, 100),

  // Interface number, description string index, MAC address string index, EP notification address and size, EP data address (out, in), and size, max segment size.
  TUD_CDC_ECM_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, STRID_MAC, EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

// Configuration array: RNDIS and CDC-ECM
// - Windows only works with RNDIS
// - MacOS only works with CDC-ECM
// - Linux will work on both
static uint8_t const * const configuration_arr[2] =
{
  [CONFIG_ID_RNDIS] = rndis_configuration,
  [CONFIG_ID_ECM  ] = ecm_configuration
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  return (index < CONFIG_ID_COUNT) ? configuration_arr[index] : NULL;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const* string_desc_arr [] =
{
  [STRID_LANGID]       = (const char[]) { 0x09, 0x04 }, // supported language is English (0x0409)
  [STRID_MANUFACTURER] = "TinyUSB",                     // Manufacturer
  [STRID_PRODUCT]      = "TinyUSB Device",              // Product
  [STRID_SERIAL]       = "123456",                      // Serial
  [STRID_INTERFACE]    = "TinyUSB Network Interface"    // Interface Description

  // STRID_MAC index is handled separately
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  unsigned int chr_count = 0;

  if (STRID_LANGID == index)
  {
    memcpy(&_desc_str[1], string_desc_arr[STRID_LANGID], 2);
    chr_count = 1;
  }
  else if (STRID_MAC == index)
  {
    // Convert MAC address into UTF-16

    for (unsigned i=0; i<sizeof(tud_network_mac_address); i++)
    {
      _desc_str[1+chr_count++] = "0123456789ABCDEF"[(tud_network_mac_address[i] >> 4) & 0xf];
      _desc_str[1+chr_count++] = "0123456789ABCDEF"[(tud_network_mac_address[i] >> 0) & 0xf];
    }
  }
  else
  {
    // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
    // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > (TU_ARRAY_SIZE(_desc_str) - 1)) chr_count = TU_ARRAY_SIZE(_desc_str) - 1;

    // Convert ASCII string into UTF-16
    for (unsigned int i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}

bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
  if (s_wifi_is_connected) {
    esp_wifi_internal_tx(ESP_IF_WIFI_STA, src, size);
  }
  tud_network_recv_renew();
  return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
  uint16_t len = arg;

  /* traverse the "pbuf chain"; see ./lwip/src/core/pbuf.c for more info */
  memcpy(dst, ref, len);
  return len;
}

void tud_network_init_cb(void)
{
}

SemaphoreHandle_t semp;
bool tud_network_wait_xmit(uint32_t ms)
{
  if (xSemaphoreTake(semp, ms/portTICK_PERIOD_MS) == pdTRUE) {
    xSemaphoreGive(semp);
    return true;
  }

  return false;
}

void tud_network_set_xmit_status(bool enable)
{
    if (enable == true) {
      xSemaphoreGive(semp);
    } else {
      xSemaphoreTake(semp, 0);
    }
}

#define DELAY_TICK    10
static esp_err_t pkt_wifi2usb(void *buffer, uint16_t len, void *eb)
{
    uint32_t loop = DELAY_TICK;
    if (!tud_ready()) {
      esp_wifi_internal_free_rx_buffer(eb);
      return ERR_USE;
    }
    
    // loop = DELAY_TICK;
    // while(loop-- && !tud_network_can_xmit()) {
    //   vTaskDelay(1);
    // }
    if (tud_network_wait_xmit(100)) {
      /* if the network driver can accept another packet, we make it happen */
      if (tud_network_can_xmit()) {
          tud_network_xmit(buffer, len);
      }
    }

    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
        case WIFI_EVENT_STA_START:
          esp_wifi_get_mac(ESP_IF_WIFI_STA, tud_network_mac_address);
          esp_wifi_connect();
          break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA connected");
            esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, pkt_wifi2usb);
            // s_wifi_is_started = true;
            s_wifi_is_connected = true;

            // esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, pkt_wifi2usb);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA disconnected");
            s_wifi_is_connected = false;
            // esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);
            if (tud_ready()) {
              esp_wifi_connect();
            }
            break;

        default:
            break;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "huawei-chenwu",
        },
    };
    // esp_netif_dhcpc_stop(sta_netif);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
}

void app_main(void)
{
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  vSemaphoreCreateBinary(semp);
  /* initialize TinyUSB */
  // board_init();
  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t tusb_cfg = {0}; // the configuration using default values
  ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
  
  tusb_init();

  wifi_init();
#ifdef CONFIG_HEAP_TRACING
    heap_trace_init_standalone(trace_record, NUM_RECORDS);
    heap_trace_start(HEAP_TRACE_LEAKS);
#endif

  while (1)
  {
    tud_task();
  }
}
