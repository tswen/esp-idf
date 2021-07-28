/* USB Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// DESCRIPTION:
// This example contains minimal code to make ESP32-S2 based device
// recognizable by USB-host devices as a USB Serial Device.

#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

#include "nvs_flash.h"

#include "semphr.h"

#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

#include "tusb.h"

#include "cmd_wifi.h"

SemaphoreHandle_t semp;

#ifdef CONFIG_HEAP_TRACING
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM
#endif

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#define MAIN_CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_RNDIS_DESC_LEN + TUD_CDC_DESC_LEN)
#define ALT_CONFIG_TOTAL_LEN     (TUD_CONFIG_DESC_LEN + TUD_CDC_ECM_DESC_LEN + TUD_CDC_DESC_LEN)

#define EPNUM_NET_NOTIF   0x81
#define EPNUM_NET_OUT     0x02
#define EPNUM_NET_IN      0x82
#define EPNUM_CDC_NOTIF   0x83
#define EPNUM_CDC_OUT     0x04

static const char *TAG = "esp_rndis";
static uint8_t buf[CONFIG_USB_CDC_RX_BUFSIZE + 1];
extern bool s_wifi_is_connected;

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       NET | VENDOR | MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) | _PID_MAP(NET, 5) )

//------------- Configuration Descriptor -------------//
enum
{
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_INTERFACE,
  STRID_MAC,
  STRID_CDC,
};

enum
{
  ITF_NUM_CDC = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_CDC1,
  ITF_NUM_CDC1_DATA,
  ITF_NUM_TOTAL
};

enum
{
  CONFIG_ID_RNDIS = 0,
  CONFIG_ID_ECM   = 1,
  CONFIG_ID_COUNT
};

static tusb_desc_device_t const _desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_ESPRESSIF_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0101,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 0x01
};

// array of pointer to string descriptors
static char const* _desc_string_arr[] =
{
  [STRID_LANGID]       = (const char[]) { 0x09, 0x04 }, // supported language is English (0x0409)
  [STRID_MANUFACTURER] = "TinyUSB",                     // Manufacturer
  [STRID_PRODUCT]      = "TinyUSB Device",              // Product
  [STRID_SERIAL]       = "123456",                      // Serial
  [STRID_INTERFACE]    = "TinyUSB Network Interface",   // Interface Description
  [STRID_MAC]          = "",                            // Interface Description
  [STRID_CDC]          = "TinyUSB CDC"                  // Interface Description

  // STRID_MAC index is handled separately
};

static uint8_t const rndis_configuration[] =
{
  // Config number (index+1), interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_RNDIS+1, ITF_NUM_TOTAL, 0, MAIN_CONFIG_TOTAL_LEN, 0, 100),

  // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_RNDIS_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, EPNUM_NET_NOTIF, 8, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE),
  
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC1, 5, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, 0x80 | EPNUM_CDC_OUT, 64),
};

static uint8_t const ecm_configuration[] =
{
  // Config number (index+1), interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(CONFIG_ID_ECM+1, ITF_NUM_TOTAL, 0, ALT_CONFIG_TOTAL_LEN, 0, 100),

  // Interface number, description string index, MAC address string index, EP notification address and size, EP data address (out, in), and size, max segment size.
  TUD_CDC_ECM_DESCRIPTOR(ITF_NUM_CDC, STRID_INTERFACE, STRID_MAC, EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN, CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
  // CDC_DESCRIPTOR(CONFIG_ID_ECM+2, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, 0x80 | EPNUM_CDC_OUT, 64),
  TUD_CDC_DESCRIPTOR(2, 5, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, 0x80 | EPNUM_CDC_OUT, 64),
};

// Configuration array: RNDIS and CDC-ECM
// - Windows only works with RNDIS
// - MacOS only works with CDC-ECM
// - Linux will work on both
static uint8_t const * const _configuration_arr[CONFIG_ID_COUNT] =
{
  [CONFIG_ID_RNDIS] = rndis_configuration,
  [CONFIG_ID_ECM  ] = ecm_configuration,
};

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
    /* TODO */
}

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
esp_err_t pkt_wifi2usb(void *buffer, uint16_t len, void *eb)
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

char *Cmd = NULL;
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(0, buf, CONFIG_USB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        Cmd = (char*)buf;
        esp_cmd_parse(Cmd);

    } else {
        ESP_LOGE(TAG, "itf %d: itfRead error", itf);
    }

    /* write back */
    // tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    // tinyusb_cdcacm_write_flush(itf, 0);
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! itf:%d dtr:%d, rst:%d", itf, dtr, rst);
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

    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t tusb_cfg = {
        .descriptor = &_desc_device,
        .string_descriptor = _desc_string_arr,
        .config_descriptor = _configuration_arr[CONFIG_ID_RNDIS], // CONFIG_ID_ECM TODO
        .external_phy = false // In the most cases you need to use a `false` value
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    initialise_wifi();

    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
}
