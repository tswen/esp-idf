#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

#include "tusb.h"

#include "cmd_wifi.h"

static const char *TAG = "rndis_wifi";

uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};
bool s_wifi_is_connected = false;

esp_err_t pkt_wifi2usb(void *buffer, uint16_t len, void *eb);

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
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    // wifi_config_t wifi_config = {
    //     .sta = {
    //         .ssid = "huawei-chenwu",
    //         .password = ""
    //     },
    // };
    // esp_netif_dhcpc_stop(sta_netif);
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
}

void wifi_test(void)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "huawei-chenwu",
            .password = ""
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    esp_wifi_connect();
}