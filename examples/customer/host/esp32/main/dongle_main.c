/**
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 * 
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <stdint.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_event.h"

#include "sdspi_drv.h"
#include "control.h"
#include "netdev_if.h"
#include "netdev_api.h"
#include "esp_host_pkt.h"
#include "uart.h"

#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#define NUM_RECORDS 100
static heap_trace_record_t trace_record[NUM_RECORDS]; // This buffer must be in internal RAM
#endif /* CONFIG_HEAP_TRACING */

static const char *TAG = "Dongle";

esp_netif_t *netif_sta;

/*
 * Register commands that can be used with FreeRTOS+CLI through the UDP socket.
 * The commands are defined in CLI-commands.c.
 */
void vRegisterCLICommands(void);

/**
  * @brief  Control path event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void control_path_event_handler(uint8_t event)
{
    wifi_event_t event_id;
    switch(event)
	{
		case STATION_CONNECTED:
		{
            printf("WiFi connected, start DHCP client\r\n");
            esp_event_post(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, NULL, sizeof(wifi_event_sta_connected_t), 0);
			break;
		}
		case STATION_STARTED:
		{
			//init_sta();
            printf("WiFi started\r\n");
            esp_event_post(WIFI_EVENT, WIFI_EVENT_STA_START, NULL, 0, 0);
			break;
		}
		case STATION_DISCONNECTED:
		{
			printf("station disconnected\n\r");
			break;
		}
		case SOFTAP_STARTED:
		{
			// printf("SOFTAP_STARTED\r\n");
			// init_ap();
			break;
		}
		case SOFTAP_STOPPED:
		{
			printf("softap stopped\n\r");
			break;
		}
		default:
		break;
	}
}

static void on_got_ip(void* arg, esp_event_base_t event_base,
                           int32_t event_id, void* event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));

}


/**
  * @brief  sdspi driver event handler callback
  * @param  event - sdspi_drv_events_e event to be handled
  * @retval None
  */
static void sdspi_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case SDSPI_DRIVER_ACTIVE:
		{
            printf("app main: control_path_init\r\n");
			/* Initiate control path now */
			control_path_init(control_path_event_handler);
			break;
		}
		default:
		break;
	}
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "host ready, start initializing slave...");
    esp_sdspi_init(NULL);

    control_path_init(control_path_event_handler);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));

    netif_sta = esp_netif_create_default_host_sta();
    assert(netif_sta);

    initialise_uart();

#ifdef CONFIG_HEAP_TRACING
    heap_trace_init_standalone(trace_record, NUM_RECORDS);
#endif
    
    /* Register commands with the FreeRTOS+CLI command interpreter. */
    vRegisterCLICommands();
}
