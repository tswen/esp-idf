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
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

#include "tusb.h"
#include "tusb_cdc_acm.h"

#include "cmd_wifi.h"

static const char *TAG = "rndis_wifi";
static EventGroupHandle_t wifi_event_group;
static bool reconnect = true;
const int CONNECTED_BIT = BIT0;
const int DISCONNECTED_BIT = BIT1;

uint8_t tud_network_mac_address[6] = {0x02,0x02,0x84,0x6A,0x96,0x00};
bool s_wifi_is_connected = false;
char* null_password = "";

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
            s_wifi_is_connected = true;
            xEventGroupClearBits(wifi_event_group, DISCONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Wi-Fi STA disconnected");
            s_wifi_is_connected = false;
            esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, NULL);

            if (reconnect && tud_ready()) {
                ESP_LOGI(TAG, "sta disconnect, reconnect...");
                esp_wifi_connect();
            } else {
                ESP_LOGI(TAG, "sta disconnect");
            }
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
            break;

        default:
            break;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t wifi_cmd_sta_join(const char* ssid, const char* pass)
{
    int bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, 0, 1, 0);

    wifi_config_t wifi_config = { 0 };
    wifi_config.sta.pmf_cfg.capable = true;

    strlcpy((char*) wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    if (pass) {
        strlcpy((char*) wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    }

    if (bits & CONNECTED_BIT) {
        reconnect = false;
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        xEventGroupWaitBits(wifi_event_group, DISCONNECTED_BIT, 0, 1, portTICK_RATE_MS);
    }

    reconnect = true;
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    esp_wifi_connect();

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, 0, 1, 5000/portTICK_RATE_MS);

    return ESP_OK;
}

char AP_Info[700];
static esp_err_t wifi_cmd_query(void)
{
    wifi_config_t cfg = {0};
    wifi_mode_t mode;

    memset(&cfg, 0, sizeof(cfg));

    esp_wifi_get_mode(&mode);
    if (WIFI_MODE_AP == mode) {
        esp_wifi_get_config(WIFI_IF_AP, &cfg);
        ESP_LOGI(TAG, "AP mode, %s %s", cfg.ap.ssid, cfg.ap.password);
    } else if (WIFI_MODE_STA == mode) {
        int bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, 0, 1, 0);
        if (bits & CONNECTED_BIT) {
            esp_wifi_get_config(WIFI_IF_STA, &cfg);
            ESP_LOGI(TAG, "+CWJAP=:%s,%d,%d,%d", cfg.sta.ssid, cfg.sta.channel, cfg.sta.listen_interval, cfg.sta.threshold.authmode);
            int lenth = sprintf(AP_Info, "\r\n+CWJAP=:%s,%d,%d,%d\r\nOK",  cfg.sta.ssid, cfg.sta.channel, cfg.sta.listen_interval, cfg.sta.threshold.authmode);
            tinyusb_cdcacm_write_queue(0, (uint8_t*)AP_Info, lenth);
            tinyusb_cdcacm_write_flush(0, 0);
        } else {
            ESP_LOGI(TAG, "sta mode, disconnected");
        }
    } else {
        ESP_LOGI(TAG, "NULL mode");
        return ESP_FAIL;
    }

    return ESP_OK;
}

// static bool wifi_cmd_sta_scan(const char* ssid)
// {
//     wifi_scan_config_t scan_config = { 0 };
//     scan_config.ssid = (uint8_t *) ssid;

//     ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
//     esp_wifi_scan_start(&scan_config, false);

//     return true;
// }

char *parse_elem(char * instr,char *search_str,int *buf_size)
{
    char *str = NULL;
    int len = 0;
    static char buf[256] = {0};
 
    str = strstr(instr,search_str);
    if(!str)
    {
        len = 0;
        return NULL;
    }
    len = str - instr;
    memset(buf,0,sizeof(buf));
    memcpy(buf,instr,len);
    *buf_size = strlen(buf);
    //printf("buf:%s\n",buf);
    return (char *)buf;
}

static esp_err_t at_set_cmd(char* Cmd, int length)
{
    char* str = NULL;
    int len = length;

    if (!strncmp(Cmd, "AT+CWJAP", 8)) {
        Cmd = Cmd + len + strlen("=");
        str = parse_elem(Cmd, ",", &len);
        if (str) {
            printf("the SSID is %s\r\n", str);
            Cmd = Cmd + len + strlen(",");
            printf("the PassWord is %s\r\n", Cmd);
            wifi_cmd_sta_join(str, Cmd);
        } else {
            printf("the SSID is %s\r\n", Cmd);
            wifi_cmd_sta_join(Cmd, null_password); 
        }
    } else {
        ESP_LOGE("esp_cmd_parse", "/* %s */ No such AT command", Cmd);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t at_get_cmd(char* Cmd)
{
    if (!strncmp(Cmd, "AT+CWJAP", 8)) {
        wifi_cmd_query();
        // Return the connected wifi information to USB
        ESP_LOGI("esp_cmd_parse", "Return the connected wifi information to USB");
        return ESP_OK;
    } else {
        ESP_LOGE("esp_cmd_parse", "/* %s */ No such AT command", Cmd);
        return ESP_FAIL;
    }
}

static esp_err_t at_cmd(char* Cmd)
{
    if (!strncmp(Cmd, "AT+CWLAP", 8)) {
        /* TODO */
        // List scan AP
        ESP_LOGI("esp_cmd_parse", "List scan AP");
        return ESP_OK;
    } else {
        ESP_LOGE("esp_cmd_parse", "/* %s */ No such AT command", Cmd);
        return ESP_FAIL;
    }
}

esp_err_t esp_cmd_parse(char* Cmd)
{
    char* str = NULL;
    int len = 0;

    // Parameter check
    if (!Cmd) {
        ESP_LOGE("esp_cmd_parse", "The Cmd is err");
        return ESP_FAIL;
    }

    if (!strncmp(Cmd, "AT+", 3)) {  // AT command processing

        printf("/** Str: %s **/\r\n", Cmd);

        str = parse_elem(Cmd, "=", &len);
        if (str) {
            printf("the Cmd is %s\r\n", str);
            if (!at_set_cmd(Cmd, len)) {
                return ESP_OK;
            }
            return ESP_FAIL;
        } else {
            str = parse_elem(Cmd, "?", &len);
            if (str) {
                if (!at_get_cmd(Cmd)) {
                    return ESP_OK;
                }
                return ESP_FAIL;
            }
            if (!at_cmd(Cmd)) {
                return ESP_OK;
            }
            ESP_LOGE("esp_cmd_parse", "AT command format error");
            return ESP_FAIL;
        }
    } else {
        ESP_LOGE("esp_cmd_parse", "/* %s */ Not an AT command", Cmd);
        return ESP_FAIL;
    }
    return ESP_OK;
}