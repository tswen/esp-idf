// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp_netif.h"
#include "esp_log.h"
#include "host_wifi_netif.h"
#include "host_netif.h"

//
//  Purpose of this module is provide object oriented abstraction to wifi interfaces
//  in order to integrate wifi as esp-netif driver
//

/**
 * @brief WiFi netif driver structure
 */
typedef struct wifi_netif_driver {
    esp_netif_driver_base_t base;
    esp_wifi_interface_t wifi_if;
}* wifi_netif_driver_t;

typedef esp_err_t (*host_rxcb_t)(void *buffer, uint16_t len, void *eb);

static const char* TAG = "host_wifi_netif";

/**
 * @brief Local storage for netif handles and callbacks for specific wifi interfaces
 */
static esp_netif_receive_t s_wifi_rxcbs[MAX_WIFI_IFS] = { NULL };
static esp_netif_t *s_wifi_netifs[MAX_WIFI_IFS] = { NULL };
static host_rxcb_t s_host_rxcbs[MAX_WIFI_IFS] = { NULL };

int esp_host_internal_tx(esp_wifi_interface_t wifi_if, void *buffer, uint16_t len)
{
    esp_err_t ret;

    if (wifi_if == ESP_WIFI_IF_STA) {
        ret = esp_host_pkt_sta_output(buffer, len);
    } else if (wifi_if == ESP_WIFI_IF_AP) {
        ret = esp_host_pkt_ap_output(buffer, len);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t esp_host_pkt_sta_input(void *buffer, uint16_t len)
{
    if (s_host_rxcbs[ESP_WIFI_IF_STA]) {
        return s_host_rxcbs[ESP_WIFI_IF_STA](buffer, len, buffer);
    }

    return ESP_OK;
}

esp_err_t esp_host_pkt_ap_input(void *buffer, uint16_t len)
{
    if (s_host_rxcbs[ESP_WIFI_IF_AP]) {
        return s_host_rxcbs[ESP_WIFI_IF_AP](buffer, len, buffer);
    }

    return ESP_OK;
}

esp_err_t esp_host_internal_reg_rxcb(esp_wifi_interface_t ifx, host_rxcb_t fn)
{
    esp_err_t ret;
    s_host_rxcbs[ifx] = fn;
/*
     if (ifx == ESP_WIFI_IF_STA) {
        ret = esp_host_register_callback(STA_PKT, (host_cb_t)esp_host_pkt_sta_input);
    } else if (ifx == ESP_WIFI_IF_AP) {
        ret = esp_host_register_callback(AP_PKT, (host_cb_t)esp_host_pkt_ap_input);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }
*/
    return ESP_OK;
}

/**
 * @brief WiFi netif driver IO functions, a thin glue layer
 *         to the original wifi interface API
 */
static esp_err_t wifi_sta_receive(void *buffer, uint16_t len, void *eb)
{
    return s_wifi_rxcbs[ESP_WIFI_IF_STA](s_wifi_netifs[ESP_WIFI_IF_STA], buffer, len, eb);
}

static esp_err_t wifi_ap_receive(void *buffer, uint16_t len, void *eb)
{
    return s_wifi_rxcbs[ESP_WIFI_IF_AP](s_wifi_netifs[ESP_WIFI_IF_AP], buffer, len, eb);
}

static void wifi_free(void *h, void* buffer)
{
}

static esp_err_t wifi_transmit(void *h, void *buffer, size_t len)
{
    wifi_netif_driver_t driver = h;
    return esp_host_internal_tx(driver->wifi_if, buffer, len);
}

static esp_err_t wifi_transmit_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
{
    wifi_netif_driver_t driver = h;
    return esp_host_internal_tx(driver->wifi_if, buffer, len);
}

static esp_err_t wifi_driver_start(esp_netif_t * esp_netif, void * args)
{
    wifi_netif_driver_t driver = args;
    driver->base.netif = esp_netif;
    esp_netif_driver_ifconfig_t driver_ifconfig = {
            .handle =  driver,
            .transmit = wifi_transmit,
            .transmit_wrap= wifi_transmit_wrap,
            .driver_free_rx_buffer = wifi_free
    };

    return esp_netif_set_driver_config(esp_netif, &driver_ifconfig);
}

void esp_host_destroy_if_driver(wifi_netif_driver_t h)
{
    free(h);
}

wifi_netif_driver_t esp_host_create_if_driver(esp_wifi_interface_t wifi_if)
{
    wifi_netif_driver_t driver = calloc(1, sizeof(struct wifi_netif_driver));
    if (driver == NULL) {
        ESP_LOGE(TAG, "No memory to create a wifi interface handle");
        return NULL;
    }
    driver->wifi_if = wifi_if;
    driver->base.post_attach = wifi_driver_start;
    return driver;
}

esp_err_t esp_host_get_if_mac(wifi_netif_driver_t ifx, uint8_t mac[6])
{
    esp_wifi_interface_t wifi_interface = ifx->wifi_if;

    return wifi_get_mac(wifi_interface, mac);
}

bool esp_host_is_if_ready_when_started(wifi_netif_driver_t ifx)
{
    // WiFi rxcb to be register wifi rxcb on start for AP only, station gets it registered on connect event
    return (ifx->wifi_if == ESP_WIFI_IF_AP);
}

esp_err_t esp_host_register_if_rxcb(wifi_netif_driver_t ifx, esp_netif_receive_t fn, void * arg)
{
    if (ifx->base.netif != arg) {
        ESP_LOGE(TAG, "Invalid argument: supplied netif=%p does not equal to interface netif=%p", arg, ifx->base.netif);
        return ESP_ERR_INVALID_ARG;
    }
    esp_wifi_interface_t wifi_interface = ifx->wifi_if;
    s_wifi_rxcbs[wifi_interface] = fn;
    host_rxcb_t rxcb = NULL;
    esp_err_t ret;

    switch (wifi_interface) {
        case ESP_WIFI_IF_STA:
            rxcb = wifi_sta_receive;
            break;

        case ESP_WIFI_IF_AP:
            rxcb = wifi_ap_receive;
            break;

        default:
            break;
    }

    if (rxcb == NULL) {
        ESP_LOGE(TAG, "Unknown wifi interface id if=%d", wifi_interface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_wifi_netifs[wifi_interface] = ifx->base.netif;
    if ((ret = esp_host_internal_reg_rxcb(wifi_interface,  rxcb)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_host_internal_reg_rxcb for if=%d failed with %d", wifi_interface, ret);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}
