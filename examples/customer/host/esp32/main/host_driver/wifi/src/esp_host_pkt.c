// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>

#include "netdev_api.h"

#include "sdspi_drv.h"

#include "host_netif.h"

struct network_handle *sta_handle;

esp_err_t esp_host_pkt_sta_output(void *buffer, uint16_t len)
{
    struct pbuf *send_buffer = NULL;
    if (buffer == NULL || len == 0) {
        return ESP_FAIL;
    }

	send_buffer = malloc(sizeof(struct pbuf));
	assert(send_buffer);

	send_buffer->payload = malloc(len);
	assert(send_buffer->payload);

	send_buffer->len = len;

	memcpy(send_buffer->payload, buffer, len);

    return network_write(sta_handle, send_buffer);
}

esp_err_t esp_host_pkt_ap_output(void *buffer, uint16_t len)
{
    if (buffer == NULL || len == 0) {
        return ESP_FAIL;
    }

    //return esp_host_pkt_output(PKT_AP_ETH, buffer, len);
    return ESP_OK;
}

static void sta_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	uint32_t sta_ip = 0;
	int ret;

	rx_buffer = network_read(net_handle, 0);

    // send to lwip
/*
    printf("Recv len: %d\n", rx_buffer->len);

    for(int i = 0; i< rx_buffer->len; i++) {
        printf(" %x", rx_buffer->payload[i]);
        if(i % 10 == 0) {
            printf("\n");
        }
    }
    printf("\n");
*/
    esp_host_pkt_sta_input(rx_buffer->payload, rx_buffer->len);

    free(rx_buffer->payload);
    rx_buffer->payload = NULL;
    free(rx_buffer);
    rx_buffer = NULL;
}

void init_sta(void)
{
	sta_handle = network_open(STA_INTERFACE, sta_rx_callback);
	assert(sta_handle);
}