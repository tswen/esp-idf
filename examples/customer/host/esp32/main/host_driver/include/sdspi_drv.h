// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

/** prevent recursive inclusion **/
#ifndef __SDSPI_DRV_H
#define __SDSPI_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes **/
#include "common.h"

/** constants/macros **/
#define MAX_NETWORK_INTERFACES  2
#define STA_INTERFACE           "ESP_STATION"
#define SOFTAP_INTERFACE        "ESP_SOFTAP"

typedef enum sdspi_drv_events_s {
	SDSPI_DRIVER_ACTIVE
} sdspi_drv_events_e;

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/
esp_err_t esp_sdspi_init(void(*sdspi_drv_evt_handler)(uint8_t));

esp_err_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen);

struct esp_private {
	uint8_t     if_type;
	uint8_t     if_num;
	void        *netdev;
};

#ifdef __cplusplus
}
#endif

#endif
