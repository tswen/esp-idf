// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
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


#ifndef _HOST_NETIF_H_
#define _HOST_NETIF_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Receive an 802.3 data frame from the WiFi station interface
  *
  * This function will forward frames to netif.
  *
  * @param[in]  buffer: received data
  * @param[in]  len: length of the data frame
  *
  * @return
  *         - ESP_OK
  */
esp_err_t esp_host_pkt_sta_input(void *buffer, uint16_t len);

/**
  * @brief  Receive an 802.3 data frame from the WiFi soft-AP interface
  *
  * This function will forward frames to netif.
  *
  * @param[in]  buffer: received data
  * @param[in]  len: length of the data frame
  *
  * @return
  *         - ESP_OK
  */
esp_err_t esp_host_pkt_ap_input(void *buffer, uint16_t len);

/**
  * @brief  Output an 802.3 data frame to WiFi station interface
  *
  * @param[in]  buffer: the buffer to be transmitted
  * @param[in]  len: the length of buffer
  *
  * @return
  *         - ESP_OK: success
  *         - others: failed
  */
esp_err_t esp_host_pkt_sta_output(void *buffer, uint16_t len);

/** 
  * @brief  Output an 802.3 data frame to WiFi soft-AP interface
  *
  * @param[in]  buffer: the buffer to be transmitted
  * @param[in]  len: the length of buffer
  *
  * @return
  *         - ESP_OK: success
  *         - others: failed
  */
esp_err_t esp_host_pkt_ap_output(void *buffer, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /*  _HOST_NETIF_H_ */