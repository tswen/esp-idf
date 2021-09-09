/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2019 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef __ESP_HOST_PKT_H__
#define __ESP_HOST_PKT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Output an 802.3 data frame to WiFi station interface
  *
  * @param[in]  void *buffer: the buffer to be transmitted
  * @param[in]  uint16_t len: the length of buffer
  *
  * @return
  *         - ESP_OK: success
  *         - others: failed
  */
esp_err_t esp_host_pkt_sta_output(void *buffer, uint16_t len);

/** 
  * @brief  Output an 802.3 data frame to WiFi soft-AP interface
  *
  * @param[in]  void *buffer: the buffer to be transmitted
  * @param[in]  uint16_t len: the length of buffer
  *
  * @return
  *         - ESP_OK: success
  *         - others: failed
  */
esp_err_t esp_host_pkt_ap_output(void *buffer, uint16_t len);

/** 
  * @brief  ESP Host ethernet packet input process
  * 
  * @param[in]  uint8_t pkt_type: packet type defined in esp_host_pkt_type_t, set to PKT_STA_ETH or PKT_AP_ETH
  * @param[in]  const void *buffer: a pointer to ethernet packet payload
  * @param[in]  uint16_t len: ethernet packet payload length in bytes
  * 
  * @return
  *    - ESP_OK: succeed
  *    - others:  failed
  */
esp_err_t esp_host_eth_pkt_input(uint8_t pkt_type, const void *buffer, uint16_t len);

void init_sta(void);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HOST_PKT_H__ */