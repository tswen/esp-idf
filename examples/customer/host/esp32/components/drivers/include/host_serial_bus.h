// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
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
//
#ifndef __host_serial_bus_H__
#define __host_serial_bus_H__

#include <stdlib.h>
#include <stdio.h>
#include "host_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEVICE_SDIO = 0,
    DEVICE_SPI,
    DEVICE_UART,
    DEVICE_MAX
} host_device_name;

typedef enum {
    IN_PROCESS,
    BUFF_FULL,
    SLAVE_RESET,
    SLAVE_READY,
} host_recv_status_t;

#define CONTROL_GET_PARAM 0x1
#define CONTROL_RECV_DONE 0x2

typedef struct host_device_handle *host_serial_bus_handle_t;

typedef esp_err_t (* host_event_callback_t)(host_serial_bus_handle_t dev, host_recv_status_t recv_status, const void *buffer, size_t size);

typedef struct
{
    esp_err_t  (*init)(host_serial_bus_handle_t dev);
    esp_err_t  (*deinit)(host_serial_bus_handle_t dev);
    int32_t (*write)(host_serial_bus_handle_t dev, const void *buffer, size_t size);
    esp_err_t  (*control)(host_serial_bus_handle_t dev, int cmd, void *args);
    esp_err_t (*event_cb)(host_serial_bus_handle_t dev, host_recv_status_t recv_status, const void *buffer, size_t size);
} host_device_ops_t;

struct host_device_handle
{
    host_device_name   device_name; 
    host_device_ops_t *ops;
};

/**
 * Open a device
 *
 * @param device_name host device name
 *
 * @return the handle of device
 */
host_serial_bus_handle_t host_serial_bus_open(host_device_name device_name);

/**
 * Perform a variety of control functions on devices.
 *
 * @param dev the pointer of device driver structure
 * @param cmd the command sent to device
 * @param arg the argument of command
 *
 * @return ESP_OK if success
 */
esp_err_t host_serial_bus_control(host_serial_bus_handle_t dev, int cmd, void *args);

/**
 * Register receive data callback
 *
 * @param dev the pointer of device driver structure
 * @param event_cb receive data callback
 *
 * @return ESP_OK if success
 *
 */
esp_err_t host_serial_bus_register_recv_callback(host_serial_bus_handle_t dev, host_event_callback_t event_cb);

/**
 * Write data to a device.
 *
 * @param dev the pointer of device driver structure
 * @param buffer the data buffer to be written to device
 * @param size the size of buffer
 *
 * @return the actually written size on successful, otherwise negative returned.
 *
 */
int32_t host_serial_bus_write(host_serial_bus_handle_t dev, const void *buffer, size_t size);

/**
 * close the device
 *
 * @param dev the pointer of device driver structure
 *
 * @return ESP_OK if success
 */
esp_err_t host_serial_bus_close(host_serial_bus_handle_t dev);

/**
 * Register SPI instance, used for internal
 *
 * @param dev the pointer of device driver structure
 *
 * @return ESP_OK if success
 */
esp_err_t host_spi_master_hw(host_serial_bus_handle_t dev);

/**
 * Register SDIO instance, used for internal
 *
 * @param dev the pointer of device driver structure
 *
 * @return ESP_OK if success
 */
esp_err_t host_sdio_master_hw(host_serial_bus_handle_t dev);

#ifdef __cplusplus
}
#endif

#endif /* __host_serial_bus_H__ */
