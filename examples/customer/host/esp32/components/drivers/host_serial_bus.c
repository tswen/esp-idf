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
#include "stdio.h"
#include "host_serial_bus.h"

#include "host_config.h"
#include "sdkconfig.h"

static const char* TAG = "HOST_HAL";

int at_debugLevel = 2;

#define DEVICE_CHECK(a, str, ret_val) \
    if (!(a)) { \
        DRIVER_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str); \
        return (ret_val); \
    }

typedef struct {
    host_device_name device_name;
    esp_err_t (*register_hw)(host_serial_bus_handle_t dev);
} host_serial_bus_list_struct;

static host_serial_bus_list_struct host_serial_bus_list[] = {
#if CONFIG_TRANSMIT_USE_SPI
    {DEVICE_SPI, host_spi_master_hw},
#else
    {DEVICE_SDIO, host_sdio_master_hw},
#endif
};

host_serial_bus_handle_t host_serial_bus_open(host_device_name device_name)
{
    DEVICE_CHECK(device_name >= DEVICE_SDIO && device_name < DEVICE_MAX, "device error", NULL);
    esp_err_t ret = ESP_FAIL;
    host_serial_bus_handle_t dev = malloc(sizeof(struct host_device_handle));

    if (dev == NULL) {
        return NULL;
    }
    
    for (uint32_t loop = 0; loop < sizeof(host_serial_bus_list) / sizeof(host_serial_bus_list[0]); loop++) {
        if (device_name == host_serial_bus_list[loop].device_name) {
            if (host_serial_bus_list[loop].register_hw) {
                ret = host_serial_bus_list[loop].register_hw(dev);
                break;
            } else {
                return NULL;
            }
        }
    }

    if (ret != ESP_OK) {
        free(dev);
        return NULL;
    }

    ret = dev->ops->init(dev);

    if (ret != ESP_OK) {
        free(dev);
        return NULL;
    }

    return dev;
}

esp_err_t host_serial_bus_control(host_serial_bus_handle_t dev, int cmd, void* args)
{
    DEVICE_CHECK(dev != NULL, "drive not init", ESP_FAIL);
    return dev->ops->control(dev, cmd, args);
}

esp_err_t host_serial_bus_register_recv_callback(host_serial_bus_handle_t dev, host_event_callback_t event_cb)
{
    DEVICE_CHECK(dev != NULL, "drive not init", ESP_FAIL);
    dev->ops->event_cb = event_cb;
    return ESP_OK;
}

int32_t host_serial_bus_write(host_serial_bus_handle_t dev, const void* buffer, size_t size)
{
    DEVICE_CHECK(dev != NULL, "drive not init", -1);
    return dev->ops->write(dev, buffer, size);
}

esp_err_t host_serial_bus_close(host_serial_bus_handle_t dev)
{
    DEVICE_CHECK(dev != NULL, "drive not init", ESP_FAIL);
    dev->ops->deinit(dev);
    free(dev);
    dev = NULL;
    return ESP_OK;
}