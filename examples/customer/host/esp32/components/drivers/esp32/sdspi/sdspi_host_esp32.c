/* SDIO host example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
#include "sdkconfig.h"

#if CONFIG_TRANSMIT_USE_SDSPI

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"

#include "host_serial_bus.h"

#include "sdspi_host.h"
#include "port.h"

#define BUFFER_LEN     4096  

static const char TAG_TX[] = "H -> S";
static const char TAG_RX[] = "S -> H";

DMA_ATTR uint8_t send_buffer[BUFFER_LEN] = "";
DMA_ATTR uint8_t rcv_buffer[BUFFER_LEN] = "";
static xSemaphoreHandle pxMutex;

static const char TAG[] = "sdspi_host";

spi_context_t context;

#define SDIO_ERROR_CHECK(x) do {                                         \
        sdio_err_t __err_rc = (x);                                       \
        if (__err_rc != SUCCESS) {                                       \
            SDIO_LOGE(TAG, "file: \"%s\" line %d\nfunc: %s\n error: %d\n", __FILE__, __LINE__,    \
                      __FUNCTION__, x);                 \
        }                                                               \
    } while(0);

void spi_mutex_lock(void)
{
    while (xSemaphoreTake(pxMutex, portMAX_DELAY) != pdPASS);
}

void spi_mutex_unlock(void)
{
    xSemaphoreGive(pxMutex);
}

static void sdspi_recv_task(void* pvParameters)
{
    esp_err_t ret;
    uint8_t flag = 1;
    uint32_t intr_raw;
    host_serial_bus_handle_t dev = (host_serial_bus_handle_t)pvParameters;

    while (1) {

        if (flag) {
            ret = at_spi_wait_int(100);
        } else {
            ret = at_spi_wait_int(portMAX_DELAY);
        }

        if (ret == ESP_ERR_TIMEOUT) {
            flag = 0;
            continue;
        }

        assert(ret == ESP_OK);

        spi_mutex_lock();

        ret = at_sdspi_get_intr(&intr_raw);
        assert(ret == ESP_OK);

        ret = at_sdspi_clear_intr(intr_raw);
        assert(ret == ESP_OK);

        if (intr_raw & HOST_SLC0_RX_NEW_PACKET_INT_ST) {
            size_t size_read = BUFFER_LEN;
            esp_err_t err = at_sdspi_get_packet(&context, rcv_buffer, BUFFER_LEN, &size_read);

            if (err == ESP_ERR_NOT_FOUND) {
                ESP_AT_LOGE(TAG, "interrupt but no data can be read");
                break;
            } else if (err != ESP_OK && err != ESP_ERR_NOT_FINISHED) {
                ESP_AT_LOGE(TAG, "rx packet error: %08X", err);
            }

            if (dev->ops->event_cb) {
                dev->ops->event_cb(dev, BUFF_FULL, rcv_buffer, size_read);
            }

            memset(rcv_buffer, '\0', sizeof(rcv_buffer));
        }
        spi_mutex_unlock();
    }
}

static int32_t esp32_sdspi_write(host_serial_bus_handle_t dev, const void* data, size_t size)
{
    esp_err_t err;
    int32_t length = size;

    if (data == NULL  || length > BUFFER_LEN) {
        ESP_LOGE(TAG, "Write data error, len:%d", length);
        return -1;
    }
    spi_mutex_lock();
    memcpy(send_buffer, data, size);

    //ets_printf("TX len:%d\n", size);
    //ESP_LOG_BUFFER_HEXDUMP(TAG_TX, send_buffer, size, ESP_LOG_INFO);

    err = at_sdspi_send_packet(&context, send_buffer, size, UINT32_MAX);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Send error, %d\n", err);
    }

    spi_mutex_unlock();

    return size;
}

static esp_err_t esp32_sdspi_init(host_serial_bus_handle_t dev)
{
    esp_err_t err;
    pxMutex = xSemaphoreCreateMutex();
    printf("Start SDSPI test\r\n");
    // Make sure SDIO slave has been inited
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGE(TAG, "host ready, start initializing slave...");
    err = at_sdspi_init();
    assert(err == ESP_OK);
    memset(&context, 0x0, sizeof(spi_context_t));
    xTaskCreate(sdspi_recv_task, "sdspi_recv_task", 2048, dev, 16, NULL);
    return ESP_OK;
}

esp_err_t host_sdio_master_hw(host_serial_bus_handle_t dev)
{
    host_device_ops_t* ops = malloc(sizeof(host_device_ops_t));
    if (ops == NULL) {
        ESP_LOGE(TAG, "malloc fail");
        return ESP_ERR_NO_MEM;
    }

    memset(ops, 0x0, sizeof(host_device_ops_t));

    ops->init = esp32_sdspi_init;
    ops->write = esp32_sdspi_write;

    dev->ops = ops;
    return ESP_OK;
}
#endif