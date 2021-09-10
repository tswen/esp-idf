/* SDIO host example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
#include "sdkconfig.h"

#if CONFIG_TRANSMIT_USE_SDIO

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"

#include "sdio_host_log.h"
#include "sdio_host_transport.h"

#include "host_serial_bus.h"

#define BUFFER_LEN     4096  

DMA_ATTR uint8_t send_buffer[BUFFER_LEN] = "";
DMA_ATTR uint8_t rcv_buffer[BUFFER_LEN] = "";
static xSemaphoreHandle pxMutex;

static const char TAG[] = "sdio_host";

int sdio_debugLevel = 2;    // print info log, set to 3 if you want to debug

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

static void sdio_recv_task(void* arg)
{
    const int wait_ms = 50;
    uint32_t intr_raw;
    host_serial_bus_handle_t dev = (host_serial_bus_handle_t)arg;
    while (1) {
        esp_err_t ret = sdio_host_wait_int(1000 / portTICK_RATE_MS);

        if (ret != SUCCESS) {
            continue;
        }

        spi_mutex_lock();
        ret = sdio_host_get_intr(&intr_raw, NULL);
        SDIO_ERROR_CHECK(ret);

        if (intr_raw == 0) {
            //SDIO_LOGW(TAG, "No intr\r\n");
            spi_mutex_unlock();
            continue;
        }

        ret = sdio_host_clear_intr(intr_raw);
        SDIO_ERROR_CHECK(ret);
        SDIO_LOGD(TAG, "intr raw: %x", intr_raw);

        if (intr_raw & HOST_SLC0_RX_NEW_PACKET_INT_ST) {
            SDIO_LOGD(TAG, "new packet coming");

            while (1) {
                size_t size_read = BUFFER_LEN;
                ret = sdio_host_get_packet(rcv_buffer, BUFFER_LEN, &size_read, wait_ms);

                if (ret == ERR_NOT_FOUND) {
                    SDIO_LOGE(TAG, "interrupt but no data can be read");
                    break;
                } else if (ret != SUCCESS && ret != ERR_NOT_FINISHED) {
                    SDIO_LOGE(TAG, "rx packet error: %08X", ret);
                    break;
                }

                if (dev->ops->event_cb) {
                    dev->ops->event_cb(dev, BUFF_FULL, rcv_buffer, size_read);
                }

                if (ret == SUCCESS) {
                    break;
                }
            }
        }
        spi_mutex_unlock();
    }

    vTaskDelete(NULL);
}

static int32_t esp32_sdio_write(host_serial_bus_handle_t dev, const void* data, size_t size)
{
    sdio_err_t err;
    int32_t length = size;

    if (data == NULL  || length > BUFFER_LEN) {
        ESP_LOGE(TAG, "Write data error, len:%d", length);
        return -1;
    }
    spi_mutex_lock();
    memcpy(send_buffer, data, size);

    if (size == 80) {
        printf("send len: %d\r\n", size);
    }

    err = sdio_host_send_packet(send_buffer, size);
    // Send timeout
    if (err == ERR_TIMEOUT) {
        SDIO_LOGW(TAG, "send timeout");
    }
    spi_mutex_unlock();

    return size;
}

static esp_err_t esp32_sdio_init(host_serial_bus_handle_t dev)
{
    sdio_err_t err;
    pxMutex = xSemaphoreCreateMutex();
    printf("Start SDIO test\r\n");
    // Make sure SDIO slave has been inited
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    SDIO_LOGI(TAG, "host ready, start initializing slave...");
    err = sdio_init();
    assert(err == ESP_OK);

    xTaskCreate(sdio_recv_task, "sdioRecvTask", 2048, dev, 16, NULL);
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

    ops->init = esp32_sdio_init;
    ops->write = esp32_sdio_write;

    dev->ops = ops;
    return ESP_OK;
}
#endif