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

#include "sdkconfig.h"

#if CONFIG_SLAVE_ESP32_SERIES

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#ifdef CONFIG_SPI_STREAM_MODE
#include "freertos/stream_buffer.h"
#elif defined(CONFIG_SPI_PACKET_MODE)
#include "freertos/ringbuf.h"
#endif

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "host_serial_bus.h"


/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define GPIO_HANDSHAKE CONFIG_SPI_HANDSHAKE_PIN
#define GPIO_MOSI CONFIG_SPI_MOSI_PIN
#define GPIO_MISO CONFIG_SPI_MISO_PIN
#define GPIO_SCLK CONFIG_SPI_SCLK_PIN
#define GPIO_CS CONFIG_SPI_CS_PIN

#ifdef CONFIG_SPI_QUAD_MODE
#define GPIO_WP CONFIG_SPI_WP_PIN
#define GPIO_HD CONFIG_SPI_HD_PIN
#endif

#define MASTER_HOST HSPI_HOST
#define DMA_CHAN 2

#define ESP_SPI_DMA_MAX_LEN  4092

#define CMD_HD_WRBUF_REG    0x01
#define CMD_HD_RDBUF_REG    0x02
#define CMD_HD_WRDMA_REG    0x03
#define CMD_HD_RDDMA_REG    0x04
#define CMD_HD_WR_END_REG   0x07
#define CMD_HD_INT0_REG     0x08

#define WRBUF_START_ADDR    0x0
#define RDBUF_START_ADDR    0x4

static const char* TAG = "MCU";

static xQueueHandle msg_queue;

#define BUFFER_SIZE  1024 * 8

static spi_device_handle_t handle;

#ifdef CONFIG_SPI_STREAM_MODE
static StreamBufferHandle_t spi_master_tx_ring_buf = NULL;
#elif defined(CONFIG_SPI_PACKET_MODE)
static RingbufHandle_t spi_master_tx_ring_buf = NULL;
static uint32_t item_size = 0;
#endif

typedef enum {
    SPI_NULL = 0,
    SPI_READ,         // slave -> master
    SPI_WRITE,             // maste -> slave
} spi_mode_t;

static xSemaphoreHandle pxMutex;

static uint8_t initiative_send_flag = 0;
static uint32_t plan_send_len = 0;
static uint8_t current_send_seq = 0;
static uint8_t current_recv_seq = 0;

typedef struct {
    uint32_t     magic    : 8;    // 0xFE
    uint32_t     send_seq : 8;
    uint32_t     send_len : 16;
} spi_send_opt_t;

typedef struct {
    uint32_t     direct : 8;
    uint32_t     seq_num : 8;
    uint32_t     transmit_len : 16;
} spi_recv_opt_t;

typedef struct {
    spi_mode_t direct;
} spi_msg_t;

void spi_mutex_lock(void)
{
    while (xSemaphoreTake(pxMutex, portMAX_DELAY) != pdPASS);
}

void spi_mutex_unlock(void)
{
    xSemaphoreGive(pxMutex);
}

/*
This ISR is called when the handshake line goes high.
There are two ways to trigger the GPIO interrupt:
1. Master sends data, slave has received successfully
2. Slave has data want to transmit
*/
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Give the semaphore.
    BaseType_t mustYield = false;
    spi_msg_t spi_msg = {
        .direct = SPI_NULL,
    };
    xQueueSendFromISR(msg_queue, (void*)&spi_msg, &mustYield);

    if (mustYield) {
        portYIELD_FROM_ISR();
    }
}

static void at_spi_master_send_data(uint8_t* data, uint32_t len)
{
    spi_transaction_t trans = {
#if defined(CONFIG_SPI_QUAD_MODE)
        .flags = SPI_TRANS_MODE_QIO,
        .cmd = CMD_HD_WRDMA_REG | (0x2 << 4),    // master -> slave command, donnot change
#elif defined(CONFIG_SPI_DUAL_MODE)
        .flags = SPI_TRANS_MODE_DIO,
        .cmd = CMD_HD_WRDMA_REG | (0x1 << 4),  
#else
        .cmd = CMD_HD_WRDMA_REG,    // master -> slave command, donnot change
#endif
        .length = len * 8,
        .tx_buffer = (void*)data
    };
    spi_device_polling_transmit(handle, &trans);
}

static void at_spi_master_recv_data(uint8_t* data, uint32_t len)
{
    spi_transaction_t trans = {
#if defined(CONFIG_SPI_QUAD_MODE)
        .flags = SPI_TRANS_MODE_QIO,
        .cmd = CMD_HD_RDDMA_REG | (0x2 << 4),    // master -> slave command, donnot change
#elif defined(CONFIG_SPI_DUAL_MODE)
        .flags = SPI_TRANS_MODE_DIO,
        .cmd = CMD_HD_RDDMA_REG | (0x1 << 4),         
#else
        .cmd = CMD_HD_RDDMA_REG,    // master -> slave command, donnot change
#endif
        .rxlength = len * 8,
        .rx_buffer = (void*)data
    };
    spi_device_polling_transmit(handle, &trans);
}

static void at_spi_rddma_done(void)
{
    spi_transaction_t end_t = {
        .cmd = CMD_HD_INT0_REG,
    };
    spi_device_polling_transmit(handle, &end_t);
}

static void at_spi_wrdma_done(void)
{
    spi_transaction_t end_t = {
        .cmd = CMD_HD_WR_END_REG,
    };
    spi_device_polling_transmit(handle, &end_t);
}


//Read SPI status(Slave send to master data length) transmit function
static spi_recv_opt_t spi_master_get_slave_status()
{
    spi_recv_opt_t recv_opt;
    spi_transaction_t trans = {
        .cmd = CMD_HD_RDBUF_REG,
        .addr = RDBUF_START_ADDR,
        .rxlength = 4 * 8,
        .rx_buffer = &recv_opt,
    };

    spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);

    return recv_opt;
}

// SPI status transmicurrent_send_seqt function(Master sendto slave data length) , address length is 0 bit(no address)
static void spi_master_set_trans_info(uint8_t send_seq, uint16_t send_len)
{
    spi_send_opt_t send_opt;
    send_opt.magic = 0xFE;
    send_opt.send_seq = send_seq;
    send_opt.send_len = send_len;

    spi_transaction_t trans = {
        .cmd = CMD_HD_WRBUF_REG,
        .addr = WRBUF_START_ADDR,
        .length = 4 * 8,
        .tx_buffer = &send_opt,
    };
    spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);
    // increment
    current_send_seq  = send_seq;
}

// SPI master sent to slave function
static int8_t spi_load_data(uint8_t* buf, int32_t len)
{
    if (len > ESP_SPI_DMA_MAX_LEN) {
        ESP_LOGE(TAG, "Send length errot, len:%d", len);
        return -1;
    }
    at_spi_master_send_data(buf, len);
    at_spi_wrdma_done();
    return 0;
}

static int32_t esp32_spi_write(host_serial_bus_handle_t dev, const void* data, size_t size)
{
    int32_t length = size;

#ifdef CONFIG_SPI_STREAM_MODE
    if (data == NULL  || length > BUFFER_SIZE) {
#elif defined(CONFIG_SPI_PACKET_MODE)
    // Each item stored in no-split/allow-split buffers will require an additional 8 bytes for a header.
    if (data == NULL  || length > BUFFER_SIZE - 8) {
#endif
        ESP_LOGE(TAG, "Write data error, len:%d", length);
        return -1;
    }

#ifdef CONFIG_SPI_STREAM_MODE
    length = xStreamBufferSend(spi_master_tx_ring_buf, data, size, portMAX_DELAY);
#elif defined(CONFIG_SPI_PACKET_MODE)
    if (xRingbufferSend(spi_master_tx_ring_buf, (void*)data, size,  portMAX_DELAY) == pdFALSE) {
        ESP_LOGE(TAG, "Send len %d to buffer error, please enlarge the buffer size", size);
        return -1;
    }
#endif

    if (initiative_send_flag == 0) {
        spi_mutex_lock();
#ifdef CONFIG_SPI_STREAM_MODE
        uint32_t tmp_send_len = xStreamBufferBytesAvailable(spi_master_tx_ring_buf);
        if (tmp_send_len > 0) {
            plan_send_len = tmp_send_len > ESP_SPI_DMA_MAX_LEN ? ESP_SPI_DMA_MAX_LEN : tmp_send_len;
            spi_master_set_trans_info(current_send_seq + 1, plan_send_len);
#elif defined(CONFIG_SPI_PACKET_MODE)
        vRingbufferGetInfo(spi_master_tx_ring_buf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size > 0) {
            spi_master_set_trans_info(current_send_seq + 1, size);
#endif
            initiative_send_flag = 1;
        }
        spi_mutex_unlock();
    }

    return length;
}

static void IRAM_ATTR spi_trans_control_task(void* arg)
{
    esp_err_t ret;
    host_serial_bus_handle_t dev = (host_serial_bus_handle_t)arg;
    spi_msg_t trans_msg = {0};
    uint32_t send_len = 0;

#ifdef CONFIG_SPI_PACKET_MODE
    uint8_t* transmit_point = NULL;
#endif

    uint8_t* trans_data = (uint8_t*)malloc(ESP_SPI_DMA_MAX_LEN * sizeof(uint8_t));
    if (trans_data == NULL) {
        ESP_LOGE(TAG, "malloc fail");
        return;
    }

    while (1) {
        xQueueReceive(msg_queue, (void*)&trans_msg, (portTickType)portMAX_DELAY);
        spi_mutex_lock();
        spi_recv_opt_t recv_opt = spi_master_get_slave_status();

        if (recv_opt.direct == SPI_WRITE) {  // master -> slave
            if (recv_opt.seq_num != current_send_seq) {
                ESP_LOGE(TAG, "SPI send seq error, %x, %x", recv_opt.seq_num, current_send_seq);
                if (recv_opt.seq_num == 1) {
                    ESP_LOGE(TAG, "Maybe SLAVE restart, ignore");
                    current_send_seq = recv_opt.seq_num;
                } else {
                    break;
                }
            }

            if (recv_opt.transmit_len > ESP_SPI_DMA_MAX_LEN || recv_opt.transmit_len == 0) {
                ESP_LOGE(TAG, "SPI send len error, %x", recv_opt.transmit_len);
                break;
            }

#ifdef CONFIG_SPI_STREAM_MODE
            send_len = xStreamBufferReceive(spi_master_tx_ring_buf, (void*) trans_data, plan_send_len, 0);

            if (send_len != plan_send_len) {
                ESP_LOGE(TAG, "Read len expect %d, but actual read %d", plan_send_len, send_len);
                break;
            }
            ret = spi_load_data(trans_data, plan_send_len);
#elif defined(CONFIG_SPI_PACKET_MODE)
            transmit_point = xRingbufferReceive(spi_master_tx_ring_buf, &plan_send_len, 0);
            if (plan_send_len == 0 || transmit_point == NULL) {
                ESP_LOGE(TAG, "Read len error, %d, %p", plan_send_len, transmit_point);
                continue;
            }
            ret = spi_load_data(transmit_point, plan_send_len);
#endif
            if (ret < 0) {
                ESP_LOGE(TAG, "Load data error");
                break;
            }
#ifdef CONFIG_SPI_STREAM_MODE
            // maybe streambuffer filled some data when SPI transimit, just consider it after send done, because send flag has already in SLAVE queue
            uint32_t tmp_send_len = xStreamBufferBytesAvailable(spi_master_tx_ring_buf);
            if (tmp_send_len > 0) {
                plan_send_len = tmp_send_len > ESP_SPI_DMA_MAX_LEN ? ESP_SPI_DMA_MAX_LEN : tmp_send_len;
#elif defined(CONFIG_SPI_PACKET_MODE)
            vRingbufferReturnItem(spi_master_tx_ring_buf, transmit_point);
            vRingbufferGetInfo(spi_master_tx_ring_buf, NULL, NULL, NULL, NULL, &item_size);
            if (item_size > 0) {
                plan_send_len = ESP_SPI_DMA_MAX_LEN;
#endif
                spi_master_set_trans_info(current_send_seq + 1, plan_send_len);
            } else {
                initiative_send_flag = 0;
            }

        } else if (recv_opt.direct == SPI_READ) {
            if (recv_opt.seq_num != ((current_recv_seq + 1) & 0xFF)) {
                ESP_LOGE(TAG, "SPI recv seq error, %x, %x", recv_opt.seq_num, (current_recv_seq + 1));
                if (recv_opt.seq_num == 1) {
                    ESP_LOGE(TAG, "Maybe SLAVE restart, ignore");
                } else {
                    break;
                }
            }

            if (recv_opt.transmit_len > BUFFER_SIZE || recv_opt.transmit_len == 0) {
                ESP_LOGE(TAG, "SPI read len error, %x, %x", recv_opt.transmit_len, BUFFER_SIZE);
                break;
            }

            current_recv_seq = recv_opt.seq_num;
            memset(trans_data, 0x0, recv_opt.transmit_len);
            at_spi_master_recv_data(trans_data, recv_opt.transmit_len);
            at_spi_rddma_done();
            if (dev->ops->event_cb) {
                dev->ops->event_cb(dev, BUFF_FULL, trans_data, recv_opt.transmit_len);
            }
        } else {
            ESP_LOGD(TAG, "Unknow direct: %d", recv_opt.direct);
            spi_mutex_unlock();
            continue;
        }

        spi_mutex_unlock();
    }

    free(trans_data);
    vTaskDelete(NULL);
}

inline void spi_bus_defalut_config(spi_bus_config_t* bus_cfg)
{
    bus_cfg->mosi_io_num = GPIO_MOSI;
    bus_cfg->miso_io_num = GPIO_MISO;
    bus_cfg->sclk_io_num = GPIO_SCLK;
#if defined(CONFIG_SPI_QUAD_MODE)
    bus_cfg->quadwp_io_num = GPIO_WP;
    bus_cfg->quadhd_io_num = GPIO_HD;
#else
    bus_cfg->quadwp_io_num = -1;
    bus_cfg->quadhd_io_num = -1;
#endif
    bus_cfg->max_transfer_sz = 14000;
}

inline void spi_device_default_config(spi_device_interface_config_t* dev_cfg)
{
    dev_cfg->clock_speed_hz = SPI_MASTER_FREQ_10M;
    dev_cfg->mode = 0;
    dev_cfg->spics_io_num = GPIO_CS;
    dev_cfg->cs_ena_pretrans = 8;
    dev_cfg->cs_ena_posttrans = 8;
    dev_cfg->command_bits = 8;
    dev_cfg->address_bits = 8;
    dev_cfg->dummy_bits = 8;
    dev_cfg->queue_size = 16;
    dev_cfg->flags = SPI_DEVICE_HALFDUPLEX;
    dev_cfg->input_delay_ns = 25;
}

static void init_master_hd(spi_device_handle_t* spi)
{
    //GPIO config for the handshake line.
    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = (1 << GPIO_HANDSHAKE)
    };

    //Create the semaphore.
    msg_queue = xQueueCreate(5, sizeof(spi_msg_t));

#ifdef CONFIG_SPI_STREAM_MODE
    spi_master_tx_ring_buf = xStreamBufferCreate(BUFFER_SIZE, 1);
#elif defined(CONFIG_SPI_PACKET_MODE)
    spi_master_tx_ring_buf = xRingbufferCreate(BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
#endif
    pxMutex = xSemaphoreCreateMutex();

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_PIN_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    //init bus
    spi_bus_config_t bus_cfg = {};
    spi_bus_defalut_config(&bus_cfg);
    ESP_ERROR_CHECK(spi_bus_initialize(MASTER_HOST, &bus_cfg, DMA_CHAN));

    //add device
    spi_device_interface_config_t dev_cfg = {};
    spi_device_default_config(&dev_cfg);
    ESP_ERROR_CHECK(spi_bus_add_device(MASTER_HOST, &dev_cfg, spi));
}

static esp_err_t  esp32_spi_init(host_serial_bus_handle_t dev)
{
    init_master_hd(&handle);
#ifdef CONFIG_SPI_STREAM_MODE
    ESP_LOGI(TAG, "Using stream mode");
#elif defined(CONFIG_SPI_PACKET_MODE)
    ESP_LOGI(TAG, "Using packet mode");
#endif
    xTaskCreate(spi_trans_control_task, "spi_trans_control_task", 1024 * 2, dev, 8, NULL);
    return ESP_OK;
}

esp_err_t host_spi_master_hw(host_serial_bus_handle_t dev)
{
    host_device_ops_t* ops = malloc(sizeof(host_device_ops_t));
    if (ops == NULL) {
        ESP_LOGE(TAG, "malloc fail");
        return ESP_ERR_NO_MEM;
    }

    memset(ops, 0x0, sizeof(host_device_ops_t));

    ops->init = esp32_spi_init;
    ops->write = esp32_spi_write;

    dev->ops = ops;
    return ESP_OK;
}
#endif
