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

#if CONFIG_SLAVE_ESP8266

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "esp_system.h"
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

static const char* TAG = "SPI master";

static xQueueHandle rdySem;
static RingbufHandle_t at_spi_master_receive_ring_buf = NULL;
static RingbufHandle_t at_spi_master_send_ring_buf = NULL;

#define RING_BUFFER_SIZE  1024 * 4

static uint32_t ring_buffer_filled_byte = 0;

/* SPI data cmd definition */
#define SPI_MASTER_WRITE_DATA_TO_SLAVE_CMD     2
#define SPI_MASTER_READ_DATA_FROM_SLAVE_CMD    3

/* SPI status cmd definition */
#define SPI_MASTER_WRITE_STATUS_TO_SLAVE_CMD   1
#define SPI_MASTER_READ_STATUS_FROM_SLAVE_CMD  4

static spi_device_handle_t handle;

static uint32_t total_send_len = 0;
typedef enum {
    SPI_NULL = 0,
    SPI_READ,         // slave -> master
    SPI_WRITE             // maste -> slave
} spi_mode_t;

typedef struct {
    uint32_t     magic    : 8;
    uint32_t     send_seq : 8;
    uint32_t     send_len : 16;
} spi_send_opt_t;

typedef struct {
    uint32_t     direct : 8;
    uint32_t     seq_num : 8;
    uint32_t     transmit_len : 16;
} spi_recv_opt_t;

static xSemaphoreHandle pxMutex;

static bool allow_write_status = true;   // If SPI is writing, not allow write status....
static bool allow_read_status = true;
static uint8_t current_send_seq = 0;
static uint8_t current_recv_seq = 0;

static bool mcu_send_flag = false;

static bool transmit_done_flag = true;
static uint16_t tmp_send_len = 0;

DMA_ATTR uint8_t recv_data[RING_BUFFER_SIZE];

void IRAM_ATTR spi_mutex_lock(void)
{
    xSemaphoreTake(pxMutex, portMAX_DELAY);
}

void IRAM_ATTR spi_mutex_unlock(void)
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
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) {
        portYIELD_FROM_ISR();
    }

}

// SPI data transmit function, address length is 8bit
static void IRAM_ATTR spi_master_tran_data(spi_mode_t mode, uint32_t* data, uint32_t len)
{
    spi_transaction_t t;
    uint32_t send_len = 0;
    memset(&t, 0x0, sizeof(t));

    send_len = (len + 3) >> 2;

    spi_transaction_ext_t trans = (spi_transaction_ext_t) {
        .base = {
#ifdef CONFIG_SPI_DUAL_MODE
            .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_MODE_DIO,
#else
            .flags = SPI_TRANS_VARIABLE_ADDR,
#endif
        },
        .address_bits = 8,
    };

    if (mode == SPI_READ) {
        trans.base.cmd = SPI_MASTER_READ_DATA_FROM_SLAVE_CMD;
        trans.base.rxlength = send_len * 32;
        trans.base.rx_buffer = (void*)data;
    } else if (mode == SPI_WRITE) {
        trans.base.cmd = SPI_MASTER_WRITE_DATA_TO_SLAVE_CMD;
        //trans.base.rxlength = 0;
        //trans.base.rx_buffer = NULL;
        trans.base.length = send_len * 32;
        trans.base.tx_buffer = (void*)data;
    }
    spi_mutex_lock();
    spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);
    spi_mutex_unlock();
}

//Read SPI status(Slave send to master data length) transmit function , address length is 0 bit(no address)
static spi_recv_opt_t IRAM_ATTR spi_master_get_slave_status()
{
    spi_recv_opt_t recv_opt;
    spi_transaction_ext_t trans = (spi_transaction_ext_t) {
        .base = {
            .flags = SPI_TRANS_VARIABLE_ADDR,
            .rxlength = 4 * 8,
            .cmd = SPI_MASTER_READ_STATUS_FROM_SLAVE_CMD,
            .rx_buffer = &recv_opt
        },
        .address_bits = 0,
    };
    spi_mutex_lock();
    spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);
    spi_mutex_unlock();
    return recv_opt;
}

// SPI status transmicurrent_send_seqt function(Master sendto slave data length) , address length is 0 bit(no address)
static void IRAM_ATTR spi_master_set_trans_info(uint8_t send_seq, uint16_t send_len)
{
    spi_send_opt_t send_opt;
    send_opt.magic = 0xFE;
    send_opt.send_seq = send_seq;
    send_opt.send_len = send_len;
    spi_transaction_ext_t trans = (spi_transaction_ext_t) {
        .base = {
            .flags = SPI_TRANS_VARIABLE_ADDR,
            .length = 4 * 8,
            .cmd = SPI_MASTER_WRITE_STATUS_TO_SLAVE_CMD,
            .tx_buffer = &send_opt
        },
        .address_bits = 0,
    };

    spi_device_polling_transmit(handle, (spi_transaction_t*)&trans);
    // increment
    current_send_seq  = send_seq;
    tmp_send_len = send_len;
    mcu_send_flag = true;
}

// SPI master sent to slave function
static int8_t IRAM_ATTR spi_load_data(uint8_t* buf, int32_t len)
{

    if (len > RING_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Send length %d is too large", len);
        return -1;
    }

    xRingbufferSend(at_spi_master_send_ring_buf, (void*)buf, len, portMAX_DELAY);

    spi_mutex_lock();
    ring_buffer_filled_byte += len;

    if (allow_write_status && transmit_done_flag) {
        spi_master_set_trans_info(current_send_seq + 1, len);
        allow_write_status = false;
    }

    spi_mutex_unlock();

    return 0;
}


static int32_t IRAM_ATTR esp32_spi_write(host_serial_bus_handle_t dev, const void* data, size_t size)
{
    int8_t ret = 0;
    int32_t length = size;

    if (data == NULL  || length >= RING_BUFFER_SIZE) {
        ESP_LOGE(TAG, "Write data error, len:%d", length);
        return -1;
    }

    ret = spi_load_data((uint8_t*)data, size);

    if (ret < 0) {
        return -1;
    }

    return size;
}

void init_driver(void)
{
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .dummy_bits = 0,
        .clock_speed_hz = 15000000,    // 15M SPI clock
        .duty_cycle_pos = 128,      //50% duty cycle
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .cs_ena_posttrans = 1,      //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size = 3,
        .flags = SPI_DEVICE_HALFDUPLEX
    };

    //GPIO config for the handshake line.
    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pin_bit_mask = (1 << GPIO_HANDSHAKE)
    };

    //Create the semaphore.
    rdySem = xSemaphoreCreateBinary();
    pxMutex = xSemaphoreCreateMutex();

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_PIN_INTR_POSEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    assert(ret == ESP_OK);

}

/* The actual SPI transfer function
*  In order to improve the transfer efficiency, use two ring buffer to buffer the received or sent data
*/
static void IRAM_ATTR spi_master_transmit(void* arg)
{
    host_serial_bus_handle_t dev = (host_serial_bus_handle_t)arg;
    uint32_t transmit_data[16];
    spi_recv_opt_t recv_opt;
    size_t recv_actual_len = 0;
    uint8_t* transmit_point = NULL;
    uint8_t* recv_data = NULL;
    uint32_t transmit_len = 0;
    size_t recv_len = 0;
    uint32_t tmp_recv_len = 0;
    uint32_t read_len = 0;
    spi_mode_t current_transmit_direct = SPI_NULL;
    while(1) {
        /* If the previous SPI transmission does not produce an interrupt in ESP8266(ESP8266 will pulldown HANDSHAKE line when produce interrupt), 
         * start a new SPI transmission, the new SPI transmission maybe not trigger corresponding interrupt.
         * We need to avoid this situation. We assume that if we have already written the WR status register or read the RD status register,
         * ESP8266 must pullup the Handshake line at 30ms (Normal should be a microsecond time trigger interrupt), otherwise we think this interrupt did not trigger, we need to reread the RD status register.
        */
        if (transmit_done_flag == false || mcu_send_flag) {
            if (xSemaphoreTake(rdySem, 30 / portTICK_PERIOD_MS) == pdFALSE) {
                ESP_LOGI(TAG, "Find timeout, %d, %d", transmit_done_flag, mcu_send_flag);

                // First check to see if the RD status hs been read
                if (transmit_done_flag == false) {
                    // Just reread RD status
                    transmit_done_flag = true;
                } else if (mcu_send_flag) {
                    // Write WR status no intrrupt, rewrite it
                    ESP_LOGD(TAG, "MCU send timeout, resend again");
                    spi_master_set_trans_info(current_send_seq, tmp_send_len);
                    mcu_send_flag = false;
                    continue;
                }
            }
        } else {
            xSemaphoreTake(rdySem, (portTickType)portMAX_DELAY);
        }
        
        if (transmit_done_flag) {
            recv_opt = spi_master_get_slave_status();
            ESP_LOGD(TAG, "Recv seq: %d, direct: %d, len: %d", recv_opt.seq_num, recv_opt.direct, recv_opt.transmit_len);
            if (recv_opt.direct == SPI_WRITE) {  // master -> slave
                if (recv_opt.seq_num != current_send_seq) {
                    ESP_LOGE(TAG, "SPI send seq error, %x, %x", recv_opt.seq_num, current_send_seq);
                    break;
                }

                if (recv_opt.transmit_len > ring_buffer_filled_byte || recv_opt.transmit_len == 0) {
                    ESP_LOGE(TAG, "SPI send len error, %x, %x", recv_opt.transmit_len, ring_buffer_filled_byte);
                    break;
                }
                current_transmit_direct = SPI_WRITE;
            } else if (recv_opt.direct == SPI_READ) {
                if (recv_opt.seq_num != ((current_recv_seq + 1) & 0xFF) && recv_opt.seq_num != (current_recv_seq & 0xFF)) {
                    ESP_LOGE(TAG, "SPI recv seq error, %x, %x", recv_opt.seq_num, (current_recv_seq + 1));
                    break;
                }

                if (recv_opt.transmit_len > RING_BUFFER_SIZE || recv_opt.transmit_len == 0) {
                    ESP_LOGE(TAG, "SPI read len error, %x, %x", recv_opt.transmit_len, RING_BUFFER_SIZE);
                    break;
                }
                allow_read_status = false;

                current_recv_seq = recv_opt.seq_num;
                current_transmit_direct = SPI_READ;
                total_send_len = recv_opt.transmit_len;
            } else {
                ESP_LOGE(TAG, "transmit direct error %x", recv_opt.direct);
                break;
            }
            transmit_len = recv_opt.transmit_len;
            transmit_done_flag =  false;
        } else {
            read_len =  transmit_len > 64 ? 64 : transmit_len;
            if (current_transmit_direct == SPI_WRITE) {
                mcu_send_flag = false;
                if (read_len > 0) {
                    transmit_point = (uint8_t*)xRingbufferReceiveUpTo(at_spi_master_send_ring_buf, &recv_actual_len, 0, read_len);

                    // When the data is at herd and tail of the buffer, ESP32 ring buffer cannot read it all at once, so we have to read it twice
                    if (read_len != recv_actual_len) {
                        /* If use other stream buffer , ignore it */
                        memcpy((uint8_t*)transmit_data, transmit_point, recv_actual_len);
                        vRingbufferReturnItem(at_spi_master_send_ring_buf, (void*)transmit_point);
                        uint32_t remain_len = read_len - recv_actual_len;
                        uint32_t current_pos = recv_actual_len;
                        transmit_point = (uint8_t*)xRingbufferReceiveUpTo(at_spi_master_send_ring_buf, &recv_actual_len, 0, remain_len);
                        assert(recv_actual_len == remain_len);
                        memcpy(((uint8_t*)transmit_data) + current_pos, transmit_point, recv_actual_len);
                    } else {
                        memcpy((uint8_t*)transmit_data, transmit_point, recv_actual_len);
                    }
                    
                    spi_master_tran_data(SPI_WRITE, transmit_data, read_len);
                    vRingbufferReturnItem(at_spi_master_send_ring_buf, (void*)transmit_point);
                    transmit_len -= read_len;
                    ring_buffer_filled_byte -= read_len;
                    if (transmit_len == 0) {
                        spi_mutex_lock();
                        if (ring_buffer_filled_byte > 0) {
                            //gpio_set_level(TEST_GPIO, 1);
                            spi_master_set_trans_info(current_send_seq + 1, ring_buffer_filled_byte);
                            //gpio_set_level(TEST_GPIO, 0);
                            allow_write_status = false;
                        } else {
                            allow_write_status = true;
                        }
                        transmit_done_flag = true;
                        current_transmit_direct = SPI_NULL;
                        spi_mutex_unlock();
                    }

                } else {
                    ESP_LOGE(TAG, "transmit len error %x", transmit_len);
                    break;
                }
            } else if (current_transmit_direct == SPI_READ) {
                spi_master_tran_data(SPI_READ, transmit_data, read_len);
                xRingbufferSend(at_spi_master_receive_ring_buf, (void*)transmit_data, read_len, portMAX_DELAY);
                transmit_len -= read_len;

                if (transmit_len == 0) {
                    recv_data = (uint8_t*)xRingbufferReceiveUpTo(at_spi_master_receive_ring_buf, &recv_len, 0, total_send_len);

                    // When the data is at head and tail of the buffer, ESP32 ring buffer cannot read it all at once, so we have to read it twice
                    if (recv_len != total_send_len) {
                        uint8_t* tmp_recv_data= (uint8_t*)malloc(total_send_len);
                        if (tmp_recv_data == NULL) {
                            ESP_LOGE(TAG, "malloc fail");
                            break;
                        }
                        memcpy(tmp_recv_data, recv_data, recv_len);
                        tmp_recv_len = recv_len;

                        vRingbufferReturnItem(at_spi_master_receive_ring_buf, (void*)recv_data);
                        total_send_len -= recv_len;

                        recv_data = (uint8_t*)xRingbufferReceiveUpTo(at_spi_master_receive_ring_buf, &recv_len, 0, total_send_len);
                        if (total_send_len != recv_len) {
                            ESP_LOGE(TAG, "Receive data error, %d,%d", recv_len, total_send_len);
                        }
                        memcpy(tmp_recv_data + tmp_recv_len, recv_data, recv_len);
                        if (dev->ops->event_cb) {
                            dev->ops->event_cb(dev, BUFF_FULL, tmp_recv_data, (tmp_recv_len +  recv_len));
                        }
                        free(tmp_recv_data);
                        tmp_recv_data = NULL;
                    } else {
                        if (dev->ops->event_cb) {
                            dev->ops->event_cb(dev, BUFF_FULL, recv_data, recv_len);
                        }
                    }

                    vRingbufferReturnItem(at_spi_master_receive_ring_buf, (void*)recv_data);
                    total_send_len -= recv_len;

                    spi_mutex_lock();
                    if (ring_buffer_filled_byte > 0 && allow_write_status) {
                        spi_master_set_trans_info(current_send_seq + 1, ring_buffer_filled_byte);
                        allow_write_status = false;
                    }
                    transmit_done_flag = true;
                    current_transmit_direct = SPI_NULL;
                    spi_mutex_unlock();

                }
            }
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t  esp32_spi_init(host_serial_bus_handle_t dev)
{
    at_spi_master_receive_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    at_spi_master_send_ring_buf = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);

    init_driver();
    xTaskCreate(spi_master_transmit, "spi_master_transmit", 1024 * 2, dev, 8, NULL);
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
