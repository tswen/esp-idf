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

/** Includes **/
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "trace.h"
#include "sdspi_drv.h"
#include "adapter.h"
#include "serial_drv.h"
#include "netdev_if.h"

#include "host_serial_bus.h"
#include "host_config.h"

/** Constants/Macros **/
#define TO_SLAVE_QUEUE_SIZE               10
#define FROM_SLAVE_QUEUE_SIZE             10

#define PROCESS_RX_TASK_STACK_SIZE        4096

#define READ_BUFFER_LEN     4096

#define ESP_PAYLOAD_HEADER_SIZE    8
#define MAX_PAYLOAD_SIZE (MAX_SPI_BUFFER_SIZE-sizeof(struct esp_payload_header))

static const char *TAG = "sdio_drv";

static host_serial_bus_handle_t s_host_device_handle;
static esp_err_t IRAM_ATTR recv_cb(host_serial_bus_handle_t device, host_recv_status_t recv_status, const void *buffer, size_t len);

static int esp_netdev_open(netdev_handle_t netdev);
static int esp_netdev_close(netdev_handle_t netdev);
static int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf);


#ifdef CONFIG_BT_A2DP_SINK_HCI
extern int sdio_recv_data_from_controller(uint8_t *data, uint16_t len);
#endif /* CONFIG_BT_A2DP_SINK_HCI */

static struct esp_private *esp_priv[MAX_NETWORK_INTERFACES];

static struct netdev_ops esp_net_ops = {
	.netdev_open = esp_netdev_open,
	.netdev_close = esp_netdev_close,
	.netdev_xmit = esp_netdev_xmit,
};

/** Exported variables **/
extern struct host_device_handle *esp_hosted_driver_handle;

// static osMutexId mutex_trans;
static SemaphoreHandle_t xSemaphore = NULL;
static SemaphoreHandle_t mutex_trans = NULL;

static TaskHandle_t Process_RX_Task_Handle = NULL;

/* Queue declaration */
static QueueHandle_t to_slave_queue = NULL;
static QueueHandle_t from_slave_queue = NULL;

/* callback of event handler */
static void (*sdspi_drv_evt_handler_fp) (uint8_t);

/** function declaration **/
/** Exported functions **/
static void process_rx_task(void* pvParameters);
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf);
static void deinit_netdev(void);

/**
  * @brief  get private interface of expected type and number
  * @param  if_type - interface type
  *         if_num - interface number
  * @retval interface handle if found, else NULL
  */
static struct esp_private * get_priv(uint8_t if_type, uint8_t if_num)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if((esp_priv[i]) &&
			(esp_priv[i]->if_type == if_type) &&
			(esp_priv[i]->if_num == if_num))
			return esp_priv[i];
	}

	return NULL;
}

/**
  * @brief  open virtual network device
  * @param  netdev - network device
  * @retval 0 on success
  */
static int esp_netdev_open(netdev_handle_t netdev)
{
	return ESP_OK;
}

/**
  * @brief  close virtual network device
  * @param  netdev - network device
  * @retval 0 on success
  */
static int esp_netdev_close(netdev_handle_t netdev)
{
	return ESP_OK;
}

/**
  * @brief  transmit on virtual network device
  * @param  netdev - network device
  *         net_buf - buffer to transmit
  * @retval None
  */
static int esp_netdev_xmit(netdev_handle_t netdev, struct pbuf *net_buf)
{
	struct esp_private *priv;
	int ret;

	if (!netdev || !net_buf)
		return ESP_FAIL;
	priv = (struct esp_private *) netdev_get_priv(netdev);

	if (!priv)
		return ESP_FAIL;

	ret = send_to_slave(priv->if_type, priv->if_num,
			net_buf->payload, net_buf->len);
	free(net_buf);

	return ret;
	return 0;
}

/**
  * @brief  create virtual network device
  * @param  None
  * @retval None
  */
static int init_netdev(void)
{
	void *ndev = NULL;
	int i = 0;
	struct esp_private *priv = NULL;
	char *if_name = STA_INTERFACE;
	uint8_t if_type = ESP_STA_IF;

	for (i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		/* Alloc and init netdev */
		ndev = netdev_alloc(sizeof(struct esp_private), if_name);
		if (!ndev) {
			deinit_netdev();
			return ESP_FAIL;
		}

		priv = (struct esp_private *) netdev_get_priv(ndev);
		if (!priv) {
			deinit_netdev();
			return ESP_FAIL;
		}

		priv->netdev = ndev;
		priv->if_type = if_type;
		priv->if_num = 0;

		if (netdev_register(ndev, &esp_net_ops)) {
			deinit_netdev();
			return ESP_FAIL;
		}

		if_name = SOFTAP_INTERFACE;
		if_type = ESP_AP_IF;

		esp_priv[i] = priv;
	}

	return ESP_OK;
}

/**
  * @brief  destroy virtual network device
  * @param  None
  * @retval None
  */
static void deinit_netdev(void)
{
	for (int i = 0; i < MAX_NETWORK_INTERFACES; i++) {
		if (esp_priv[i]) {
			if (esp_priv[i]->netdev) {
				netdev_unregister(esp_priv[i]->netdev);
				netdev_free(esp_priv[i]->netdev);
			}
			esp_priv[i] = NULL;
		}
	}
}


/** function definition **/

/** Exported Functions **/
/**
  * @brief  spi driver initialize
  * @param  spi_drv_evt_handler - event handler of type spi_drv_events_e
  * @retval None
  */
esp_err_t esp_sdspi_init(void(*spi_drv_evt_handler)(uint8_t))
{
	esp_err_t retval = ESP_OK;

	s_host_device_handle = host_serial_bus_open(DEVICE_SDIO);
	if (s_host_device_handle == NULL) {
		ESP_LOGE(TAG, "open device error");
		return ESP_FAIL;
	}
	host_serial_bus_register_recv_callback(s_host_device_handle, recv_cb);

	/* register callback */
	sdspi_drv_evt_handler_fp = spi_drv_evt_handler;

	retval = init_netdev();
	if (retval) {
		printf("netdev failed to init\n\r");
		assert(retval==ESP_OK);
	}

	/* Queue - tx */
	to_slave_queue = xQueueCreate(TO_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(to_slave_queue);

	/* Queue - rx */
	from_slave_queue = xQueueCreate(FROM_SLAVE_QUEUE_SIZE,
			sizeof(interface_buffer_handle_t));
	assert(from_slave_queue);

	/* Task - RX processing */
	xTaskCreate(process_rx_task, "Process_RX_Task", PROCESS_RX_TASK_STACK_SIZE, NULL, 12, &Process_RX_Task_Handle);
	configASSERT(Process_RX_Task_Handle);

	return ESP_OK;
}


static void check_and_execute_spi_transaction(uint16_t wlen)
{
	uint8_t * txbuff = NULL;
	uint8_t is_valid_tx_buf = 0;

	/* Get next tx buffer to be sent */
	txbuff = get_tx_buffer(&is_valid_tx_buf);
	// ESP_LOG_BUFFER_HEXDUMP(TAG, txbuff, wlen + ESP_PAYLOAD_HEADER_SIZE, ESP_LOG_INFO);

	host_serial_bus_write(s_host_device_handle, txbuff, wlen + ESP_PAYLOAD_HEADER_SIZE);

	free(txbuff);
}

#ifdef CONFIG_BT_A2DP_SINK_HCI
int sdio_send_data_to_controller(uint8_t *data, uint16_t len){

	uint8_t * send_buff = (uint8_t* )malloc(len);
	memcpy(send_buff, data, len);
	int ret = send_to_slave(ESP_HCI_IF, 0, send_buff, len);

    return ret;
}
#endif /* CONFIG_BT_A2DP_SINK_HCI */

/**
  * @brief  Send to slave via SPI
  * @param  iface_type -type of interface
  *         iface_num - interface number
  *         wbuffer - tx buffer
  *         wlen - size of wbuffer
  * @retval sendbuf - Tx buffer
  */
esp_err_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen)
{
	interface_buffer_handle_t buf_handle = {0};

	if (!wbuffer || !wlen || (wlen > MAX_PAYLOAD_SIZE)) {
		printf("write fail: buff(%p) 0? OR (0<len(%u)<=max_poss_len(%u))?\n\r",
				wbuffer, wlen, MAX_PAYLOAD_SIZE);
		if(wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return ESP_FAIL;
	}
	memset(&buf_handle, 0, sizeof(buf_handle));

	buf_handle.if_type = iface_type;
	buf_handle.if_num = iface_num;
	buf_handle.payload_len = wlen;
	buf_handle.payload = wbuffer;
	buf_handle.priv_buffer_handle = wbuffer;
	buf_handle.free_buf_handle = free;

	if (pdTRUE != xQueueSend(to_slave_queue, &buf_handle, portMAX_DELAY)) {
		printf("Failed to send buffer to_slave_queue\n\r");
		if(wbuffer) {
			free(wbuffer);
			wbuffer = NULL;
		}
		return ESP_FAIL;
	}

	check_and_execute_spi_transaction(wlen);

	return ESP_OK;
}

static esp_err_t IRAM_ATTR recv_cb(host_serial_bus_handle_t device, host_recv_status_t recv_status, const void *buffer, size_t len)
{
    if (!buffer || !len) {
        return ESP_FAIL;
    }
	ESP_LOGD(TAG, "Host recv len %d", len);


	interface_buffer_handle_t buf_handle = {0};
	struct  esp_payload_header *payload_header;
	uint16_t payload_len, offset;

	uint8_t *recv_data = malloc((len + 1) * sizeof(uint8_t));
	assert(recv_data);
	memcpy(recv_data, buffer, len);

	/* create buffer rx handle, used for processing */
	payload_header = (struct esp_payload_header *) recv_data;

	/* Fetch length and offset from payload header */
	payload_len = le16toh(payload_header->len);
	offset = le16toh(payload_header->offset);

	buf_handle.priv_buffer_handle = recv_data;
	buf_handle.free_buf_handle = free;
	buf_handle.payload_len = payload_len;
	buf_handle.if_type     = payload_header->if_type;
	buf_handle.if_num      = payload_header->if_num;
	buf_handle.payload     = recv_data + offset;
	if (pdTRUE != xQueueSend(from_slave_queue,
				&buf_handle, 5000 / portTICK_PERIOD_MS)) {
		printf("Failed to send buffer\n\r");
	}
}

/** Local functions **/

/**
  * @brief  RX processing task
  * @param  argument: Not used
  * @retval None
  */
static void process_rx_task(void* pvParameters)
{
	esp_err_t ret = ESP_OK;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t *payload = NULL;
	struct pbuf *buffer = NULL;
	struct esp_priv_event *event = NULL;
	struct esp_private *priv = NULL;
	uint8_t *serial_buf = NULL;
	uint8_t * send_to_host = NULL;

	while (1) {
		ret = xQueueReceive(from_slave_queue, &buf_handle, portMAX_DELAY);
		if (ret != pdTRUE) {
			continue;
		}

		/* point to payload */
		payload = buf_handle.payload;

		/* process received buffer for all possible interface types */
		if (buf_handle.if_type == ESP_SERIAL_IF) {

			serial_buf = (uint8_t *)malloc(buf_handle.payload_len);
			assert(serial_buf);

			memcpy(serial_buf, payload, buf_handle.payload_len);

			/* serial interface path */
			serial_rx_handler(buf_handle.if_num, serial_buf,
					buf_handle.payload_len);

		} else if((buf_handle.if_type == ESP_STA_IF) ||
				(buf_handle.if_type == ESP_AP_IF)) {
			priv = get_priv(buf_handle.if_type, buf_handle.if_num);

			if (priv) {
				buffer = (struct pbuf *)malloc(sizeof(struct pbuf));
				assert(buffer);

				buffer->len = buf_handle.payload_len;
				buffer->payload = malloc(buf_handle.payload_len);
				assert(buffer->payload);

				memcpy(buffer->payload, buf_handle.payload,
						buf_handle.payload_len);
				
				netdev_rx(priv->netdev, buffer);
			}

		} else if (buf_handle.if_type == ESP_PRIV_IF) {
			/* priv transaction received */

			event = (struct esp_priv_event *) (payload);
			if (event->event_type == ESP_PRIV_EVENT_INIT) {
				/* halt spi transactions for some time,
				 * this is one time delay, to give breathing
				 * time to slave before spi trans start */
				vTaskDelay(50000);
				if (sdspi_drv_evt_handler_fp) {
					sdspi_drv_evt_handler_fp(SDSPI_DRIVER_ACTIVE);
				}
			} else {
				/* User can re-use this type of transaction */
			}
		}
#ifdef CONFIG_BT_A2DP_SINK_HCI
		  else if (buf_handle.if_type == ESP_HCI_IF) {
			//esp_log_buffer_hex("BT_RX", payload, buf_handle.payload_len);
			send_to_host = (uint8_t *)malloc(buf_handle.payload_len);
			memcpy(send_to_host, buf_handle.payload, buf_handle.payload_len);
			sdio_recv_data_from_controller(send_to_host, buf_handle.payload_len);
		}
#endif /* CONFIG_BT_A2DP_SINK_HCI */

		/* Free buffer handle */
		/* When buffer offloaded to other module, that module is
		 * responsible for freeing buffer. In case not offloaded or
		 * failed to offload, buffer should be freed here.
		 */
/****************************************************/ //debug 
		if (buf_handle.free_buf_handle) {
			buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
		}
/****************************************************/
	}
}


/**
  * @brief  Next TX buffer in SPI transaction
  * @param  argument: Not used
  * @retval sendbuf - Tx buffer
  */
static uint8_t * get_tx_buffer(uint8_t *is_valid_tx_buf)
{
	struct  esp_payload_header *payload_header;
	uint8_t *sendbuf = NULL;
	uint8_t *payload = NULL;
	uint16_t len = 0;
	interface_buffer_handle_t buf_handle = {0};

	*is_valid_tx_buf = 0;

	/* Check if higher layers have anything to transmit, non blocking.
	 * If nothing is expected to send, queue receive will fail.
	 * In that case only payload header with zero payload
	 * length would be transmitted.
	 */
	if (pdTRUE == xQueueReceive(to_slave_queue, &buf_handle, 0)) {
		len = buf_handle.payload_len;
	}

	if (len) {

		sendbuf = (uint8_t *) malloc(MAX_SPI_BUFFER_SIZE);
		if (!sendbuf) {
			printf("malloc failed\n\r");
			goto done;
		}

		memset(sendbuf, 0, MAX_SPI_BUFFER_SIZE);

		*is_valid_tx_buf = 1;

		/* Form Tx header */
		payload_header = (struct esp_payload_header *) sendbuf;
		payload = sendbuf + sizeof(struct esp_payload_header);
		payload_header->len     = htole16(len);
		payload_header->offset  = htole16(sizeof(struct esp_payload_header));
		payload_header->if_type = buf_handle.if_type;
		payload_header->if_num  = buf_handle.if_num;
		payload_header->reserved1 = 0;

		memcpy(payload, buf_handle.payload, min(len, MAX_PAYLOAD_SIZE));
	}

done:
	/* free allocated buffer */
	if (buf_handle.free_buf_handle)
		buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);

	return sendbuf;
}
