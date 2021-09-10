/*
 * FreeRTOS V202107.00
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/**
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 * 
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_netif.h"
#include "esp_netif_types.h"

#include "FreeRTOS_CLI.h"

#include "iperf.h"
#include "control.h"

#ifdef CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

#define cliNEW_LINE    "\r\n"

char* null_password = "";

extern esp_netif_t *netif_sta;
static esp_netif_ip_info_t ip;

#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
static BaseType_t prvTaskStatusCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
#endif

#ifdef CONFIG_HEAP_TRACING
static BaseType_t prvHeapTracingCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
#endif

static BaseType_t prvStationCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvScanCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvAPCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvSetWiFiModeCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvRamCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvRestartCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvGetVersionCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

static BaseType_t prvIperfCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
/* Structure that defines the "task-stats" command line command. */
static const CLI_Command_Definition_t xTaskStatus =
{
	"task-status", /* The command string to type. */
	"task-status: Displays the state of each task\r\n",
	prvTaskStatusCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
#endif

#ifdef CONFIG_HEAP_TRACING
/* Structure that defines the "heaptracing" command line command. */
static const CLI_Command_Definition_t xHeapTracingStatus =
{
	"heaptracing", /* The command string to type. */
	"heaptracing: \r\n",
	prvHeapTracingCommand, /* The function to run. */
	0 /* No parameters are expected. */
};
#endif

/* Structure that defines the "sta" command line command. */
static const CLI_Command_Definition_t xStationCommand =
{
	"sta", /* The command string to type. */
	"sta: join specified soft-AP\r\n",
	prvStationCommand, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

/* Structure that defines the "scan" command line command. */
static const CLI_Command_Definition_t xScanCommand =
{
	"scan", /* The command string to type. */
	"scan: Scan the AP Information\r\n",
	prvScanCommand, /* The function to run. */
	0 /* The user can enter any number of commands. */
};

/* Structure that defines the "ap" command line command. */
static const CLI_Command_Definition_t xAPCommand =
{
	"ap", /* The command string to type. */
	"ap: configure ssid and password\r\n",
	prvAPCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "mode" command line command. */
static const CLI_Command_Definition_t xSetWiFiModeCommand =
{
	"mode", /* The command string to type. */
	"mode <mode>: <sta> station mode; <ap> ap mode\r\n",
	prvSetWiFiModeCommand, /* The function to run. */
	1 /* one parameters are expected. */
};

/* Structure that defines the "ram" command line command. */
static const CLI_Command_Definition_t xRamCommand =
{
	"ram", /* The command string to type. */
	"ram: Get the current size of free heap memory and minimum size of free heap memory\r\n",
	prvRamCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "restart" command line command. */
static const CLI_Command_Definition_t xRestartCommand =
{
	"restart", /* The command string to type. */
	"restart: Software reset of the chip\r\n",
	prvRestartCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "version" command line command. */
static const CLI_Command_Definition_t xGetVersionCommand =
{
	"version", /* The command string to type. */
	"version: Get version of chip and SDK\r\n",
	prvGetVersionCommand, /* The function to run. */
	0 /* No parameters are expected. */
};

/* Structure that defines the "iperf" command line command. */
static const CLI_Command_Definition_t xIperfCommand =
{
	"iperf", /* The command string to type. */
	"iperf: throughput test\r\n",
	prvIperfCommand, /* The function to run. */
	-1 /* The user can enter any number of commands.*/
};

/*-----------------------------------------------------------*/

void vRegisterCLICommands( void )
{
    FreeRTOS_CLICreatMux();

	/* Register all the command line commands defined immediately above. */
#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
	FreeRTOS_CLIRegisterCommand( &xTaskStatus );
#endif
#ifdef CONFIG_HEAP_TRACING
	FreeRTOS_CLIRegisterCommand( &xHeapTracingStatus );
#endif
	FreeRTOS_CLIRegisterCommand( &xAPCommand );
	FreeRTOS_CLIRegisterCommand( &xStationCommand );
	FreeRTOS_CLIRegisterCommand( &xSetWiFiModeCommand );
	FreeRTOS_CLIRegisterCommand( &xScanCommand );
	FreeRTOS_CLIRegisterCommand( &xRamCommand );
	FreeRTOS_CLIRegisterCommand( &xRestartCommand );
	FreeRTOS_CLIRegisterCommand( &xGetVersionCommand );
	FreeRTOS_CLIRegisterCommand( &xIperfCommand );
}
/*-----------------------------------------------------------*/

static BaseType_t prvIperfCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	char *pc1, *pc2;
	BaseType_t xLength1, xLength2;
	iperf_cfg_t cfg_iperf_test;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );
	memset( &cfg_iperf_test, 0, sizeof(cfg_iperf_test) );

#ifdef CONFIG_HEAP_TRACING
	heap_trace_start(HEAP_TRACE_LEAKS);
#endif

	pc2 = ( char * ) FreeRTOS_CLIGetParameter(pcCommandString, 2, &xLength2);
	pc1 = ( char * ) FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLength1);

	if (pc1 == NULL) {
		sprintf(pcWriteBuffer, "Invalid command\r\n");
		return pdFALSE;
	} else {
		/* Sanity check something was returned. */
		configASSERT( pc1 );
		/* Terminate the string. */
		pc1[ xLength1 ] = 0x00;

		esp_netif_get_ip_info(netif_sta, &ip);

		/* iperf -a */
		if(strncmp(pc1, "-a", strlen("-a")) == 0) {
			printf("iperf stop\r\n");
			iperf_stop();
			return pdFALSE;
		}
		/* iperf -s */
		else if(strncmp(pc1, "-s", strlen("-s")) == 0) {
			cfg_iperf_test.flag |= IPERF_FLAG_SERVER;
			cfg_iperf_test.sip = ip.ip.addr;
			cfg_iperf_test.sport = IPERF_DEFAULT_PORT;
			cfg_iperf_test.dport = IPERF_DEFAULT_PORT;
			cfg_iperf_test.interval = IPERF_DEFAULT_INTERVAL;
			cfg_iperf_test.time = IPERF_DEFAULT_TIME;
		}
		/* iperf -c */
		else if(strncmp(pc1, "-c", strlen("-c")) == 0) {
			if (pc2 == NULL) {
				sprintf(pcWriteBuffer, "Missing destination IP address\r\n");
				return pdFALSE;
			} else {
				cfg_iperf_test.dip = esp_ip4addr_aton(pc2);
				cfg_iperf_test.flag |= IPERF_FLAG_CLIENT;
				cfg_iperf_test.sip = ip.ip.addr;
				cfg_iperf_test.flag |= IPERF_FLAG_TCP;
				cfg_iperf_test.sport = IPERF_DEFAULT_PORT;
				cfg_iperf_test.dport = IPERF_DEFAULT_PORT;
				cfg_iperf_test.interval = IPERF_DEFAULT_INTERVAL;
				cfg_iperf_test.time = IPERF_DEFAULT_TIME;
			}
		} else {
			sprintf(pcWriteBuffer, "Invalid command\r\n");
			return pdFALSE;
		}
	}

	printf("mode=%s-%s sip=%d.%d.%d.%d:%d, dip=%d.%d.%d.%d:%d, interval=%d, time=%d\r\n",
			cfg_iperf_test.flag & IPERF_FLAG_TCP ? "tcp" : "udp",
			cfg_iperf_test.flag & IPERF_FLAG_SERVER ? "server" : "client",
			cfg_iperf_test.sip & 0xFF, (cfg_iperf_test.sip >> 8) & 0xFF, (cfg_iperf_test.sip >> 16) & 0xFF, (cfg_iperf_test.sip >> 24) & 0xFF, cfg_iperf_test.sport,
			cfg_iperf_test.dip & 0xFF, (cfg_iperf_test.dip >> 8) & 0xFF, (cfg_iperf_test.dip >> 16) & 0xFF, (cfg_iperf_test.dip >> 24) & 0xFF, cfg_iperf_test.dport,
			cfg_iperf_test.interval, cfg_iperf_test.time);
	iperf_start(&cfg_iperf_test);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

#ifdef CONFIG_HEAP_TRACING
static BaseType_t prvHeapTracingCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

    ESP_ERROR_CHECK( heap_trace_stop() );
    heap_trace_dump();

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
#endif
/*-----------------------------------------------------------*/

#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
static BaseType_t prvTaskStatusCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "Task          State  Priority  Stack	#\r\n************************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	char *data = (char *)malloc(512);
	/* Generate a table of task stats. */
	strcpy( data, pcHeader );
	vTaskList( data + strlen( pcHeader ) );
	printf("%s", data);
	free(data);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
#endif
/*-----------------------------------------------------------*/

static BaseType_t prvStationCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	char *pc1, *pc2, *pc3, *pc4, *pc5, *pc6;
	BaseType_t xLength1, xLength2, xLength3, xLength4, xLength5, xLength6;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	/* Obtain the sixth parameter */
	pc6 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									6,                  /* Return the Sixth parameter. */
									&xLength6           /* Store the parameter string length. */
								);

	/* Obtain the fifth parameter */
	pc5 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									5,                  /* Return the fifth parameter. */
									&xLength5           /* Store the parameter string length. */
								);

	/* Obtain the fourth parameter */
	pc4 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									4,                  /* Return the fourth parameter. */
									&xLength4           /* Store the parameter string length. */
								);

	/* Obtain the third parameter. */
	pc3 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									3,                  /* Return the third parameter. */
									&xLength3           /* Store the parameter string length. */
								);

	/* Obtain the second parameter */
	pc2 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									2,                  /* Return the second parameter. */
									&xLength2           /* Store the parameter string length. */
								);

	/* Obtain the first parameter. */
	pc1 = ( char * ) FreeRTOS_CLIGetParameter
								(
									pcCommandString,    /* The command string itself. */
									1,                  /* Return the first parameter. */
									&xLength1           /* Store the parameter string length. */
								);

	if (pc1 == NULL) {
		get_application_mode();
		return pdFALSE;
	} else {
		/* Sanity check something was returned. */
		configASSERT( pc1 );
		/* Terminate the string. */
		pc1[ xLength1 ] = 0x00;

		if(strncmp(pc1, "-d", strlen("-d")) == 0) {
			// if (wif_cmd_disconnect_wifi() == ESP_OK) {
			// 	sprintf(pcWriteBuffer, "OK\r\n");
			// } else {
			// 	sprintf(pcWriteBuffer, "FAIL\r\n");
			// }
			return pdFALSE;
		} else if(strncmp(pc1, "-s", strlen("-s")) != 0) {
			sprintf(pcWriteBuffer, "Invalid parameter\r\n");
			return pdFALSE;
		}
	}

	if (pc2 == NULL) {
		sprintf(pcWriteBuffer, "Invalid parameter\r\n");
		return pdFALSE;
	} else {
		if (pc3 == NULL) {
			station_connect(pc2, null_password);
			printf("the ssid is %s.\r\n", pc2);
			printf("the ssid len is %d.\r\n", xLength2);
			return pdFALSE;
		}
	}

	if (pc4 == NULL) {
		station_connect(pc2, null_password);
		printf("the ssid is %s.\r\n", pc2);
		printf("the ssid len is %d.\r\n", (xLength2 + xLength3 + 1));
		return pdFALSE;
	}

	if (pc5 == NULL) {
		/* Terminate the string. */
		pc2[ xLength2 ] = 0x00;

		if(strncmp(pc3, "-p", strlen("-p")) != 0) {
			sprintf(pcWriteBuffer, "Invalid parameter\r\n");
			return pdFALSE;
		}
		/* Terminate the string. */
		pc3[ xLength3 ] = 0x00;
		station_connect(pc2, pc4);
		printf("the ssid is %s, the password is %s.\r\n", pc2, pc4);
		printf("the ssid len is %d, the password len is %d.\r\n", xLength2, xLength4);
		return pdFALSE;
	}

	if (pc6 == NULL) {
		if(strncmp(pc3, "-p", strlen("-p")) == 0) {    
			if(strncmp(pc4, "-p", strlen("-p")) != 0) {
				/* Terminate the string. */
				pc2[ xLength2 ] = 0x00;
				pc3[ xLength3 ] = 0x00;
				station_connect(pc2, pc4);
				printf("the ssid is %s, the password is %s.\r\n", pc2, pc4);
				printf("the ssid len is %d, the password len is %d.\r\n", xLength2, (xLength4 + xLength5 + 1));
				return pdFALSE;
			}
		}
		/* Terminate the string. */
		pc3[ xLength3 ] = 0x00;
		pc4[ xLength4 ] = 0x00;
		station_connect(pc2, pc5);
		printf("the ssid is %s, the password is %s.\r\n", pc2, pc5);
		printf("the ssid len is %d, the password len is %d.\r\n", (xLength2 + xLength3 + 1), xLength5);
	} else {
		/* Terminate the string. */
		pc3[ xLength3 ] = 0x00;
		pc4[ xLength4 ] = 0x00;
		station_connect(pc2, pc5);
		printf("the ssid is %s, the password is %s.\r\n", pc2, pc5);
		printf("the ssid len is %d, the password len is %d.\r\n", (xLength2 + xLength3 + 1), (xLength5 + xLength6 + 1));
	}

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvScanCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	get_ap_scan_list();

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvAPCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	softap_start();
	get_application_mode();

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvSetWiFiModeCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );


	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvRamCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	uint32_t heap_size = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
	uint32_t size = esp_get_free_heap_size();

	sprintf(pcWriteBuffer, "free heap size: %d, min heap size: %u\r\n", size, heap_size);

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvRestartCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );
	
    esp_restart();

	return pdFALSE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvGetVersionCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

    esp_chip_info_t info;
    esp_chip_info(&info);
	sprintf(pcWriteBuffer, "IDF Version:%s\r\nChip info:\r\n\tcores:%d\r\n\tfeature:%s%s%s%s%d%s\r\n\trevision number:%d\r\n", 
							esp_get_idf_version(),
							info.cores,
							info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
							info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
							info.features & CHIP_FEATURE_BT ? "/BT" : "",
							info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
							spi_flash_get_chip_size() / (1024 * 1024), " MB",
							info.revision);

	return pdFALSE;
}
/*-----------------------------------------------------------*/
