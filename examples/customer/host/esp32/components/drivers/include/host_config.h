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
#ifndef __HOST_DRIVER_CONFIG_H__
#define __HOST_DRIVER_CONFIG_H__

#include <stdlib.h>
#include <stdio.h>
#include "esp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int at_debugLevel;

#define DRIVER_LOGE(x, ...) {if(at_debugLevel >= 0) {printf("E %s: ",x); printf(__VA_ARGS__); printf("\n");}}
#define DRIVER_LOGW(x, ...) {if(at_debugLevel >= 1) {printf("W %s: ",x); printf(__VA_ARGS__); printf("\n");}}
#define DRIVER_LOGI(x, ...) {if(at_debugLevel >= 2) {printf("I %s: ",x); printf(__VA_ARGS__); printf("\n");}}
#define DRIVER_LOGD(x, ...) {if(at_debugLevel >= 3) {printf("D %s: ",x); printf(__VA_ARGS__); printf("\n");}}
#define DRIVER_LOGV(x, ...) {if(at_debugLevel >= 4) {printf("V %s: ",x); printf(__VA_ARGS__); printf("\n");}}

#define esp_err_t esp_err_t

#ifdef __cplusplus
}
#endif

#endif /* __HOST_DRIVER_CONFIG_H__ */
