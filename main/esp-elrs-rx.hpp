#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <cstring>
#include "sdkconfig.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "./src/UartELRSParser/UartELRSParser.hpp"
#include "./src/PWMMotorsControl/PWMMotorsControl.hpp"
#include "./src/Controller/Controller.hpp"

#ifndef main_h
#define main_h

#define BUF_SIZE (100)
#define RD_BUF_SIZE (BUF_SIZE)



typedef struct crsf_channels_t {
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED;

#endif
