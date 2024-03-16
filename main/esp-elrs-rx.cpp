/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <cstring>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h" // Add this line to include the missing header

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define PATTERN_CHR_NUM    (3)  

static const char *TAG = "ESP_ELRS_CONTROLLER";

#define BUF_SIZE (70)
#define RD_BUF_SIZE (BUF_SIZE)

extern "C"
{
    void app_main();
}

QueueHandle_t uart_queue;


#define WIDTH (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

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

class Controller {
    public:
        Controller() { };
        void packageReceived(crsf_channels_t *channels) {
            printf("Received package\n");
            printf("Ch0: %d Ch1: %d Ch2: %d Ch3: %d Ch4: %d Ch5: %d Ch6: %d Ch7: %d Ch8: %d Ch9: %d Ch10: %d Ch11: %d Ch12: %d", channels->ch0, channels->ch1, channels->ch2, channels->ch3, channels->ch4, channels->ch5, channels->ch6, channels->ch7, channels->ch8, channels->ch9, channels->ch10, channels->ch11, channels->ch12);
        };
};
#define PackageMaxSize 64
uint8_t data[PackageMaxSize];
class UartELRS {
    private:
        bool reading = false;
        uint8_t crcTable[256];
        uint8_t BF_channels[PackageMaxSize];
        Controller *controller;
        crsf_channels_t *cr_channels = (crsf_channels_t *)BF_channels;

        uint8_t crcFast(uint8_t const message[], int nBytes) {
            uint8_t data;
            uint8_t remainder = 0;
            for (int byte = 0; byte < nBytes; ++byte)
            {
                data = message[byte] ^ (remainder >> (WIDTH - 8));
                remainder = crcTable[data] ^ (remainder << 8);
            }
            return (remainder);
        }

        void crcInit(void) {
            uint8_t remainder;
            for (int dividend = 0; dividend < 256; ++dividend) {
                remainder = dividend << (WIDTH - 8);
                for (uint8_t bit = 8; bit > 0; --bit)
                {
                    if (remainder & TOPBIT)
                    {
                        remainder = (remainder << 1) ^ 0xD5;
                    }
                    else
                    {
                        remainder = (remainder << 1);
                    }
                }
                crcTable[dividend] = remainder;
            }
        }
    public:
        uart_port_t portNum;

        UartELRS(uart_port_t portNum, Controller *controller) {
            this->controller = controller;
            this->portNum = portNum;

            uart_config_t uart_config = {
                .baud_rate = 400000,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh = BUF_SIZE,
            };
            const int uart_buffer_size = (BUF_SIZE);
            ESP_ERROR_CHECK(uart_driver_install(portNum, 1024, 1024, 10, &uart_queue, 0));
            ESP_ERROR_CHECK(uart_param_config(portNum, &uart_config));
            ESP_ERROR_CHECK(uart_set_pin(portNum, 4, 5, 18, 19));
            ESP_ERROR_CHECK(uart_enable_rx_intr(this->portNum));

            crcInit();
        };

        void read(uint16_t maxSize) {
            if(this->reading) {
                return;
            }
            this->reading = true;

            uint8_t currentByte;
            uart_read_bytes(this->portNum, &currentByte, 1, portMAX_DELAY);
            if (currentByte == 0xc8 || currentByte == 0xee || currentByte == 0xea || currentByte == 0xec) {
                uint8_t size;
                uart_read_bytes(this->portNum, &size, 1, portMAX_DELAY);
                if(size > maxSize - 1) {
                    size = (uint8_t)maxSize -1;
                }

                uart_read_bytes(this->portNum, data, size, portMAX_DELAY);
                uint8_t crc = crcFast(data, size-1);

                // if(crc == data[size-1]) {
                    // if (data[0] == 0x16) {
                        for (int ix = 1; ix < size; ix++)
                        BF_channels[ix-1] = data[ix];
                        
                        controller->packageReceived(cr_channels);
                //     }
                // }
            }
            this->reading = false;
        };
};


static void uart_event_task(void *args) {
    UartELRS *uartElrs = (UartELRS *) args;
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type) {
            case UART_DATA:
                {
                    uartElrs->read(event.size);
                    break;
                }
            case UART_FIFO_OVF:
                uart_flush_input(uartElrs->portNum);
                xQueueReset(uart_queue);
                break;
            case UART_BUFFER_FULL:
                uart_flush_input(uartElrs->portNum);
                xQueueReset(uart_queue);
                break;
            case UART_BREAK:
                break;
            case UART_PARITY_ERR:
                break;
            case UART_FRAME_ERR:
                break;
            case UART_PATTERN_DET: {
                uart_get_buffered_data_len(uartElrs->portNum, &buffered_size);
                int pos = uart_pattern_pop_pos(uartElrs->portNum);
                if (pos == -1) {
                    uart_flush_input(uartElrs->portNum);
                } else {
                    uart_read_bytes(uartElrs->portNum, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(uartElrs->portNum, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                }
                break;
            }
            default:
                // ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

        
    Controller *controller = new Controller();
    UartELRS *uart = new UartELRS(UART_NUM_2, controller);



    xTaskCreate(uart_event_task, "uart_event_task", 2048, uart, 12, NULL);
}
