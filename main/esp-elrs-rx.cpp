/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <cstring>
#include <cmath>
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define PATTERN_CHR_NUM    (3)  

static const char *TAG = "ESP_ELRS_CONTROLLER";

#define BUF_SIZE (100)
#define RD_BUF_SIZE (BUF_SIZE)

extern "C"
{
    void app_main();
}

QueueHandle_t uart_queue;


#define WIDTH (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

#define MAX_CHANNEL_VAL (813)
#define DAC_8_BIT_RESOLUTION (255)

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

class PWMMotors {
    private:
        double roundSmooth(double val) {
            return sqrt((2 * val) - (val * val));
        }
        uint16_t channelValToDac(uint16_t val) {
            return (uint16_t)(DAC_8_BIT_RESOLUTION * roundSmooth((double)val/MAX_CHANNEL_VAL));
        }

    public:
        PWMMotors(int leftForwardPin, int leftBackwardPin, int rightForwardPin, int rightBackwardPin) {
            uint32_t LEDC_FREQUENCY = 200000;

            ledc_timer_config_t pwmTimer = {
                .speed_mode       = LEDC_LOW_SPEED_MODE,
                .duty_resolution  = LEDC_TIMER_8_BIT,
                .timer_num        = LEDC_TIMER_0,
                .freq_hz          = LEDC_FREQUENCY,
                .clk_cfg          = LEDC_AUTO_CLK,
                .deconfigure     = false
            };
            ESP_ERROR_CHECK(ledc_timer_config(&pwmTimer));


            ledc_channel_config_t leftForwardPimConfig = {
                .gpio_num       = leftForwardPin,
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_0,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = DAC_8_BIT_RESOLUTION,
                .hpoint         = 0,
                .flags          = { .output_invert = 0 }
            };
            ledc_channel_config_t leftBackwardPimConfig = {
                .gpio_num       = leftBackwardPin,
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_1,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = DAC_8_BIT_RESOLUTION,
                .hpoint         = 0,
                .flags          = { .output_invert = 0 }
            };
            ledc_channel_config_t rightForwardPimConfig = {
                .gpio_num       = rightForwardPin,
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_2,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = DAC_8_BIT_RESOLUTION,
                .hpoint         = 0,
                .flags          = { .output_invert = 0 }
            };
            ledc_channel_config_t rightBackwardPimConfig = {
                .gpio_num       = rightBackwardPin,
                .speed_mode     = LEDC_LOW_SPEED_MODE,
                .channel        = LEDC_CHANNEL_3,
                .intr_type      = LEDC_INTR_DISABLE,
                .timer_sel      = LEDC_TIMER_0,
                .duty           = DAC_8_BIT_RESOLUTION,
                .hpoint         = 0,
                .flags          = { .output_invert = 0 }
            };


            ESP_ERROR_CHECK(ledc_channel_config(&leftForwardPimConfig));
            ESP_ERROR_CHECK(ledc_channel_config(&leftBackwardPimConfig));
            ESP_ERROR_CHECK(ledc_channel_config(&rightForwardPimConfig));
            ESP_ERROR_CHECK(ledc_channel_config(&rightBackwardPimConfig));
        }

        void setSpeed(uint16_t ml_speed, uint8_t mlDir, uint16_t mr_speed, uint8_t mrDir) {
            uint16_t dutyML = DAC_8_BIT_RESOLUTION - this->channelValToDac(ml_speed);
            if(mlDir) {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyML));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, DAC_8_BIT_RESOLUTION));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
            } else {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyML));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, DAC_8_BIT_RESOLUTION));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
            }

            uint16_t dutyMR = DAC_8_BIT_RESOLUTION - this->channelValToDac(mr_speed);
            if(mrDir) {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyMR));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, DAC_8_BIT_RESOLUTION));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
            } else {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, dutyMR));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, DAC_8_BIT_RESOLUTION));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
            }
        }
};

class Controller {
    struct motorsState_t {
        uint16_t ml_speed;
        uint16_t mr_speed;
        uint8_t mlDir;
        uint8_t mrDir;
    };
    private:
        PWMMotors *motors;
        motorsState_t motorsState;
        void convertElrsToMotorsDirection(crsf_channels_t *channels) {
            int maxChannelVal = MAX_CHANNEL_VAL;
            uint16_t stickZeroPosition = 992;
            uint16_t rotationStick = channels->ch0;
            uint16_t throttleStick = channels->ch1;
            int throttleVal = throttleStick - stickZeroPosition;
            int rotationVal = rotationStick - stickZeroPosition;

            int lSpeedAbsolute;
            int rSpeedAbsolute;
            if(throttleVal < -50) {
                lSpeedAbsolute = (throttleVal - rotationVal);
                rSpeedAbsolute = (throttleVal + rotationVal);
            } else {
                lSpeedAbsolute = (throttleVal + rotationVal);
                rSpeedAbsolute = (throttleVal - rotationVal);
            }

            motorsState.ml_speed = (uint16_t)(sqrt(abs(lSpeedAbsolute))*sqrt(maxChannelVal));
            motorsState.mr_speed = (uint16_t)(sqrt(abs(rSpeedAbsolute))*sqrt(maxChannelVal));

            if(motorsState.ml_speed > maxChannelVal) {
                motorsState.ml_speed = MAX_CHANNEL_VAL;
            }
            if(motorsState.mr_speed > maxChannelVal) {
                motorsState.mr_speed = MAX_CHANNEL_VAL;
            }
            motorsState.mlDir = (lSpeedAbsolute > 0) ? 1 : 0;
            motorsState.mrDir = (rSpeedAbsolute > 0) ? 1 : 0;
            this->motors->setSpeed(motorsState.ml_speed, motorsState.mlDir, motorsState.mr_speed, motorsState.mrDir);
        }
    public:
        Controller(PWMMotors *motors) {
            this->motors = motors;
        };
        void packageReceived(crsf_channels_t *channels) {
            this->convertElrsToMotorsDirection(channels);
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
                .baud_rate = 420000,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh = BUF_SIZE,
                .source_clk = UART_SCLK_DEFAULT
            };
            const int uart_buffer_size = 1024;
            ESP_ERROR_CHECK(uart_driver_install(portNum, uart_buffer_size, 1024, 10, &uart_queue, 0));
            ESP_ERROR_CHECK(uart_param_config(portNum, &uart_config));
            ESP_ERROR_CHECK(uart_set_pin(portNum, 4, 5, 18, 19));
            ESP_ERROR_CHECK(uart_enable_rx_intr(portNum));

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
                if(size > maxSize) {
                    size = (uint8_t)maxSize;
                }

                uart_read_bytes(this->portNum, data, size, portMAX_DELAY);
                uint8_t crc = crcFast(data, size - 1);
                
                if(crc == data[size-1]) {
                    if (data[0] == 0x16) {
                        for (int ix = 1; ix < size; ix++) {
                            BF_channels[ix-1] = data[ix];
                        }
                        controller->packageReceived(cr_channels);
                    }
                }
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
                uartElrs->read(event.size);
                break;
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

    PWMMotors *pwmMotors = new PWMMotors(21, 15, 16, 17);
    Controller *controller = new Controller(pwmMotors);
    UartELRS *uart = new UartELRS(UART_NUM_2, controller);



    xTaskCreate(uart_event_task, "uart_event_task", 2048, uart, 12, NULL);
}
