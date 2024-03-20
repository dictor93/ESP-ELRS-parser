#include "./src/UartELRSParser/UartELRSParser.hpp"
#include "./src/PWMMotorsControl/PWMMotorsControl.hpp"
#include "./src/Controller/Controller.hpp"

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

#define PATTERN_CHR_NUM    (3)  

static const char *TAG = "ESP_ELRS_CONTROLLER";

extern "C"
{
    void app_main();
}

QueueHandle_t uart_queue;

static void uart_event_task(void *args) {
    UartELRSParser *uartElrs = (UartELRSParser *) args;
    uart_event_t event;
    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            uartElrs->onUartIntQueueItem(&event, &uart_queue);
        }
    }
    vTaskDelete(NULL);
}

#include "driver/uart.h" // Include the header file that defines "UART_NUM_2"

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    PWMMotorsControl *pwmMotors = new PWMMotorsControl(21, 15, 16, 17);
    Controller *controller = new Controller(pwmMotors);
    UartELRSParser *uart = new UartELRSParser(UART_NUM_2, controller, &uart_queue);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, uart, 12, NULL);
}
