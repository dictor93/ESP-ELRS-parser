#include "esp-elrs-rx.hpp"
#include "./src/UartELRSParser/UartELRSParser.hpp"
#include "./src/PWMMotorsControl/PWMMotorsControl.hpp"
#include "./src/Controller/Controller.hpp"


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
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for (;;) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            uartElrs->onUartIntQueueItem(&event, &uart_queue);
        }
    }
    free(dtmp);
    dtmp = NULL;
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
