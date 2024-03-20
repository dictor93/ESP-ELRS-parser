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

#include "driver/uart.h" // Include the header file that defines "UART_NUM_2"

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    PWMMotorsControl *pwmMotors = new PWMMotorsControl(21, 15, 16, 17);
    Controller *controller = new Controller(pwmMotors);
    UartELRSParser *uart = new UartELRSParser(UART_NUM_2, controller, &uart_queue);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, uart, 12, NULL);
}
