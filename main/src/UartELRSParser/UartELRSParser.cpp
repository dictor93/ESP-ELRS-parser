#include "UartELRSParser.hpp"

UartELRSParser::UartELRSParser(uart_port_t portNum, Controller *controller,
                               QueueHandle_t *uart_queue) {
    this->controller = controller;
    this->portNum = portNum;

    uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = BUF_SIZE,
    };
    const int uart_buffer_size = 1024;
    ESP_ERROR_CHECK(uart_driver_install(portNum, uart_buffer_size, 1024, 10,
                                        uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(portNum, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(portNum, 4, 5, 18, 19));
    ESP_ERROR_CHECK(uart_enable_rx_intr(portNum));

    this->crcInit();
};

void UartELRSParser::crcInit(void) {
    uint8_t remainder;
    for (int dividend = 0; dividend < 256; ++dividend) {
        remainder = dividend << (WIDTH - 8);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ 0xD5;
            } else {
                remainder = (remainder << 1);
            }
        }
        crcTable[dividend] = remainder;
    }
}

uint8_t UartELRSParser::crcFast(uint8_t const message[], int nBytes) {
    uint8_t data;
    uint8_t remainder = 0;
    for (int byte = 0; byte < nBytes; ++byte) {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }
    return (remainder);
}

void UartELRSParser::read(uint16_t maxSize) {
    if (this->reading) {
        return;
    }
    this->reading = true;

    uint8_t currentByte;
    uart_read_bytes(this->portNum, &currentByte, 1, portMAX_DELAY);
    if (currentByte == 0xc8 || currentByte == 0xee || currentByte == 0xea ||
        currentByte == 0xec) {
        uint8_t size;
        uart_read_bytes(this->portNum, &size, 1, portMAX_DELAY);
        if (size > maxSize) {
            size = (uint8_t)maxSize;
        }
        uint8_t data[size];
        uart_read_bytes(this->portNum, data, size, portMAX_DELAY);
        uint8_t crc = crcFast(data, size - 1);

        if (crc == data[size - 1]) {
            if (data[0] == 0x16) {
                for (int ix = 1; ix < size; ix++) {
                    BF_channels[ix - 1] = data[ix];
                }
                controller->packageReceived(cr_channels);
            }
        }
    }
    this->reading = false;
};

void UartELRSParser::onUartIntQueueItem(uart_event_t *event, QueueHandle_t *uart_queue) {
    switch (event->type) {
        case UART_DATA:
            this->read(event->size);
            break;
        case UART_FIFO_OVF:
            uart_flush_input(this->portNum);
            xQueueReset(*uart_queue);
            break;
        case UART_BUFFER_FULL:
            uart_flush_input(this->portNum);
            xQueueReset(*uart_queue);
            break;
        default:
            break;
    }
}
