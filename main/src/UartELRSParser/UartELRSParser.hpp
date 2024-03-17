#include "../Controller/Controller.hpp"
#include "driver/uart.h"
#include "../../esp-elrs-rx.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>


#define PackageMaxSize 64

#ifndef UartELRSParserCl
#define UartELRSParserCl

class UartELRSParser {
private:
  bool reading = false;
  uint8_t crcTable[256];
  uint8_t BF_channels[PackageMaxSize];
  Controller *controller;
  crsf_channels_t *cr_channels;
  uint8_t crcFast(uint8_t const message[], int nBytes);
  void crcInit(void);
public:
  uart_port_t portNum;
  UartELRSParser(uart_port_t portNum, Controller *controller, QueueHandle_t *uart_queue);
  void read(uint16_t maxSize);
};

#endif
