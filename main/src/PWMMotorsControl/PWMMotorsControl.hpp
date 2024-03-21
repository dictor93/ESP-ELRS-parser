#include <cmath>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#define WIDTH (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))
#define MAX_CHANNEL_VAL (813)
#define DAC_8_BIT_RESOLUTION (255)

#ifndef PWMMotorsControlCl
#define PWMMotorsControlCl

class PWMMotorsControl {
private:
  double roundSmooth(double val);
  uint16_t channelValToDac(uint16_t val);

public:
  PWMMotorsControl(int leftForwardPin, int leftBackwardPin, int rightForwardPin, int rightBackwardPin);
  void setSpeed(uint16_t ml_speed, uint8_t mlDir, uint16_t mr_speed, uint8_t mrDir);
};

#endif
