#include <cmath>
#include "driver/gpio.h"
#include "../PWMMotorsControl/PWMMotorsControl.hpp"

#ifndef ControllerCl
#define ControllerCl

struct motorsState_t {
  uint16_t ml_speed;
  uint16_t mr_speed;
  uint8_t mlDir;
  uint8_t mrDir;
};

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
    private:
        PWMMotorsControl *motors;
        motorsState_t motorsState;
        void convertElrsToMotorsDirection(crsf_channels_t *channels);
    public:
        Controller(PWMMotorsControl *motors);
        void packageReceived(crsf_channels_t *channels);
};
#endif
