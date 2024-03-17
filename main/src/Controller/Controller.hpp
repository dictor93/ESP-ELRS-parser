#include <cmath>
#include "../../esp-elrs-rx.hpp"
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
