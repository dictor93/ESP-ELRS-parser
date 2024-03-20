#include "Controller.hpp"
#include "../../esp-elrs-rx.hpp"

class Controller {
    
    private:
        PWMMotorsControl *motors;
        motorsState_t motorsState;
        void convertElrsToMotorsDirection(crsf_channels_t *channels) {
            int maxChannelVal = MAX_CHANNEL_VAL;
            uint16_t stickZeroPosition = 992;
            uint16_t rotationStick = channels->ch0;
            uint16_t throttleStick = channels->ch1;
            int throttleVal = throttleStick - stickZeroPosition;
            int rotationVal = (int)((rotationStick - stickZeroPosition));


            int lSpeedAbsolute;
            int rSpeedAbsolute;
            if(throttleVal < -50) {
                lSpeedAbsolute = (throttleVal - rotationVal);
                rSpeedAbsolute = (throttleVal + rotationVal);
            } else {
                lSpeedAbsolute = (throttleVal + rotationVal);
                rSpeedAbsolute = (throttleVal - rotationVal);
            }

            printf("lSpeedAbsolute: %d, rSpeedAbsolute: %d\n", lSpeedAbsolute, rSpeedAbsolute);
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
        Controller(PWMMotorsControl *motors) {
            this->motors = motors;
        };
        void packageReceived(crsf_channels_t *channels) {
            this->convertElrsToMotorsDirection(channels);
        };
};