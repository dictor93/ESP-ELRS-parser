#include "PWMMotorsControl.hpp"

PWMMotorsControl::PWMMotorsControl(int leftForwardPin, int leftBackwardPin,
                                   int rightForwardPin, int rightBackwardPin) {
    uint32_t LEDC_FREQUENCY = 200000;

    ledc_timer_config_t pwmTimer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                    .duty_resolution = LEDC_TIMER_8_BIT,
                                    .timer_num = LEDC_TIMER_0,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK,
                                    .deconfigure = false};
    ESP_ERROR_CHECK(ledc_timer_config(&pwmTimer));

    ledc_channel_config_t leftForwardPimConfig = {
        .gpio_num = leftForwardPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = DAC_8_BIT_RESOLUTION,
        .hpoint = 0,
        .flags = {.output_invert = 0}};
    ledc_channel_config_t leftBackwardPimConfig = {
        .gpio_num = leftBackwardPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = DAC_8_BIT_RESOLUTION,
        .hpoint = 0,
        .flags = {.output_invert = 0}};
    ledc_channel_config_t rightForwardPimConfig = {
        .gpio_num = rightForwardPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = DAC_8_BIT_RESOLUTION,
        .hpoint = 0,
        .flags = {.output_invert = 0}};
    ledc_channel_config_t rightBackwardPimConfig = {
        .gpio_num = rightBackwardPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = DAC_8_BIT_RESOLUTION,
        .hpoint = 0,
        .flags = {.output_invert = 0}};

    ESP_ERROR_CHECK(ledc_channel_config(&leftForwardPimConfig));
    ESP_ERROR_CHECK(ledc_channel_config(&leftBackwardPimConfig));
    ESP_ERROR_CHECK(ledc_channel_config(&rightForwardPimConfig));
    ESP_ERROR_CHECK(ledc_channel_config(&rightBackwardPimConfig));
}

double PWMMotorsControl::roundSmooth(double val) {
    return sqrt((2 * val) - (val * val));
}

uint16_t PWMMotorsControl::channelValToDac(uint16_t val) {
    return (uint16_t)(DAC_8_BIT_RESOLUTION *
                      roundSmooth((double)val / MAX_CHANNEL_VAL));
}

void PWMMotorsControl::setSpeed(uint16_t ml_speed, uint8_t mlDir,
                                uint16_t mr_speed, uint8_t mrDir) {
    printf("ml_speed: %d, mlDir: %d, mr_speed: %d, mrDir: %d\n", ml_speed,
           mlDir, mr_speed, mrDir);
    uint16_t dutyML = DAC_8_BIT_RESOLUTION - this->channelValToDac(ml_speed);
    if (mlDir) {
        ESP_ERROR_CHECK(
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyML));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                                      DAC_8_BIT_RESOLUTION));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    } else {
        ESP_ERROR_CHECK(
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyML));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                      DAC_8_BIT_RESOLUTION));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }

    uint16_t dutyMR = DAC_8_BIT_RESOLUTION - this->channelValToDac(mr_speed);
    if (mrDir) {
        ESP_ERROR_CHECK(
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyMR));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3,
                                      DAC_8_BIT_RESOLUTION));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
    } else {
        ESP_ERROR_CHECK(
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, dutyMR));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2,
                                      DAC_8_BIT_RESOLUTION));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
    }
}
