#include "Servo.h"

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}
void Servo_Set(uint16_t angle)
{
    uint16_t ccr;
    if (angle > 180) {
        angle = 180;
    }
    uint16_t high_time_us = HIGH_TIME_MIN_US + (angle * (HIGH_TIME_MAX_US - HIGH_TIME_MIN_US) / 180);
    ccr = (uint16_t)((uint32_t)ARR_VALUE * high_time_us / PERIOD_US);
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, ccr);
}