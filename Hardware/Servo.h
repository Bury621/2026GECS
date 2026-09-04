#ifndef __SERVO_H
#define __SERVO_H
#include "main.h"

#define ARR_VALUE         19999      // 自动重载值（8000级分辨率）
#define PERIOD_US         20000     // PWM周期 20ms = 20000μs
#define ANGLE_MIN         0         // 最小角度
#define ANGLE_MAX         180       // 最大角度
#define HIGH_TIME_MIN_US  500       // 0°对应0.5ms = 500μs
#define HIGH_TIME_MAX_US  2500      // 180°对应2.5ms = 2500μs

extern TIM_HandleTypeDef htim13;
void Servo_Init(void);
void Servo_Set(uint16_t angel);
#endif