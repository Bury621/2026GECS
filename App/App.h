#ifndef __APP_H
#define __APP_H

#include "main.h"

#include "string.h"
#include "Debug.h"

#include "Led.h"
#include "button.h"
#include "Servo.h"
#include "Ray.h"
#include "DMG6220.h"
#include "Emm.h"
#include "../Hardware/SSD1306/ssd1306.h"
#include "../Hardware/SSD1306/fonts.h"
#include "hwt101.h"

#include "Maxi_Connect.h"
#include "Openmv_Connect.h"
#include "vofamanager.h"
#include "robot.h"

void App_Init(void);
void App_Run(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern CAN_HandleTypeDef hcan1;

//extern TIM_HandleTypeDef htim7;
#endif