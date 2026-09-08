#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "Debug.h"
#include "Led.h"
#include "button.h"
#include "Servo.h"
#include "protoflow.h"
#include "vofamanager.h"
#include "../Hardware/SSD1306/ssd1306.h"
#include "../Hardware/SSD1306/fonts.h"
#include "string.h"
#include "Emm.h"
#include "Maxi_Connect.h"

void App_Init(void);
void App_Run(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#endif