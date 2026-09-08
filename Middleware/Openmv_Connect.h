#ifndef __OPENMV_CONNECT_H
#define __OPENMV_CONNECT_H

#include "main.h"

#define OPENMV_UART huart5 //配置好的UART
#define OPENMV_CONNECT_DMA hdma1 //配置好的DMA
#define OPENMV_CONNECT_RX_BUFFER_SIZE 6 //数据缓冲区大小

extern UART_HandleTypeDef OPENMV_UART;
extern DMA_HandleTypeDef OPENMV_CONNECT_DMA;

typedef struct
{
    uint8_t rx_buf[OPENMV_CONNECT_RX_BUFFER_SIZE]; //接收缓冲区
}Openmv_csx;

typedef struct
{
    uint8_t line_error; //到边线的误差
    uint8_t turning_state; //是否在转弯状态，1是正在转弯，0是正常走路
    uint8_t state; //解析是否成功，必须判断，0的话就是成功，1的话就是失败
}Openmv_package; //返回数据包

void Openmv_Init(Openmv_csx *csx);
Openmv_package Openmv_Feed(Openmv_csx *csx,uint16_t size);
#endif