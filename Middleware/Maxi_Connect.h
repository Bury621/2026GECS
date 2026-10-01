#ifndef __MAXI_CONNECT_H
#define __MAXI_CONNECT_H

#include "main.h"

#define MAXI_UART huart6 //配置好的UART
#define MAXI_CONNECT_DMA hdma2 //配置好的DMA
#define MAXI_CONNECT_RX_BUFFER_SIZE 16 //数据缓冲区大小

extern UART_HandleTypeDef MAXI_UART;
extern DMA_HandleTypeDef MAXI_CONNECT_DMA;

typedef struct
{
    uint8_t rx_buf[MAXI_CONNECT_RX_BUFFER_SIZE]; //接收缓冲区
}Maxi_csx;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint8_t position; //目标位置
    uint8_t state; //解析是否成功，必须判断，0的话就是成功，1的话就是失败
}Maxi_posion_package; //返回数据包

void Maxi_Init(Maxi_csx *csx);
void Maxi_Send_cmd(uint8_t cmd);
Maxi_posion_package Maxi_Feed(Maxi_csx *csx,uint16_t size);
#endif
