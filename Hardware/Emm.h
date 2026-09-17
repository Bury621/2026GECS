#ifndef __EMM_H
#define __EMM_H

#include "stdint.h"
#include "main.h"

#define EMM_UART huart2
extern UART_HandleTypeDef EMM_UART;
typedef enum{
	EMM_DIR_CW=0,
	EMM_DIR_CCW
}EMM_DIR;
typedef enum {
    EMM_MODE_RELATIVE_TO_PREVIOUS = 0,  // 相对上一输入目标位置
    EMM_MODE_ABSOLUTE = 1,              // 相对坐标零点
    EMM_MODE_RELATIVE_TO_CURRENT = 2    // 相对当前实时位置
}EMM_MotorMoveMode;
typedef enum{
    EMM_SYNC_DISABLE = 0 ,
    EMM_SYNC_ENABLE = 1
}EMM_SYNC_SIGN;

void Emm_Speed_Mode(uint8_t addr, int32_t speed, uint8_t acceleration,EMM_SYNC_SIGN sync);
void Emm_position_Mode(uint8_t addr, int32_t speed, uint8_t acceleration, 
                       uint32_t Pulse_Count, EMM_MotorMoveMode movemode,EMM_SYNC_SIGN sync);
void Emm_Stop(uint8_t addr);
void Emm_Home(uint8_t addr,EMM_SYNC_SIGN sync);
void Emm_Sync_Trig(void);
#endif