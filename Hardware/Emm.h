#ifndef __EMM_H
#define __EMM_H

#include "stdint.h"
#include "main.h"

#define EMM_UART huart3
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
int32_t Emm_Read_Pulse_Count(uint8_t addr); // 按速度命令累计脉冲数，带方向
void Emm_Clear_Pulse_Count(uint8_t addr);   // 只清零里程基准，不修改电机坐标
void Emm_Set_Forward_Speed(int32_t speed, EMM_SYNC_SIGN sync); // 设置纵向里程速度，只使用 vx
int32_t Emm_Read_Forward_Pulse_Count(void); // 读取仅由 vx 累计的纵向里程脉冲
void Emm_Clear_Forward_Pulse_Count(void);   // 清零纵向里程基准
void Emm_Stop(uint8_t addr);
void Emm_Home(uint8_t addr,EMM_SYNC_SIGN sync);
void Emm_Sync_Trig(void);
#endif
