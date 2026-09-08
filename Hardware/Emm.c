#include "Emm.h"
void Emm_Speed_Mode(uint8_t addr, int32_t speed, uint8_t acceleration,EMM_SYNC_SIGN sync)
{
    // 1. 先取绝对值用于速度值
    int32_t abs_speed = speed > 0 ? speed : -speed;
    
    // 2. 速度限幅（绝对值限幅）
    if (abs_speed > 3000) abs_speed = 3000;
    
    // 3. 方向判断（基于原始值）
    EMM_DIR dir = (speed >= 0) ? EMM_DIR_CW : EMM_DIR_CCW;
    
    uint8_t command[8] = {
        addr, 
        0xF6, 
        dir, 
        (uint8_t)(abs_speed >> 8),    // 高8位
        (uint8_t)(abs_speed & 0xFF),  // 低8位
        acceleration, 
        sync, 
        0x6B
    };
    HAL_UART_Transmit(&EMM_UART, command, 8, HAL_MAX_DELAY);
}

void Emm_position_Mode(uint8_t addr, int32_t speed, uint8_t acceleration, 
                       uint32_t Pulse_Count, EMM_MotorMoveMode movemode,EMM_SYNC_SIGN sync)
{
    // 参数改为 int32_t，支持正负速度
    int32_t abs_speed = speed > 0 ? speed : -speed;
    if (abs_speed > 3000) abs_speed = 3000;
    
    EMM_DIR dir = (speed >= 0) ? EMM_DIR_CW : EMM_DIR_CCW;
    
    uint8_t command[13] = {
        addr, 
        0xFD, 
        dir, 
        (uint8_t)(abs_speed >> 8),    // 高8位
        (uint8_t)(abs_speed & 0xFF),  // 低8位
        acceleration,
        (uint8_t)(Pulse_Count >> 24),  // 脉冲数 高8位
        (uint8_t)((Pulse_Count >> 16) & 0xFF),
        (uint8_t)((Pulse_Count >> 8) & 0xFF),
        (uint8_t)(Pulse_Count & 0xFF),
        movemode, 
        sync, 
        0x6B
    };
    HAL_UART_Transmit(&EMM_UART, command, 13, HAL_MAX_DELAY);
}

void Emm_Stop(uint8_t addr)
{
    uint8_t command[5] = {addr, 0xFE, 0x98, 0x00, 0x6B};
    HAL_UART_Transmit(&EMM_UART, command, 5, HAL_MAX_DELAY);
}
void Emm_Home(uint8_t addr,EMM_SYNC_SIGN sync)
{
    uint8_t command[5] = {addr,0x9A,0x02,sync,0x6B};
    HAL_UART_Transmit(&EMM_UART, command, 5, HAL_MAX_DELAY);
}
void Emm_Sync_Trig(void)
{
    uint8_t command[4] = {0x00,0xFF,0x66,0x6B};
    HAL_UART_Transmit(&EMM_UART, command, 4, HAL_MAX_DELAY);
}