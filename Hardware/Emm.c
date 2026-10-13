#include "Emm.h"

#define EMM_PULSES_PER_REV 3200
#define EMM_RPM_MS_DIVISOR 60000

static int64_t emm_pulse_numerator[256] = {0};
static int64_t emm_pulse_offset_numerator[256] = {0};
static int32_t emm_active_speed[256] = {0};
static int32_t emm_pending_speed[256] = {0};
static uint32_t emm_last_tick[256] = {0};
static uint8_t emm_speed_valid[256] = {0};
static uint8_t emm_speed_pending[256] = {0};

/* 纵向里程只累计 vx，不包含 vy 和 omega。 */
static int64_t emm_forward_pulse_numerator = 0;
static int64_t emm_forward_pulse_offset_numerator = 0;
static int32_t emm_active_forward_speed = 0;
static int32_t emm_pending_forward_speed = 0;
static uint32_t emm_forward_last_tick = 0;
static uint8_t emm_forward_speed_valid = 0;
static uint8_t emm_forward_speed_pending = 0;

static void Emm_Update_Forward_Pulse_Count(uint32_t now)
{
    uint32_t elapsed = 0;

    if (!emm_forward_speed_valid)
    {
        emm_forward_last_tick = now;
        return;
    }

    elapsed = now - emm_forward_last_tick;
    if (elapsed != 0)
    {
        emm_forward_pulse_numerator += (int64_t)emm_active_forward_speed *
                                       EMM_PULSES_PER_REV * elapsed;
        emm_forward_last_tick = now;
    }
}

static void Emm_Update_Pulse_Count(uint8_t addr, uint32_t now)
{
    uint32_t elapsed = 0;

    if (!emm_speed_valid[addr])
    {
        emm_last_tick[addr] = now;
        return;
    }

    elapsed = now - emm_last_tick[addr];
    if (elapsed != 0)
    {
        emm_pulse_numerator[addr] += (int64_t)emm_active_speed[addr] *
                                     EMM_PULSES_PER_REV * elapsed;
        emm_last_tick[addr] = now;
    }
}

void Emm_Speed_Mode(uint8_t addr, int32_t speed, uint8_t acceleration,EMM_SYNC_SIGN sync)
{
    // 1. 先取绝对值用于速度值
    int32_t abs_speed = speed > 0 ? speed : -speed;
    
    // 2. 速度限幅（绝对值限幅）
    if (abs_speed > 3000) abs_speed = 3000;
    
    // 3. 方向判断（基于原始值）
    EMM_DIR dir = (speed >= 0) ? EMM_DIR_CW : EMM_DIR_CCW;
    int32_t signed_speed = (speed >= 0) ? abs_speed : -abs_speed;
    uint32_t now = HAL_GetTick();

    Emm_Update_Pulse_Count(addr, now);
    if (sync == EMM_SYNC_ENABLE)
    {
        emm_pending_speed[addr] = signed_speed;
        emm_speed_pending[addr] = 1;
    }
    else
    {
        emm_active_speed[addr] = signed_speed;
        emm_speed_valid[addr] = 1;
        emm_speed_pending[addr] = 0;
    }
    
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

int32_t Emm_Read_Pulse_Count(uint8_t addr)
{
    Emm_Update_Pulse_Count(addr, HAL_GetTick());
    return (int32_t)((emm_pulse_numerator[addr] - emm_pulse_offset_numerator[addr]) /
                     EMM_RPM_MS_DIVISOR);
}

void Emm_Clear_Pulse_Count(uint8_t addr)
{
    Emm_Update_Pulse_Count(addr, HAL_GetTick());
    emm_pulse_offset_numerator[addr] = emm_pulse_numerator[addr];
}

void Emm_Set_Forward_Speed(int32_t speed, EMM_SYNC_SIGN sync)
{
    uint32_t now = HAL_GetTick();

    if (sync == EMM_SYNC_ENABLE)
    {
        emm_pending_forward_speed = speed;
        emm_forward_speed_pending = 1;
    }
    else
    {
        Emm_Update_Forward_Pulse_Count(now);
        emm_active_forward_speed = speed;
        emm_forward_speed_valid = 1;
        emm_forward_speed_pending = 0;
    }
}

int32_t Emm_Read_Forward_Pulse_Count(void)
{
    Emm_Update_Forward_Pulse_Count(HAL_GetTick());
    return (int32_t)((emm_forward_pulse_numerator -
                      emm_forward_pulse_offset_numerator) /
                     EMM_RPM_MS_DIVISOR);
}

void Emm_Clear_Forward_Pulse_Count(void)
{
    Emm_Update_Forward_Pulse_Count(HAL_GetTick());
    emm_forward_pulse_offset_numerator = emm_forward_pulse_numerator;
}

void Emm_Stop(uint8_t addr)
{
    uint8_t command[5] = {addr, 0xFE, 0x98, 0x00, 0x6B};
    Emm_Update_Pulse_Count(addr, HAL_GetTick());
    emm_active_speed[addr] = 0;
    emm_speed_valid[addr] = 1;
    emm_speed_pending[addr] = 0;
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
    uint32_t now = HAL_GetTick();

    for (uint16_t addr = 0; addr < 256; addr++)
    {
        if (emm_speed_pending[addr])
        {
            Emm_Update_Pulse_Count((uint8_t)addr, now);
            emm_active_speed[addr] = emm_pending_speed[addr];
            emm_speed_valid[addr] = 1;
            emm_speed_pending[addr] = 0;
        }
    }

    if (emm_forward_speed_pending)
    {
        Emm_Update_Forward_Pulse_Count(now);
        emm_active_forward_speed = emm_pending_forward_speed;
        emm_forward_speed_valid = 1;
        emm_forward_speed_pending = 0;
    }
}
