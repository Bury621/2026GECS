#ifndef DMG6220_H
#define DMG6220_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define DMG6220_DEFAULT_P_MAX 12.5f
#define DMG6220_DEFAULT_V_MAX 45.0f
#define DMG6220_DEFAULT_T_MAX 10.0f

typedef enum
{
    DMG6220_CMD_ENABLE = 0,
    DMG6220_CMD_DISABLE,
    DMG6220_CMD_SAVE_ZERO,
    DMG6220_CMD_CLEAR_ERROR
} DMG6220_Command;

typedef struct
{
    uint8_t motor_id;
    uint8_t state;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    float position;
    float velocity;
    float torque;
    uint8_t temperature_mos;
    uint8_t temperature_rotor;
    uint32_t last_rx_tick;
    uint32_t rx_count;
} DMG6220_Feedback;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t can_id;
    uint8_t master_id;
    float position_max;
    float velocity_max;
    float torque_max;
    DMG6220_Feedback feedback;
    uint32_t rx_error_count;
} DMG6220;

HAL_StatusTypeDef DMG6220_Init(DMG6220 *motor, CAN_HandleTypeDef *hcan,
                              uint8_t can_id, uint8_t master_id);

HAL_StatusTypeDef DMG6220_SendCommand(DMG6220 *motor,
                                     DMG6220_Command command);

HAL_StatusTypeDef DMG6220_SendMit(DMG6220 *motor, float position,
                                 float velocity, float kp, float kd,
                                 float torque);

HAL_StatusTypeDef DMG6220_SendPositionVelocity(DMG6220 *motor,
                                              float position,
                                              float velocity);

HAL_StatusTypeDef DMG6220_SendVelocity(DMG6220 *motor, float velocity);

uint32_t DMG6220_Poll(DMG6220 *motor);

HAL_StatusTypeDef DMG6220_WaitFeedback(DMG6220 *motor, uint32_t timeout_ms);

uint8_t DMG6220_IsOnline(const DMG6220 *motor, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
