#ifndef DMG6220_H
#define DMG6220_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t can_id;
    uint8_t position_valid;
    uint8_t error;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float target_rad;
    float target_velocity_rad_s;
    uint32_t last_target_tick;
    uint8_t target_valid;
} DMG6220;

HAL_StatusTypeDef DMG6220_Init(DMG6220 *motor, CAN_HandleTypeDef *hcan,
                              uint8_t can_id);

HAL_StatusTypeDef DMG6220_Enable(DMG6220 *motor);

/* Read pending feedback frames and update the cached motor state. */
HAL_StatusTypeDef DMG6220_Update(DMG6220 *motor);

/* Return the latest feedback angle in degrees. */
HAL_StatusTypeDef DMG6220_GetPosition(DMG6220 *motor, float *angle_deg);

/* Wait until at least one valid feedback frame has been received. */
HAL_StatusTypeDef DMG6220_WaitForPosition(DMG6220 *motor,
                                         uint32_t timeout_ms);

/*
 * Position-velocity mode. The target is moved to an equivalent angle on the
 * shortest path from the latest feedback position. speed_dps is the maximum
 * mechanical speed; its sign is ignored. The motor ControlMode must be set to
 * position-velocity mode in DAMIAO's configuration tool.
 */
HAL_StatusTypeDef DMG6220_MoveAngle(DMG6220 *motor, float angle_deg,
                                    float speed_dps);

/* Raw position-velocity frame: angle and speed are in radians/rad/s. */
HAL_StatusTypeDef DMG6220_SendPosition(DMG6220 *motor, float position_rad,
                                      float velocity_rad_s);

/* Resend the latched position command every 10 ms while it is active. */
HAL_StatusTypeDef DMG6220_Service(DMG6220 *motor);

HAL_StatusTypeDef DMG6220_SendMit(DMG6220 *motor, float position,
                                 float velocity, float kp, float kd,
                                 float torque);

#ifdef __cplusplus
}
#endif

#endif
