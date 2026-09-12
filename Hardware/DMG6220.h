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
} DMG6220;

HAL_StatusTypeDef DMG6220_Init(DMG6220 *motor, CAN_HandleTypeDef *hcan,
                              uint8_t can_id);

HAL_StatusTypeDef DMG6220_Enable(DMG6220 *motor);

/* Move to an absolute angle from zero. angle_deg: degree, torque_nm: N.m. */
HAL_StatusTypeDef DMG6220_MoveAngle(DMG6220 *motor, float angle_deg,
                                    float torque_nm);

HAL_StatusTypeDef DMG6220_SendMit(DMG6220 *motor, float position,
                                 float velocity, float kp, float kd,
                                 float torque);

#ifdef __cplusplus
}
#endif

#endif
