#ifndef GYRO_INTEGRATE_H
#define GYRO_INTEGRATE_H

#include "../Hardware/mpu6050/driver_mpu6050.h"
#include "../Hardware/mpu6050/driver_mpu6050_interface.h"

#define GYRO_INTEGRATE_UPDATE_CYCLE 10 //积分周期,单位ms
#define GYRO_INTEGRATE_BASIC_YAW 0.341786 //零偏校准量

#define GYRO_DEAD_AREA 0 //死区
void Gyro_integrate_Init(mpu6050_handle_t *csx);
float Gyro_update(mpu6050_handle_t *csx);
float Gyro_read(void);
void Gyro_reset(void);
#endif