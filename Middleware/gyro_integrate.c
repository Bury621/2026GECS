#include "gyro_integrate.h"
static float yaw=0; //初始化积分的函数
/**
 * @brief 初始化mpu6050并且初始化这个积分的库
 * 
 * @param csx mpu6050驱动的句柄指针
 */
void Gyro_integrate_Init(mpu6050_handle_t *csx)
{
    DRIVER_MPU6050_LINK_INIT(csx, mpu6050_handle_t);
    DRIVER_MPU6050_LINK_IIC_INIT(csx, mpu6050_interface_iic_init);
    DRIVER_MPU6050_LINK_IIC_DEINIT(csx, mpu6050_interface_iic_deinit);
    DRIVER_MPU6050_LINK_IIC_READ(csx, mpu6050_interface_iic_read);
    DRIVER_MPU6050_LINK_IIC_WRITE(csx, mpu6050_interface_iic_write);
    DRIVER_MPU6050_LINK_DELAY_MS(csx, mpu6050_interface_delay_ms);
    DRIVER_MPU6050_LINK_DEBUG_PRINT(csx, mpu6050_interface_debug_print);
    DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(csx, mpu6050_interface_receive_callback);
    csx->iic_addr=MPU6050_ADDRESS_AD0_LOW;
    mpu6050_init(csx);

    
}
/**
 * @brief 在定时器中断里面定时积分,周期是GYRO_INTEGRATE_UPDATE_CYCLE
 * 
 * @param csx 句柄指针
 */
float Gyro_update(mpu6050_handle_t *csx)
{
    float gyro_dps[3];
    mpu6050_read(csx,NULL,NULL,NULL,&gyro_dps,NULL);
    //printf("%f\r\n",(gyro_dps[2]- GYRO_INTEGRATE_BASIC_YAW) * (0.001f*GYRO_INTEGRATE_UPDATE_CYCLE));
    if((gyro_dps[2]>0?gyro_dps[2]:-gyro_dps[2])>=GYRO_DEAD_AREA)
    {
        yaw+=(gyro_dps[2]+ GYRO_INTEGRATE_BASIC_YAW) * (0.001f*GYRO_INTEGRATE_UPDATE_CYCLE); //只积分偏航角
    }
    return gyro_dps[2]+ GYRO_INTEGRATE_BASIC_YAW;
}
/**
 * @brief 返回积分之后的角度值
 * 
 * @return float 
 */
float Gyro_read(void)
{
    return yaw;
}

/**
 * @brief 重置角度
 * 
 */
void Gyro_reset(void)
{
    yaw=0;
}