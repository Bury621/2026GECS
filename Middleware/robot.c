#include "robot.h"

/* 轮子编号：1 左前，2 右前，3 左后，4 右后 */
#define WHEEL_LF  1
#define WHEEL_RF  2
#define WHEEL_LR  3
#define WHEEL_RR  4

/* 加速度，按实际需求设置，0~255 */
#define MOVE_ACCEL  0

/* 最大轮速 */
#define WHEEL_MAX_SPEED  3000

#define ROBOT_REMOTE_CONTROL_SPEED 150
/* 方向修正：1 保持，-1 取反，按实际调试修改 */
static const int8_t dir_fix[4] = { -1, 1, -1, 1 };

/* 缓存三个运动分量，单轴更新时保留其他分量 */
static int16_t robot_vx = 0;
static int16_t robot_vy = 0;
static int16_t robot_omega = 0;

/* 角度环内部状态 */
static float robot_angle_continuous = 0.0f;
static float robot_angle_last_yaw = 0.0f;
static float robot_angle_last_omega = 0.0f;
static uint8_t robot_angle_has_yaw = 0;

Robot_remote_sign remote_sign=ROBOT_REMOTE_STOP; //遥控状态标志位

static void Robot_update_move(void)
{
    /* X型麦克纳姆轮速度分解 */
    int32_t w1 = (int32_t)robot_vx + robot_vy - robot_omega;   /* 左前 */
    int32_t w2 = (int32_t)robot_vx - robot_vy + robot_omega;   /* 右前 */
    int32_t w3 = (int32_t)robot_vx - robot_vy - robot_omega;   /* 左后 */
    int32_t w4 = (int32_t)robot_vx + robot_vy + robot_omega;   /* 右后 */

    /* 四个轮子的原始值 */
    int32_t raw[4] = { w1, w2, w3, w4 };

    /* 找最大绝对值，用于等比缩放 */
    int32_t max_abs = 0;
    for (int i = 0; i < 4; i++) {
        int32_t a = raw[i] > 0 ? raw[i] : -raw[i];
        if (a > max_abs) max_abs = a;
    }

    /* 超过 3000 则等比缩放，保持运动方向比例 */
    if (max_abs > WHEEL_MAX_SPEED) {
        for (int i = 0; i < 4; i++) {
            raw[i] = (int32_t)((int64_t)raw[i] * WHEEL_MAX_SPEED / max_abs);
        }
    }

    /* 应用方向修正 */
    for (int i = 0; i < 4; i++) {
        raw[i] *= dir_fix[i];
    }

    /* 依次发送给四个电机，addr 对应轮子编号 */
    Emm_Speed_Mode(WHEEL_LF, raw[0], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_RF, raw[1], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_LR, raw[2], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Speed_Mode(WHEEL_RR, raw[3], MOVE_ACCEL, EMM_SYNC_ENABLE);
    HAL_Delay(ROBOT_MOVE_SET_MOTOR_DELAY);
    Emm_Sync_Trig();
}

/**
 * @brief 设置小车的运动状态（X型麦克纳姆轮）
 *
 * 轮子布局:
 *     1  2
 *     3  4
 *
 * @param vx    前后速度，向前为正
 * @param vy    左右速度，向右为正
 * @param omega 旋转速度，逆时针为正
 */
void Robot_set_move(int16_t vx, int16_t vy, int16_t omega)
{
    robot_vx = vx;
    robot_vy = vy;
    robot_omega = omega;
    Robot_update_move();
}

void Robot_set_vx(int16_t vx)
{
    robot_vx = vx;
    Robot_update_move();
}

void Robot_set_vy(int16_t vy)
{
    robot_vy = vy;
    Robot_update_move();
}

void Robot_set_omega(int16_t omega)
{
    robot_omega = omega;
    Robot_update_move();
}

void Robot_Angle_Update(hwt101_csx *hwt101, PIDController *pid, uint16_t target_angle)
{
    float angle_sensor_data;
    if(hwt101_read_yaw(hwt101,&angle_sensor_data)!=0)
    {
        return;
    }

    if(robot_angle_has_yaw==0)
    {
        robot_angle_continuous=angle_sensor_data;
        robot_angle_last_yaw=angle_sensor_data;
        robot_angle_has_yaw=1;
        pid->prevMeasurement=robot_angle_continuous;
    }
    else
    {
        float delta_angle=angle_sensor_data-robot_angle_last_yaw;
        if(delta_angle>180.0f)
        {
            delta_angle-=360.0f;
        }
        else if(delta_angle<-180.0f)
        {
            delta_angle+=360.0f;
        }
        robot_angle_continuous+=delta_angle;
        robot_angle_last_yaw=angle_sensor_data;
    }

    int16_t bounded_target=(int16_t)((int32_t)target_angle%360);
    if(bounded_target>180)
    {
        bounded_target-=360;
    }

    float target_error=(float)bounded_target-robot_angle_continuous;
    while(target_error>180.0f)
    {
        target_error-=360.0f;
    }
    while(target_error<-180.0f)
    {
        target_error+=360.0f;
    }

    float target_setpoint=robot_angle_continuous+target_error;
    float angle_pid_output=PIDController_Update(pid,target_setpoint,robot_angle_continuous);

    /* HWT101 的正向与本车 omega 的正向相反 */
    float omega_output=-angle_pid_output;
    float omega_delta=omega_output-robot_angle_last_omega;
    if(omega_delta>10.0f)
    {
        omega_output=robot_angle_last_omega+10.0f;
    }
    else if(omega_delta<-10.0f)
    {
        omega_output=robot_angle_last_omega-10.0f;
    }
    robot_angle_last_omega=omega_output;
    Robot_set_omega((int16_t)omega_output);
}

void Robot_stop_move(void)
{
    Robot_set_move(0,0,0);
}
void Robot_remote_control(void)
{
    if(remote_sign==ROBOT_REMOTE_FORWARD)
    {
        Robot_set_vx(ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_BACKWARD)
    {
        Robot_set_vx(-ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_LEFT)
    {
        Robot_set_vy(-ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    if(remote_sign==ROBOT_REMOTE_RIGHT)
    {
        Robot_set_vy(ROBOT_REMOTE_CONTROL_SPEED);
        remote_sign=0;
    }
    else if(remote_sign==ROBOT_REMOTE_TO_STOP)
    {
        Robot_set_move(0,0,0);
        remote_sign=0;
    }
}
