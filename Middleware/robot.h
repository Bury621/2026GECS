#ifndef __ROBOT_H
#define __ROBOT_H

#include "Emm.h"
#include "DMG6220.h"

#include <hwt101.h>
#include <PID.h>

#define ROBOT_MOVE_SET_MOTOR_DELAY 5

#define ROBOT_TURNING_JUJGUE_ANGLE_DEADBAND 1 //角度误差多少以内认为旋转完成

/* 差速转弯默认参数，实际线速度可通过 _ex 接口单独指定。
   omega = linear_speed * ROBOT_DIFF_TURN_OMEGA_GAIN / radius_cm，
   若实车转弯半径不准，优先标定该增益。 */
#define ROBOT_DIFF_TURN_DEFAULT_LINEAR_SPEED 150.0f
#define ROBOT_DIFF_TURN_MAX_OMEGA 500.0f
#define ROBOT_DIFF_TURN_OMEGA_GAIN 100.0f
#define ROBOT_DIFF_TURN_MIN_RADIUS_CM 0.1f
#define ROBOT_DIFF_TURN_ANGLE_DEADBAND 1.0f
#define ROBOT_DIFF_TURN_TIMEOUT_MS 30000U

#define ROBOT_QR_BACK_TIME 370 //扫二维码后退时间，单位10ms
#define ROBOT_QR_WAIT_TIME 150 //扫二维码等待时间，单位10ms
#define ROBOT_QR_FORWARD_TIME 370 //扫二维码前进时间，单位10ms
#define ROBOT_OBSTACLE_TURN_WAIT_TIME 100 //穿越障碍物前转弯等待时间，单位10ms
#define ROBOT_Y_ALIGN_TIME 100 //差速转弯后的横向闭环时间，单位10ms
#define ROBOT_ROAD_TIME1 102
#define ROBOT_ROAD_TIME2 49
#define ROBOT_ROAD_TIME3 194
#define ROBOT_ROAD_TIME4 154
//#define ROBOT_ROAD_TIME5 116
#define ROBOT_STRAGE3_FORWARD_TIME1 110 //TURN4前直行前半段时间，单位10ms
#define ROBOT_STRAGE3_FORWARD_TIME2 57 //TURN4前直行后半段时间，单位10ms
#define ROBOT_ROAD_TIME6 80

#define ROBOT_ROAD_TIME7 230 //从任务一到任务二那段直线的时间
#define ROBOT_ROAD_TIME8 80 //完成任务二之后那个直道的时间

#define ROBOT_TASK1_BALL_TARGET 134 //任务一闭环目标小球的位置
#define ROBOT_TASK1_BALL_ERROR_TOLERANCE 3.0f //任务一位置误差容差
#define ROBOT_TASK1_BALL_STABLE_TIME_MS 200U //误差持续在容差内的时间
#define ROBOT_TASK1_MAXI_CMD_DELAY_MS 500U //进入任务一后延迟发送视觉命令的时间

#define ROBOT_TASK2_TARGET_TARGET 300 //任务二闭环目标靶子的位置
#define ROBOT_TASK2_TARGET_ERROR_TOLERANCE 3.0f //任务二位置误差容差
#define ROBOT_TASK2_TARGET_STABLE_TIME_MS 200U //任务二误差持续在容差内的时间
#define ROBOT_TASK2_LASER_ON_TIME_MS 2000U //任务二激光打开持续时间

#define ROBOT_TASK3_HOSTAGE_TARGET 400 //任务三闭环人质的位置
#define ROBOT_TASK3_HOSTAGE_ERROR_TOLERANCE 4.0f //任务三人质位置误差容差
#define ROBOT_TASK3_HOSTAGE_STABLE_TIME_MS 200U //任务三误差持续在容差内的时间

#define ROBOT_ROAD_HOSTAGE_OFFSET_TIME 50 //任务三补偿时间
#define ROBOT_TASK3_DMG_125_ANGLE 110 //伸出夹爪前将DMG转到125度
#define ROBOT_TASK3_DMG_125_WAIT_TIME 135 //DMG转到125度后的等待时间，单位10ms
#define ROBOT_TASK3_OUT3_WAIT_TIME 50 //夹爪伸出后等待时间，单位10ms
#define ROBOT_TASK3_CLOSE_GRIPPER_WAIT_TIME 50 //夹爪闭合后等待时间，单位10ms
#define ROBOT_TASK3_CLAW_IN_WAIT_TIME 50 //夹爪收回后等待时间，单位10ms
#define ROBOT_TASK3_DMG_190_ANGLE 190 //任务三最后的DMG6220目标角度
#define ROBOT_TASK3_TO_END_DMG_235_ANGLE 235 //前往终点后DMG6220目标角度
#define ROBOT_TASK3_TO_END_DMG_235_DELAY_TIME 50 //进入前往终点状态后的等待时间，单位10ms
#define ROBOT_ROAD_TO_END_TIME 150 //夹取完成后前往终点的时间，单位10ms
typedef enum{
    ROBOT_REMOTE_STOP=0,
    ROBOT_REMOTE_FORWARD,
    ROBOT_REMOTE_BACKWARD,
    ROBOT_REMOTE_LEFT,
    ROBOT_REMOTE_RIGHT,
    ROBOT_REMOTE_TO_STOP=255
}Robot_remote_sign;

#define ROBOT_TASK1_EXTEND_GRIPPER_WAIT_TIME 300 //夹爪伸出后等待时间，单位10ms
#define ROBOT_TASK1_CLOSE_GRIPPER_WAIT_TIME 100 //夹爪闭合后等待时间，单位10ms
#define ROBOT_TASK1_TOWER_UP_WAIT_TIME 150 //塔吊上升后等待时间，单位10ms

#define ROBOT_TASK1_BACK_TIME1 9 //位置1的后退时间，单位10ms
#define ROBOT_TASK1_BACK_TIME2 137 //位置2的后退时间，单位10ms
#define ROBOT_TASK1_BACK_TIME3 240 //位置3的后退时间，单位10ms

#define ROBOT_TASK1_BUCKET_DMG_ANGLE 280 //排爆桶上方DMG6220目标角度
#define ROBOT_TASK1_BUCKET_DMG_WAIT_TIME 300 //DMG转到排爆桶后的等待时间，单位10ms
#define ROBOT_TASK1_OPEN_GRIPPER_WAIT_TIME 100 //打开夹爪后等待时间，单位10ms
#define ROBOT_TASK1_DMG_190_ANGLE 190 //关闭夹爪后的DMG6220目标角度
#define ROBOT_TASK1_DMG_190_WAIT_TIME 80 //DMG转到190后的等待时间，单位10ms
#define ROBOT_TASK1_DMG_100_ANGLE 100 //最后的DMG6220目标角度
#define ROBOT_TASK1_DMG_100_WAIT_TIME 200 //DMG转到100后的等待时间，单位10ms

#define ROBOT_ROAD_PULSE1 15000 //任务二结束的第一条直道的计里程
#define ROBOT_ROAD_PULSE2 32300 //任务二结束的第二条直道的计里程

typedef enum{
    ROBOT_STAGE_WAIT_FOR_START=0,
    ROBOT_STAGE_START,
    ROBOT_STAGE_SCAN_QR,
    ROBOT_STAGE_QR_WAIT,
    ROBOT_STAGE_QR_FORWARD,
    ROBOT_STAGE_TURN1,
    ROBOT_STAGE_TURN1_Y_ALIGN,
    ROBOT_STAGE_STRAGE1,
    ROBOT_STAGE_THROUGH_OBSTACLE,
    ROBOT_STAGE_THROUGH_OBSTACLE_WAIT,
    ROBOT_STAGE_TURN2,
    ROBOT_STAGE_STRAGE2,
    ROBOT_STAGE_TURN3,
    ROBOT_STAGE_STRAGE3,
    ROBOT_STAGE_STRAGE3_FORWARD1,
    ROBOT_STAGE_STRAGE3_Y_ALIGN,
    ROBOT_STAGE_TURN4,
    ROBOT_STAGE_STRAGE4,
    ROBOT_STAGE_TASK1,
    ROBOT_STAGE_TASK1_EXTEND_GRIPPER,
    ROBOT_STAGE_TASK1_CLOSE_GRIPPER,
    ROBOT_STAGE_TASK1_CLOSE_GRIPPER_WAIT,
    ROBOT_STAGE_TASK1_TOWER_UP,
    ROBOT_STAGE_TASK1_BACK_BY_POSITION,
    ROBOT_STAGE_TASK1_DMG_TO_BUCKET,
    ROBOT_STAGE_TASK1_DMG_TO_BUCKET_WAIT,
    ROBOT_STAGE_TASK1_OPEN_GRIPPER_WAIT,
    ROBOT_STAGE_TASK1_DMG_190_WAIT,
    ROBOT_STAGE_TASK1_DMG_100_WAIT,
    ROBOT_STAGE_TASK1_DONE,
    ROBOT_STAGE_TASK2,
    ROBOT_STAGE_TASK2_DONE,
    ROBOT_STAGE_TO_TASK3_STRAIGHT1,
    ROBOT_STAGE_TO_TASK3_STRAIGHT2,
    ROBOT_STAGE_TASK3,
    ROBOT_STAGE_TASK3_OFFSET_FORWARD,
    ROBOT_STAGE_TASK3_DMG_125_WAIT,
    ROBOT_STAGE_TASK3_CLAW_OUT3_WAIT,
    ROBOT_STAGE_TASK3_CLOSE_GRIPPER_WAIT,
    ROBOT_STAGE_TASK3_CLAW_IN_WAIT,
    ROBOT_STAGE_TASK3_TO_END,
    ROBOT_STAGE_END
}Robot_stage;

typedef enum{
    ROBOT_DIFFERENTIAL_TURN_IDLE=0,
    ROBOT_DIFFERENTIAL_TURN_RUNNING,
    ROBOT_DIFFERENTIAL_TURN_DONE
}Robot_differential_turn_state;

void Robot_set_move(int16_t vx,int16_t vy,int16_t omega);
void Robot_set_vx(int16_t vx);
void Robot_set_vy(int16_t vy);
void Robot_set_omega(int16_t omega);
void Robot_Angle_Update(hwt101_csx *hwt101, PIDController *pid, uint16_t target_angle);
void Robot_stop_move(void);
void Robot_remote_control(void);
void Robot_tick(void);
void Robot_Claw_open(void);
void Robot_Claw_close(void);
void Robot_Tower_down(void);
void Robot_Tower_middle(void);
void Robot_Tower_up(void);
void Robot_Claw_out1(void);
void Robot_Claw_in(void);
void Robot_Claw_out2(void);
void Robot_Claw_out3(void);
uint8_t Robot_is_turning(hwt101_csx *hwt101, float target_angle);

/**
 * @brief 开始一次差速转弯
 * @param radius_cm 转弯半径，单位 cm，0 表示原地差速旋转
 * @param target_angle_deg 目标绝对航向角，函数自动选择最短转向方向
 * @return 0 成功，1 陀螺仪数据无效，2 参数无效
 */
uint8_t Robot_start_differential_turn(float radius_cm, float target_angle_deg);

/**
 * @brief 开始一次差速转弯，并指定前进方向线速度命令
 * @param linear_speed 非负的前进速度命令，内部会自动限幅
 */
uint8_t Robot_start_differential_turn_ex(float radius_cm,
                                          float target_angle_deg,
                                          float linear_speed);

void Robot_stop_differential_turn(void);
uint8_t Robot_is_differential_turning(void);
#endif
