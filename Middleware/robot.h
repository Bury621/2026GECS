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

#define ROBOT_QR_BACK_TIME 180 //扫二维码后退时间，单位10ms
#define ROBOT_QR_WAIT_TIME 220 //扫二维码等待时间，单位10ms
#define ROBOT_QR_FORWARD_TIME 180 //扫二维码前进时间，单位10ms
#define ROBOT_OBSTACLE_TURN_WAIT_TIME 100 //穿越障碍物前转弯等待时间，单位10ms
#define ROBOT_ROAD_TIME1 103
#define ROBOT_ROAD_TIME2 55
#define ROBOT_ROAD_TIME3 268
#define ROBOT_ROAD_TIME4 75
#define ROBOT_ROAD_TIME5 116
#define ROBOT_ROAD_TIME6 80


typedef enum{
    ROBOT_REMOTE_STOP=0,
    ROBOT_REMOTE_FORWARD,
    ROBOT_REMOTE_BACKWARD,
    ROBOT_REMOTE_LEFT,
    ROBOT_REMOTE_RIGHT,
    ROBOT_REMOTE_TO_STOP=255
}Robot_remote_sign;

typedef enum{
    ROBOT_STAGE_WAIT_FOR_START=0,
    ROBOT_STAGE_START,
    ROBOT_STAGE_SCAN_QR,
    ROBOT_STAGE_QR_WAIT,
    ROBOT_STAGE_QR_FORWARD,
    ROBOT_STAGE_TURN1,
    ROBOT_STAGE_STRAGE1,
    ROBOT_STAGE_THROUGH_OBSTACLE,
    ROBOT_STAGE_THROUGH_OBSTACLE_WAIT,
    ROBOT_STAGE_TURN2,
    ROBOT_STAGE_STRAGE2,
    ROBOT_STAGE_TURN3,
    ROBOT_STAGE_STRAGE3,
    ROBOT_STAGE_TURN4,
    ROBOT_STAGE_STRAGE4,
    ROBOT_STAGE_STRAGE5,
    ROBOT_STAGE_TASK1
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
