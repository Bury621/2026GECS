#ifndef __ROBOT_H
#define __ROBOT_H

#include "Emm.h"
#include "DMG6220.h"

#include <hwt101.h>
#include <PID.h>

#define ROBOT_MOVE_SET_MOTOR_DELAY 5

#define ROBOT_TURNING_JUJGUE_ANGLE_DEADBAND 3 //角度误差多少以内认为旋转完成

#define ROBOT_ROAD_TIME1 135
#define ROBOT_ROAD_TIME2 140
#define ROBOT_ROAD_TIME3 190
#define ROBOT_ROAD_TIME4 175
#define ROBOT_ROAD_TIME5 193
#define ROBOT_ROAD_TIME6 140

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
    ROBOT_STAGE_TURN1,
    ROBOT_STAGE_STRAGE1,
    ROBOT_STAGE_THROUGH_OBSTACLE,
    ROBOT_STAGE_TURN2,
    ROBOT_STAGE_STRAGE2,
    ROBOT_STAGE_TURN3,
    ROBOT_STAGE_STRAGE3,
    ROBOT_STAGE_TURN4,
    ROBOT_STAGE_STRAGE4,
    ROBOT_STAGE_STRAGE5,
    ROBOT_STAGE_TASK1
}Robot_stage;

void Robot_set_move(int16_t vx,int16_t vy,int16_t omega);
void Robot_set_vx(int16_t vx);
void Robot_set_vy(int16_t vy);
void Robot_set_omega(int16_t omega);
void Robot_Angle_Update(hwt101_csx *hwt101, PIDController *pid, uint16_t target_angle);
void Robot_stop_move(void);
void Robot_remote_control(void);
void Robot_tick(void);
uint8_t Robot_is_turning(hwt101_csx *hwt101, float target_angle);
#endif
