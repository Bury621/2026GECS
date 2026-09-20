#ifndef __ROBOT_H
#define __ROBOT_H

#include "Emm.h"
#include "DMG6220.h"

#define ROBOT_MOVE_SET_MOTOR_DELAY 5

typedef enum{
    ROBOT_REMOTE_STOP=0,
    ROBOT_REMOTE_FORWARD,
    ROBOT_REMOTE_BACKWARD,
    ROBOT_REMOTE_LEFT,
    ROBOT_REMOTE_RIGHT,
    ROBOT_REMOTE_TO_STOP=255
}Robot_remote_sign;
void Robot_set_move(int16_t vx,int16_t vy,int16_t omega);
void Robot_stop_move(void);
void Robot_remote_control();

#endif