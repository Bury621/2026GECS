#ifndef __ROBOT_H
#define __ROBOT_H

#include "Emm.h"
#include "DMG6220.h"

#define ROBOT_MOVE_SET_MOTOR_DELAY 3
void Robot_set_move(int16_t vx,int16_t vy,int16_t omega);

#endif