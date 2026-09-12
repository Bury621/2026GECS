#ifndef __DMG6220_H
#define __DMG6620_H

#include "main.h"


typedef struct{
    CAN_HandleTypeDef csx;
    
}dm_csx;
void DM_Init(dm_csx csx);
#endif