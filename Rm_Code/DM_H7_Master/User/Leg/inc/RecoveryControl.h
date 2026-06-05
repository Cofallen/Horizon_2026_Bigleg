#ifndef __RECOVERY_CONTROL_H
#define __RECOVERY_CONTROL_H

#include "main.h"
#include "vmc.h"
#include "DM_Motor.h"

void Recovery_Init(void);

void RollRecovery_Control(MOTOR_Typedef *motor,
                          Leg_Typedef *left,
                          Leg_Typedef *right);
void PitchRecovery_Control(MOTOR_Typedef *motor,
                           Leg_Typedef *left,
                           Leg_Typedef *right);
#endif