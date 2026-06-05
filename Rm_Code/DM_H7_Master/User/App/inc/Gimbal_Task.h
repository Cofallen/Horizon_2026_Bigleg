#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "RUI_DBUS.h"
#include "MY_define.h"
#include "Monitor_Task.h"
#include "Motors.h"

uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    RUI_ROOT_STATUS_Typedef *Root,
                    MOTOR_Typedef *MOTOR);
#endif
