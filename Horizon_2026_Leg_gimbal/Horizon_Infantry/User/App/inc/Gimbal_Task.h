#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "IMU_Task.h"

uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    MOTOR_Typedef *MOTOR,
                    IMU_Data_t *IMU);
uint8_t Gimbal_AIM_INIT();
uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typedef *motor);

#endif
