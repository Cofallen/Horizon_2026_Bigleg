#ifndef __GET_TARGET_H
#define __GET_TARGET_H

#include "vmc.h"
#include "All_Init.h"
#include "IMU_Task.h"

void Chassis_Get_target(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt);

#endif // !__GET_TARGET_H