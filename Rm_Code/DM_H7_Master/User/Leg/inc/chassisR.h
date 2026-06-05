#ifndef __CHASSISR_H
#define __CHASSISR_H

#include "vmc.h"
#include "RUI_DBUS.h"

void ChassisR_Init(MOTOR_Typedef *motor, Leg_Typedef *object);
void ChassisR_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt);
void ChassisR_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt);

#endif // !__CHASSISR_H