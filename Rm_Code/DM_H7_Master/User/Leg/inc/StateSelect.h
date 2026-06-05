#ifndef __STATESELECT_H
#define __STATESELECT_H

#include "main.h"
#include "vmc.h"
#include "All_Init.h"

void Robot_UpdateMode(Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus);
void Robot_Control(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus, float dt);
void Robot_LimitOutput(Leg_Typedef *left, Leg_Typedef *right);
void Robot_SendTorque(Leg_Typedef *left, Leg_Typedef *right);


#endif