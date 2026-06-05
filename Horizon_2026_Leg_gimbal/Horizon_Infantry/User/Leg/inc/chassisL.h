#ifndef __CHASSISL_H
#define __CHASSISL_H

#include "vmc.h"
#include "RUI_DBUS.h"

void Chassis_SendTorque();

void DM_send_iq(CAN_HandleTypeDef *hcan, uint16_t motor_id, float torque);

#endif // !__CHASSISL_H