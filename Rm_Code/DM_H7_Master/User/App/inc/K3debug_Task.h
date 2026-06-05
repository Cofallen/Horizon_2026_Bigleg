#ifndef __K3DEBUG_TASK_
#define __K3DEBUG_TASK_

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "RUI_DBUS.h"
#include "MY_define.h"
#include "Motors.h"
#include "NIMING.h"

void k3debug_init(MOTOR_Typedef *MOTOR);

void k3debug_task(	MOTOR_Typedef *MOTOR,
					DBUS_Typedef *DBUS);

#endif
