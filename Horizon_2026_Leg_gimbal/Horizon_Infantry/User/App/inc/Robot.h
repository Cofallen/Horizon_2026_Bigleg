#ifndef __ROBOT_H
#define __ROBOT_H

#include "RUI_DBUS.h"
#include "Motors.h"
#include "Power_CAP.h"
#include "Vision.h"

void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               VisionRxDataUnion *Vision);

#endif
