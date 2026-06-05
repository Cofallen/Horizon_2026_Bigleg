#include "Robot.h"

void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               VisionRxDataUnion *Vision)
{
    switch (mode) {

        case 1://底盘
        {

        } break;
        case 2://云台
        {
            switch (DBUS->Remote.S1_u8 )
            {
            case 1:
            case 2:
            case 3:
            {
                CONTAL->HEAD.Pitch -= (float) ((DBUS->Remote.CH3_int16)) * 0.0003f;
                CONTAL->HEAD.Pitch = (CONTAL->HEAD.Pitch > CONTAL->HEAD.Pitch_MAX) ? CONTAL->HEAD.Pitch_MAX : CONTAL->HEAD.Pitch;
                CONTAL->HEAD.Pitch = (CONTAL->HEAD.Pitch < CONTAL->HEAD.Pitch_MIN) ? CONTAL->HEAD.Pitch_MIN : CONTAL->HEAD.Pitch;

                CONTAL->HEAD.Yaw -= (float) (DBUS->Remote.CH2_int16) * 0.0006f;
            }
                break;
            // case 3:
            // {
            //     if (VisionRxData.Data.isOnline)
            //     {
            //         CONTAL->HEAD.Pitch = VisionRxData.Data.PitchAngle/360.0f*8192.0f;
            //         CONTAL->HEAD.Yaw   = VisionRxData.Data.YawAngle/360.0f*8192.0f;
            //     }
            //     else
            //     {
            //         CONTAL->HEAD.Pitch = (float)ALL_MOTOR.M6020[PITCH].DATA.Angle_Infinite;
            //         CONTAL->HEAD.Yaw   = (float)ALL_MOTOR.M6020[YAW].DATA.Angle_Infinite;
            //     }
            //     CONTAL->HEAD.Pitch = (CONTAL->HEAD.Pitch > CONTAL->HEAD.Pitch_MAX) ? CONTAL->HEAD.Pitch_MAX : CONTAL->HEAD.Pitch;
            //     CONTAL->HEAD.Pitch = (CONTAL->HEAD.Pitch < CONTAL->HEAD.Pitch_MIN) ? CONTAL->HEAD.Pitch_MIN : CONTAL->HEAD.Pitch;
            // }
            // break;
            default:
                break;
            }

        } break;

        case 3://电容(待修改)
        {

        } break;

        case 4://发射
        {

        } break;
    }
}