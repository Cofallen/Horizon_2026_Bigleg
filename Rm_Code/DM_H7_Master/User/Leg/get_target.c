#include "get_target.h"
#include "main.h"
#include "board2board.h"
#include "All_Init.h"
#include "vmc.h"

float yaw_process = 0.0f;

void Chassis_Get_target(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt)
{
    // object->target.theta = (float)dbus->Remote.CH1_int16 / 660.0f * 1.0f;
    object->target.theta = 0.0f;
    object->target.dtheta = 0.0f;
    object->target.dot_s = (float)dbus->Remote.CH1_int16 / 660.0f * 2.0f + (float)(dbus->KeyBoard.W - dbus->KeyBoard.S) * 0.3f;
    object->target.s = Discreteness_Sum(&object->Discreteness.target_s, object->target.dot_s, dt);
    object->target.phi = 0.0f;
    object->target.dphi = 0.0f;
    object->target.yaw -= ((float)dbus->Remote.CH2_int16 / 660000.0f * 4.0f - (float)(dbus->KeyBoard.A - dbus->KeyBoard.D) * 0.005f + (float)(dbus->Mouse.X_Flt * 0.002f));
    
    // object->target.yaw = -((float)dbus->Remote.CH2_int16 / 660.0f * 2.0f) ;
    // object->target.yaw = -((float)dbus->Remote.CH2_int16 / 660.0f * 4.0f) + PID_calc(&pid_follow, outppp, 0.0f);
    // yaw_process = -boardRxData.dataNeaten.yaw_imu / 57.3f;
    // if (IMU_Data.yaw - yaw_process < -3.14f)
    // {
    //     yaw_process += 3.14f;
    // }
    // else if (IMU_Data.yaw - yaw_process > 3.14f)
    // {
    //     yaw_process -= 3.14f;
    // }

    // // 切换 小陀螺/跟随
    // (dbus->Remote.Dial_int16) ? (object->target.yaw += dbus->Remote.Dial_int16 / 660000.0f) : (object->target.yaw = yaw_process);
    // object->target.yaw = yaw_process;
    // object->target.yaw = -boardRxData.dataNeaten.yaw_imu / 57.3f;

    object->target.l0 += ((float)dbus->Remote.CH0_int16 / 660000.0f + (float)(dbus->KeyBoard.Q - dbus->KeyBoard.E) * 0.0001f);
    (object->target.l0 > MAX_LEG_LENGTH) ? (object->target.l0 = MAX_LEG_LENGTH) : (object->target.l0 < MIN_LEG_LENGTH) ? (object->target.l0 = MIN_LEG_LENGTH) : 0;
}