#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"
#include "chassisL.h"
#include "board2board.h"
#include "get_target.h"

float PID_S_RF[3] = {4.0f, 0.0f, 0.0f};
float PID_P_RF[3] = {-10.0f, 0.0f, 10.0f};
float PID_S_RB[3] = {4.0f, 0.0f, 0.0f};
float PID_P_RB[3] = {-10.0f, 0.0f, 10.0f};

void ChassisR_Init(MOTOR_Typedef *motor, Leg_Typedef *object)
{   
    ALL_MOTOR.right_front.DATA.pos_init_rad = 0.447538912f;
    ALL_MOTOR.right_back.DATA.pos_init_rad  = -2.86950278f;
    ALL_MOTOR.right_wheel.DATA.Angle_Init = ALL_MOTOR.right_wheel.DATA.Angle_Infinite;
    PID_Init(&motor->right_front.PID_P, 6.0f, 0.1f, PID_P_RF,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->right_front.PID_S, 9.0f, 0.1f, PID_S_RF,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->right_back.PID_P, 6.0f, 0.1f, PID_P_RB,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    PID_Init(&motor->right_back.PID_S, 9.0f, 0.1f, PID_S_RB,
              0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
}

void ChassisR_UpdateState(Leg_Typedef *object, MOTOR_Typedef *motor, IMU_Data_t *imu, float dt)
{
    // 更新状态 
    object->stateSpace.theta = (-PI / 2.0f + object->vmc_calc.phi0[POS] + imu->pitch /  57.3f);
    object->stateSpace.dtheta = Discreteness_Diff(&object->Discreteness.Theta, object->stateSpace.theta, dt);
    object->stateSpace.phi = -imu->pitch / 57.3f;
    object->stateSpace.dphi = -imu->gyro[0];  
    // object->stateSpace.dphi = Discreteness_Diff(&object->Discreteness.Phi, object->stateSpace.phi, dt);

    object->stateSpace.ddtheta = Discreteness_Diff(&object->Discreteness.dTheta, object->stateSpace.dtheta, dt);
}



void ChassisR_Control(Leg_Typedef *object, DBUS_Typedef *dbus, IMU_Data_t *imu, float dt)
{   
    // 目标值获取应加上滤波 重写一个函数
    Chassis_Get_target(object, dbus, imu, dt);

    object->LQR.T_w = (object->LQR.K[0] * (object->stateSpace.theta - object->target.theta) +
                     object->LQR.K[1] * (object->stateSpace.dtheta - object->target.dtheta) +
                     object->LQR.K[2] * (object->stateSpace.s - object->target.s) +
                     object->LQR.K[3] * (object->stateSpace.dot_s - object->target.dot_s) +
                     object->LQR.K[4] * (object->stateSpace.phi - object->target.phi) +
                     object->LQR.K[5] * (object->stateSpace.dphi - object->target.dphi));

    object->LQR.T_p = (object->LQR.K[6] * (object->stateSpace.theta - object->target.theta) +
                      object->LQR.K[7] * (object->stateSpace.dtheta - object->target.dtheta) +
                      object->LQR.K[8] * (object->stateSpace.s - object->target.s) +
                      object->LQR.K[9] * (object->stateSpace.dot_s - object->target.dot_s) +
                      object->LQR.K[10] * (object->stateSpace.phi - object->target.phi) +
                      object->LQR.K[11] * (object->stateSpace.dphi - object->target.dphi));

    // PID_Calculate(&object->pid.F0_l_x, object->vmc_calc.L0[POS], object->target.l0);
    PID_calc(&object->pid.F0_l_p, object->vmc_calc.L0[POS], object->target.l0);
    PID_calc(&object->pid.F0_l_s, object->vmc_calc.L0[VEL], object->pid.F0_l_p.out);
    object->LQR.dF_0 = object->pid.F0_l_s.out;

    PID_calc(&object->pid.Roll, imu->roll / 57.3f, object->target.roll);
    object->LQR.dF_roll = -object->pid.Roll.out;

    PID_calc(&object->pid.Delta, object->LQR.delta, object->target.d2theta);
    object->LQR.dF_delta = object->pid.Delta.out;

    PID_calc(&object->pid.Yaw, imu->YawTotalAngle / 57.3f, object->target.yaw);
    object->LQR.dF_yaw = object->pid.Yaw.out;

    // if (object->status.jump == 3 || object->status.jump == 4)
    //   object->LQR.F_0 = object->LQR.dF_0 + object->LQR.dF_roll; 
    // else 
      object->LQR.F_0 = (MASS_BODY / 2.0f * 9.81f / arm_cos_f32(object->stateSpace.theta) + object->LQR.dF_0 + object->LQR.dF_roll) - object->vmc_calc.Fv;
    // object->LQR.F_0 = object->LQR.dF_0 - object->LQR.dF_roll;

    // pid修正
    object->LQR.T_p = object->LQR.T_p - object->LQR.dF_delta;
    object->LQR.T_w = object->LQR.T_w - object->LQR.dF_yaw;

    // // 测试补偿
    // object->LQR.F_0 = - object->vmc_calc.Fv;
    // object->LQR.T_p = 0.0f;

    object->LQR.torque_setT[0] = object->vmc_calc.JRM[0][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[0][1] * object->LQR.T_p;
    object->LQR.torque_setT[1] = object->vmc_calc.JRM[1][0] * object->LQR.F_0 + \
                                 object->vmc_calc.JRM[1][1] * object->LQR.T_p;
    object->LQR.torque_setW  = object->LQR.T_w * kr;

    object->LQR.torque_get_F_0 =  (object->vmc_calc.JRM_inv[0][0] * ALL_MOTOR.left_front.DATA.IQ + \
                                   object->vmc_calc.JRM_inv[0][1] * ALL_MOTOR.left_back.DATA.IQ);
    object->LQR.torque_get_T_p =   object->vmc_calc.JRM_inv[1][0] * ALL_MOTOR.left_front.DATA.IQ + \
                                   object->vmc_calc.JRM_inv[1][1] * ALL_MOTOR.left_back.DATA.IQ;

    // 限幅
    // (object->LQR.torque_setT[0] > object->limit.T_max) ? (object->LQR.torque_setT[0] = object->limit.T_max) : (object->LQR.torque_setT[0] < -object->limit.T_max) ? (object->LQR.torque_setT[0] = -object->limit.T_max) : 0;
    // (object->LQR.torque_setT[1] > object->limit.T_max) ? (object->LQR.torque_setT[1] = object->limit.T_max) : (object->LQR.torque_setT[1] < -object->limit.T_max) ? (object->LQR.torque_setT[1] = -object->limit.T_max) : 0;
    // (object->LQR.torque_setW > object->limit.W_max) ? (object->LQR.torque_setW = object->limit.W_max) : (object->LQR.torque_setW < -object->limit.W_max) ? (object->LQR.torque_setW = -object->limit.W_max) : 0;
    // if (dbus->Remote.S2_u8 == 1)
    // {
    //   object->LQR.torque_setT[0] = 0.0f;
    //   object->LQR.torque_setT[1] = 0.0f;
    //   object->LQR.torque_setW = 0.0f;
    // }

    // (object->LQR.torque_setT[0] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[0] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[0] = -MAX_TORQUE_LEG_T) : 0;
    // (object->LQR.torque_setT[1] > MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = MAX_TORQUE_LEG_T) : (object->LQR.torque_setT[1] < -MAX_TORQUE_LEG_T) ? (object->LQR.torque_setT[1] = -MAX_TORQUE_LEG_T) : 0;
    // (object->LQR.torque_setW > MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = MAX_TORQUE_LEG_W) : (object->LQR.torque_setW < -MAX_TORQUE_LEG_W) ? (object->LQR.torque_setW = -MAX_TORQUE_LEG_W) : 0;
}