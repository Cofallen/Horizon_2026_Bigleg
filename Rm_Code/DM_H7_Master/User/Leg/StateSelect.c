#include "StateSelect.h"
#include "chassisL.h"
#include "chassisR.h"
#include "MY_Define.h"
#include "DM_Motor.h"
#include "controller.h"
#include "All_Init.h"
#include "vmc.h"
#include "get_K.h"
#include "pid_temp.h"
#include "BM_Motor.h"
#include "observe.h"
#include "board2board.h"
#include "get_target.h"
#include "KNN.h"
#include "legRotate.h"

// float w[] = {8.23521264, 8.69834965, 1.78259253, 0.16771652, 1.07922558, 0.03220763, -0.53748833, 0.42575712, -0.58027362};
// float b = -6.20173577;
// float mean[] = {-30.41808307, -12.34488136, 0.00315275, -0.00464040, 1.35854588, -2.00639961, -0.01038345, 0.92982620, 9.67937703};
// float std[] = {94.37885327, 19.92452379, 0.38786233, 1.16555753, 5.68133384, 239.56580740, 0.33913288, 0.14331179, 2.84528473};
// 离地状态
// left->status.offGround = ground_check(&Leg_l, &IMU_Data, w, b, mean, std);
// right->status.offGround = ground_check(&Leg_r, &IMU_Data, w, b, mean, std);
// left->status.offGround = ground_check(&Leg_l, &IMU_Data);
// right->status.offGround = ground_check(&Leg_r, &IMU_Data);
    float theta_diff;
    float theta_max;
void Robot_UpdateMode(Leg_Typedef *left,
                      Leg_Typedef *right,
                      DBUS_Typedef *dbus)
{
    float theta_l;
    float theta_r;

    float theta_avg;
    // float theta_diff;
    // float theta_max;

    float pitch;
    float pitch_rate;

    float L0_avg;

    theta_l = left->stateSpace.theta;
    theta_r = right->stateSpace.theta;

    theta_avg =
        0.5f *
        (theta_l + theta_r);

    theta_diff =
        fabsf(theta_l - theta_r);

    theta_max =
        MAX(fabsf(theta_l),
            fabsf(theta_r));

    pitch =
        IMU_Data.pitch / 57.3f;

    pitch_rate =
        IMU_Data.gyro[1];

    L0_avg =
        0.5f *
        (left->vmc_calc.L0[POS] +
         right->vmc_calc.L0[POS]);
//    VOFA_justfloat(theta_l, theta_r, theta_avg, theta_diff, theta_max, pitch, pitch_rate,  L0_avg,
//                        0, RobotManager.mode);
    if(dbus->Remote.S2_u8 == 1)
    {
        RobotManager.mode = ROBOT_DISABLE;

        RobotManager.fallen_count = 0;
        RobotManager.rising_count = 0;
        RobotManager.step_count = 0;
        RobotManager.transition_count = 0;

        return;
    }

    switch(RobotManager.mode)
    {
        case ROBOT_DISABLE:

            RobotManager.mode = ROBOT_BALANCE;

        break;

        case ROBOT_BALANCE:

            if(theta_max  > 0.8f ||
               theta_diff > 1.80f ||
               fabsf(pitch) > 0.30f)
            {
                RobotManager.fallen_count++;

                if(RobotManager.fallen_count > 1)
                {
                    LegRotate_Reset();      // 设定轨迹初始

                    RobotManager.mode = ROBOT_FALLEN;

                    RobotManager.fallen_count = 0;
                }
            }
            else
            {
                RobotManager.fallen_count = 0;
            }

            if(L0_avg > 0.34f &&
               theta_avg > 0.80f &&
               theta_diff < 0.50f)
            {
                RobotManager.step_count++;

                if(RobotManager.step_count > 100)
                {
                    RobotManager.mode = ROBOT_STEP;

                    RobotManager.step_count = 0;
                }
            }
            else
            {
                RobotManager.step_count = 0;
            }

        break;

        case ROBOT_FALLEN:
            
            if(theta_diff < 0.40f &&
               theta_max  > 1.00f &&
               theta_max  < 1.35f)
            {
                RobotManager.rising_count++;

                if(RobotManager.rising_count > 400)
                {
                    RobotManager.mode = ROBOT_RISING;

                    RobotManager.rising_count = 0;
                }
            }
            else
            {
                RobotManager.rising_count = 0;
            }

        break;

        case ROBOT_RISING:

            if(theta_max  > 1.45f ||
               theta_diff > 1.80f ||
               fabsf(pitch) > 0.90f)
            {
                RobotManager.mode = ROBOT_FALLEN;

                RobotManager.rising_count = 0;

                break;
            }

            if(theta_max  < 1.15f &&
               theta_diff < 0.40f)
            {
                RobotManager.rising_count++;

                if(RobotManager.rising_count > 15)
                {
                    RobotManager.mode =
                        ROBOT_TRANSITION;

                    RobotManager.rising_count = 0;
                }
            }
            else
            {
                RobotManager.rising_count = 0;
            }

        break;

        case ROBOT_TRANSITION:

            if(theta_max  > 1.45f ||
               theta_diff > 1.80f)
            {
                RobotManager.mode = ROBOT_FALLEN;

                RobotManager.transition_count = 0;

                break;
            }

            RobotManager.transition_count++;

            if(RobotManager.transition_count > 300)
            {
                RobotManager.mode = ROBOT_BALANCE;

                RobotManager.transition_count = 0;
            }

        break;

        case ROBOT_STEP:

            if(theta_diff > 1.80f ||
               theta_max  > 1.40f)
            {
                RobotManager.mode = ROBOT_FALLEN;
            }

        break;
    }
}

void Robot_Control(MOTOR_Typedef *motor, Leg_Typedef *left, Leg_Typedef *right, DBUS_Typedef *dbus, float dt)
{
    switch(RobotManager.mode)
    {
        case ROBOT_DISABLE:

            left->LQR.torque_setT[0] = 0.0f;
            left->LQR.torque_setT[1] = 0.0f;
            left->LQR.torque_setW    = 0.0f;

            right->LQR.torque_setT[0] = 0.0f;
            right->LQR.torque_setT[1] = 0.0f;
            right->LQR.torque_setW    = 0.0f;

        break;

        case ROBOT_BALANCE:

            Chassis_Fit_K(ChassisL_LQR_K_coeffs, left->vmc_calc.L0[POS], left->LQR.K);

            Chassis_Fit_K(ChassisR_LQR_K_coeffs, right->vmc_calc.L0[POS], right->LQR.K);

            ChassisL_Control(left, dbus, &IMU_Data, dt);

            ChassisR_Control(right, dbus, &IMU_Data, dt);

        break;

        case ROBOT_FALLEN:

            // LegRotate_UpdateTheta(left, right);

            LegRotate_Control(motor,
                            left,
                            right,
                            dt);

        break;

        case ROBOT_RISING:

            Chassis_Fit_K(ChassisL_LQR_K_rising, left->vmc_calc.L0[POS], left->LQR.K);

            Chassis_Fit_K(ChassisR_LQR_K_rising, right->vmc_calc.L0[POS], right->LQR.K);

            left->target.l0  = MIN_LEG_LENGTH;
            right->target.l0 = MIN_LEG_LENGTH;

            ChassisL_Control(left, dbus, &IMU_Data, dt);

            ChassisR_Control(right, dbus, &IMU_Data, dt);

        break;

        case ROBOT_TRANSITION:

            Chassis_Fit_K(ChassisL_LQR_K_coeffs,
                          left->vmc_calc.L0[POS],
                          left->LQR.K);

            Chassis_Fit_K(ChassisR_LQR_K_coeffs,
                          right->vmc_calc.L0[POS],
                          right->LQR.K);

            ChassisL_Control(left, dbus, &IMU_Data, dt);

            ChassisR_Control(right, dbus, &IMU_Data, dt);

        break;

        case ROBOT_STEP:

            left->LQR.torque_setT[0] = motor->left_front.PID_S.Output;

            left->LQR.torque_setT[1] = motor->left_back.PID_S.Output;

            right->LQR.torque_setT[0] = motor->right_front.PID_S.Output;

            right->LQR.torque_setT[1] = motor->right_back.PID_S.Output;

            left->LQR.torque_setW = 0.0f;
            right->LQR.torque_setW = 0.0f;

        break;
    }
}

void Robot_LimitOutput(Leg_Typedef *left, Leg_Typedef *right)
{
    float T_max = 0.0f;
    float W_max = 0.0f;

    switch(RobotManager.mode)
    {
        case ROBOT_DISABLE:

            T_max = 0.0f;
            W_max = 0.0f;

        break;

        case ROBOT_BALANCE:

            T_max = MAX_TORQUE_LEG_T;
            W_max = MAX_TORQUE_LEG_W;   // todo max

        break;

        case ROBOT_FALLEN:

            T_max = 15.0f;
            W_max = 0.0f;

        break;

        case ROBOT_RISING:

            T_max = 8.0f;       // todo 8
            W_max = 0.0f;

        break;

        case ROBOT_TRANSITION:

            T_max = 16.0f;
            W_max = 1.0f;       // todo 3

        break;

        case ROBOT_STEP:

            T_max = 0.0f;       // todo 5
            W_max = 0.0f;

        break;
    }
    left->limit.T_max = T_max;
    left->limit.W_max = W_max;  // 查看，可删除

#define LIMIT(x,max)                    \
    do                                  \
    {                                   \
        if((x) > (max))  (x) = (max);   \
        if((x) < -(max)) (x) = -(max);  \
    }while(0)

    LIMIT(left->LQR.torque_setT[0], T_max);
    LIMIT(left->LQR.torque_setT[1], T_max);
    LIMIT(left->LQR.torque_setW,    W_max);

    LIMIT(right->LQR.torque_setT[0], T_max);
    LIMIT(right->LQR.torque_setT[1], T_max);
    LIMIT(right->LQR.torque_setW,    W_max);

    // VOFA_justfloat(left->LQR.torque_setT[0], left->LQR.torque_setT[1],right->LQR.torque_setT[0],
    //                 right->LQR.torque_setT[1], RobotManager.mode, 0,0,0,0,0);
}

void Robot_SendTorque(Leg_Typedef *left, Leg_Typedef *right)
{
    left->torque_send.T1 = -left->LQR.torque_setT[0];

    left->torque_send.T2 = -left->LQR.torque_setT[1];

    left->torque_send.Tw = left->LQR.torque_setW;

    right->torque_send.T1 = right->LQR.torque_setT[0];

    right->torque_send.T2 = right->LQR.torque_setT[1];

    right->torque_send.Tw = -right->LQR.torque_setW;
}