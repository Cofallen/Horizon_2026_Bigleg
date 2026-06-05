#include "RecoveryControl.h"
#include "pid_temp.h"
#include "All_Init.h"
#include "legRotate.h"

static pid_type_def roll_pid;
static pid_type_def pitch_pid;

static const float roll_param[3] = {1.0f, 0.0f, 0.0f};
static const float pitch_param[3] = {0.01f, 0.0f, 0.0f};

void Recovery_Init(void)
{
    PID_init(&roll_pid,
             PID_POSITION,
             roll_param,
             4.0f,
             0.0f);
    PID_init(&pitch_pid,
             PID_POSITION,
             roll_param,
             15.0f,
             0.0f);
}

void RollRecovery_Control(MOTOR_Typedef *motor,
                          Leg_Typedef *left,
                          Leg_Typedef *right)
{
    float roll;

    float iq;

    float torque;

    float err_l;
    float err_r;

    roll =
        IMU_Data.roll / 57.3f;

    if(fabsf(roll) < 0.05f)
    {
        PID_clear(&roll_pid);

        return;
    }

    err_l =
        fabsf(
            LegRotate.theta_l -
            LegRotate.target_l_final 
        );

    err_r =
        fabsf(
            LegRotate.theta_r -
            LegRotate.target_r_final 
        );

    if(roll < 0.0f)
    {
        if(err_l > 0.40f)
        {
            PID_clear(&roll_pid);

            return;
        }

        iq =
            0.5f *
            (
                motor->left_front.DATA.IQ +
                motor->left_back.DATA.IQ
            );

        torque =
            -PID_calc(&roll_pid,
                     iq,
                     8.0f * roll);

        left->LQR.torque_setT[0] += torque;

        left->LQR.torque_setT[1] += torque;
    }
    else
    {
        if(err_r > 0.20f)
        {
            PID_clear(&roll_pid);

            return;
        }

        iq =
            0.5f *
            (
                motor->right_front.DATA.IQ +
                motor->right_back.DATA.IQ
            );

        torque =
            PID_calc(&roll_pid,
                     iq,
                     8.0f * roll);

        right->LQR.torque_setT[0] += torque;

        right->LQR.torque_setT[1] += torque;
    }
}

void PitchRecovery_Control(MOTOR_Typedef *motor,
                           Leg_Typedef *left,
                           Leg_Typedef *right)
{
    float pitch;
    float iq_l, iq_r;
    float torque_l, torque_r;
    float err_l, err_r;

    pitch = fabsf(IMU_Data.pitch / 57.3f);

    err_l = fabsf(LegRotate.theta_l - LegRotate.target_l_final);
    err_r = fabsf(LegRotate.theta_r - LegRotate.target_r_final);

    if(pitch < 0.50f)
    {
        PID_clear(&pitch_pid);
        pitch_recovery_flag = 0;
        return;
    }

    if(err_l > 0.5f || err_r > 0.5f)
    {
        PID_clear(&pitch_pid);
        pitch_recovery_flag = 0;
        return;
    }

    pitch_recovery_flag = 1;

    iq_l =
        0.5f * (
            motor->left_front.DATA.IQ +
            motor->left_back.DATA.IQ
        );

    iq_r =
        0.5f * (
            motor->right_front.DATA.IQ +
            motor->right_back.DATA.IQ
        );

    torque_l = PID_calc(&pitch_pid, iq_l, 8.0f * pitch);
    torque_r = PID_calc(&pitch_pid, iq_r, 8.0f * pitch);

    left->LQR.torque_setT[0]  += torque_l;
    left->LQR.torque_setT[1]  += torque_l;

    right->LQR.torque_setT[0] += torque_r;
    right->LQR.torque_setT[1] += torque_r;
}