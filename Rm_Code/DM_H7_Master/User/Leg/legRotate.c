#include "legRotate.h"
#include "arm_math.h"
#include "VOFA.h"
#include "RecoveryControl.h"

LegRotate_t LegRotate;

static const float PID_POS[3] = {200.0f, 0.0f, 10000.0f};
static const float PID_VEL[3] = {0.05f, 0.0f, 0.0f};
uint8_t pitch_recovery_flag = 0;

#define LEG_ROTATE_TARGET     1.2f          // 单次目标位置（不含圈数）
#define LEG_ROTATE_SPEED      1.5f          // 追踪速度
#define LEG_ROTATE_TORQUE_MAX 3.0f          // 最大限幅
#define LEG_TARGET_DIFF       2.0f          // 目标差多少开启下次规划
static inline void limit(float *x, float max)
{
    if(*x > max)
        *x = max;

    if(*x < -max)
        *x = -max;
}

static float Leg_Unwrap(float theta,
                        float *last,
                        int32_t *round)
{
    float diff;

    diff = theta - *last;

    if(diff > PI)
    {
        (*round)--;
    }
    else if(diff < -PI)
    {
        (*round)++;
    }

    *last = theta;

    return theta + 2.0f * PI * (*round);
}

static float GetLeftTarget(float theta_now)
{
    float target;
    int32_t k;

    k = (int32_t)((theta_now - LEG_ROTATE_TARGET)
        / (2.0f * PI));

    target =
        LEG_ROTATE_TARGET +
        2.0f * PI * k;

    if(target - LEG_TARGET_DIFF > theta_now)
    {
        target -= 2.0f * PI;
    }

    return target;
}

static float GetRightTarget(float theta_now)
{
    float target;
    int32_t k;

    k = (int32_t)((theta_now - LEG_ROTATE_TARGET)
        / (2.0f * PI));

    target = LEG_ROTATE_TARGET + 2.0f * PI * k;

    if(target - LEG_TARGET_DIFF > theta_now)
    {
        target -= 2.0f * PI;
    }

    return target;
}

static float Traj_Update(float ref,
                         float target,
                         float dt)
{
    float err;
    float step;

    err = target - ref;

    step = LEG_ROTATE_SPEED * dt;

    limit(&err, step);

    return ref + err;
}

void LegRotate_Init(void)
{
    PID_init(&LegRotate.l_pos,
             PID_POSITION,
             PID_POS,
             100.0f,
             0.2f);

    PID_init(&LegRotate.l_vel,
             PID_POSITION,
             PID_VEL,
             3.0f,
             0.0f);

    PID_init(&LegRotate.r_pos,
             PID_POSITION,
             PID_POS,
             100.0f,
             0.2f);

    PID_init(&LegRotate.r_vel,
             PID_POSITION,
             PID_VEL,
             4.0f,
             0.0f);

    LegRotate.theta_ref_l = LEG_ROTATE_TARGET;
    LegRotate.theta_ref_r = LEG_ROTATE_TARGET;
}

void LegRotate_UpdateTheta(Leg_Typedef *left,
                           Leg_Typedef *right)
{
    LegRotate.theta_l =
        Leg_Unwrap(left->stateSpace.theta,
                   &LegRotate.theta_l_last,
                   &LegRotate.round_l);

    LegRotate.theta_r =
        Leg_Unwrap(right->stateSpace.theta,
                   &LegRotate.theta_r_last,
                   &LegRotate.round_r);
}

void LegRotate_Reset(void)
{
    LegRotate.theta_ref_l = LegRotate.theta_l;
    LegRotate.theta_ref_r = LegRotate.theta_r;

    PID_clear(&LegRotate.l_pos);
    PID_clear(&LegRotate.l_vel);

    PID_clear(&LegRotate.r_pos);
    PID_clear(&LegRotate.r_vel);
}

void LegRotate_Control(MOTOR_Typedef *motor,
                       Leg_Typedef *left,
                       Leg_Typedef *right,
                       float dt)
{

    float vel_ref_l;
    float vel_ref_r;

    LegRotate.target_l_final =
        GetLeftTarget(LegRotate.theta_l);

    LegRotate.target_r_final =
        GetRightTarget(LegRotate.theta_r);

    LegRotate.theta_ref_l = 
        Traj_Update(LegRotate.theta_ref_l,
                    LegRotate.target_l_final,
                    dt);

    LegRotate.theta_ref_r = 
        Traj_Update(LegRotate.theta_ref_r,
                    LegRotate.target_r_final,
                    dt);
                    
    vel_ref_l =
        PID_calc(&LegRotate.l_pos,
                LegRotate.theta_l,
                LegRotate.theta_ref_l);

    vel_ref_r =
        PID_calc(&LegRotate.r_pos,
                LegRotate.theta_r,
                LegRotate.theta_ref_r);

    left->LQR.torque_setT[0] =
        -PID_calc(&LegRotate.l_vel,
                 -motor->left_front.DATA.vel,
                 vel_ref_l);

    left->LQR.torque_setT[1] =
        -PID_calc(&LegRotate.l_vel,
                 -motor->left_back.DATA.vel,
                 vel_ref_l);

    right->LQR.torque_setT[0] =
        -PID_calc(&LegRotate.r_vel,
                 motor->right_front.DATA.vel,
                 vel_ref_r);

    right->LQR.torque_setT[1] =
        -PID_calc(&LegRotate.r_vel,
                 motor->right_back.DATA.vel,
                 vel_ref_r);

    limit(&left->LQR.torque_setT[0],
           LEG_ROTATE_TORQUE_MAX);

    limit(&left->LQR.torque_setT[1],
           LEG_ROTATE_TORQUE_MAX);

    limit(&right->LQR.torque_setT[0],
           LEG_ROTATE_TORQUE_MAX);

    limit(&right->LQR.torque_setT[1],
           LEG_ROTATE_TORQUE_MAX);
    
    if(pitch_recovery_flag)
    {
        limit(&left->LQR.torque_setT[0],
            0);

        limit(&left->LQR.torque_setT[1],
            0);

        limit(&right->LQR.torque_setT[0],
            0);

        limit(&right->LQR.torque_setT[1],
            0);
    }

    left->LQR.torque_setW = 0.0f;
    right->LQR.torque_setW = 0.0f;

    RollRecovery_Control(motor,
                         left,
                         right);
    PitchRecovery_Control(motor,
                           left,
                           right);         

    // VOFA_justfloat(
    //     LegRotate.theta_l,
    //     LegRotate.theta_ref_l,
    //     LegRotate.target_l_final ,

    //     LegRotate.theta_r,
    //     LegRotate.theta_ref_r,
    //     LegRotate.target_r_final ,

    //     left->LQR.torque_setT[0],
    //     right->LQR.torque_setT[0],

    //     left->LQR.torque_setT[1],
    //     right->LQR.torque_setT[1]);
}