/* legRotate.h */

#ifndef __LEG_ROTATE_H
#define __LEG_ROTATE_H

#include "main.h"
#include "pid_temp.h"
#include "vmc.h"

typedef struct
{
    float q0;
    float q1;

    float T;
    float t;

    uint8_t finish;

} Traj_t;

typedef struct
{
    pid_type_def l_pos;
    pid_type_def l_vel;

    pid_type_def r_pos;
    pid_type_def r_vel;

    Traj_t traj_l;
    Traj_t traj_r;

    float theta_l;
    float theta_r;

    float theta_l_last;
    float theta_r_last;

    int32_t round_l;
    int32_t round_r;

    float theta_ref_l;
    float theta_ref_r;

    float target_l_final;
    float target_r_final;
} LegRotate_t;

extern LegRotate_t LegRotate;
extern uint8_t pitch_recovery_flag;

void LegRotate_Init(void);

void LegRotate_UpdateTheta(Leg_Typedef *left,
                           Leg_Typedef *right);

void LegRotate_Start(Leg_Typedef *left,
                     Leg_Typedef *right);

void LegRotate_Control(MOTOR_Typedef *motor,
                       Leg_Typedef *left,
                       Leg_Typedef *right,
                       float dt);

void LegRotate_Reset(void);

#endif