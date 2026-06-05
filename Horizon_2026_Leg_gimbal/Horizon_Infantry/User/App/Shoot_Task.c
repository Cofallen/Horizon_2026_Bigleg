#include "Shoot_Task.h"
#include <math.h>
#include "get_K.h"
#include "Heat_Task.h"

#define DBUS_DOWN     2
#define DBUS_UP    1
#define DBUS_MID    3

#define FRICTION_SPEED         6300.0f

#define JAM_ERROR_THRESHOLD    0.2f
#define JAM_SPEED_THRESHOLD    30.0f
#define JAM_COUNT_THRESHOLD    50

#define MOUSE_BURST_TIME       20

Shooter_t shooter;

typedef enum
{
    SHOOT_EVENT_NONE = 0,
    SHOOT_EVENT_SINGLE,
    SHOOT_EVENT_BURST

} Shoot_Event_e;

void MOTOR_PID_Shoot_INIT(MOTOR_Typedef *motor)
{
    float PID_S_Shoot_L[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_L[3] = {0.0f, 0.0f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_L.PID_S, 10000.0f, 0.0f, 
              PID_S_Shoot_L, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_L.PID_P, 1000.0f, 50.0f, 
              PID_P_Shoot_L, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    
    float PID_S_Shoot_R[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_R[3] = {0.0f, 0.0f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_R.PID_S, 10000.0f, 0.0f, 
              PID_S_Shoot_R, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_R.PID_P, 1000.0f, 50.0f, 
              PID_P_Shoot_R, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);

    float PID_S_Shoot_G[3] = {15.0f, 0.0f, 0.0f};
    float PID_P_Shoot_G[3] = {1.0f, 0.01f, 0.0f};

    PID_Init(&motor->DJI_3508_Shoot_G.PID_S, 15000.0f, 0.0f, 
              PID_S_Shoot_G, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
    PID_Init(&motor->DJI_3508_Shoot_G.PID_P, 3000.0f, 50.0f, 
              PID_P_Shoot_G, 0.0f, 0.0f, 
              0.0f, 0.0f, 0, 
              Integral_Limit);
}


static Shoot_Event_e Shooter_GetEvent(Shooter_t *s, boardRxData_t *boardRxData)
{
    uint8_t sw = s->dbus->Remote.S1_u8;

    s->ctrl.last_switch = sw;

    if (g_heat.heat <= 20.0f)
        return SHOOT_EVENT_NONE;

    if (sw == DBUS_UP || boardRxData->dataNeaten.mouseL == 1)
        return SHOOT_EVENT_SINGLE;

    if (sw == DBUS_DOWN || boardRxData->dataNeaten.mouseL == 2)
        return SHOOT_EVENT_BURST;

    if (sw == DBUS_MID || boardRxData->dataNeaten.mouseL == 0)
        return SHOOT_EVENT_NONE;
        
    return SHOOT_EVENT_NONE;
}

static uint8_t Shooter_SingleFinished(Shooter_t *s, MOTOR_Typedef *MOTOR)
{
    float err = fabsf(
        MOTOR->DJI_3508_Shoot_G.DATA.Angle_Infinite
        - s->ctrl.target_angle);

    return (err < 0.1f * s->ctrl.single_angle);
}

static void Shooter_FireSingle(Shooter_t *s)
{
    if (s->ctrl.single_lock)
        return;

    s->ctrl.target_angle -= s->ctrl.single_angle;
    s->ctrl.single_lock = 1;
}

static void Shooter_FireBurst(Shooter_t *s)
{
    s->ctrl.target_angle -= s->ctrl.single_angle * 0.01f;
}

static void Shooter_FireStop(Shooter_t *s,
                             MOTOR_Typedef *MOTOR)
{
    s->ctrl.target_angle =
        s->ctrl.target_angle;
    s->ctrl.single_lock = 0;
}


static void Shooter_Output(Shooter_t *s, MOTOR_Typedef *MOTOR)
{
    float friction = FRICTION_SPEED;

    s->motor->DJI_3508_Shoot_L.DATA.Aim = friction;

    s->motor->DJI_3508_Shoot_R.DATA.Aim = friction;

    s->motor->DJI_3508_Shoot_G.DATA.Aim =
        s->ctrl.target_angle;

    // PID
    PID_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_S, 
                   MOTOR->DJI_3508_Shoot_L.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_L.DATA.Aim);

    PID_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_S,
                   MOTOR->DJI_3508_Shoot_R.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_R.DATA.Aim);

    PID_Calculate(&MOTOR->DJI_3508_Shoot_G.PID_P, 
                   MOTOR->DJI_3508_Shoot_G.DATA.Angle_Infinite,
                   MOTOR->DJI_3508_Shoot_G.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_G.PID_S,
                   MOTOR->DJI_3508_Shoot_G.DATA.Speed_now,
                   MOTOR->DJI_3508_Shoot_G.PID_P.Output);

    DJI_Current_Ctrl(&hcan1, 0x200, 0, 0, MOTOR->DJI_3508_Shoot_G.PID_S.Output, 0);
    DJI_Current_Ctrl(&hcan2, 0x200, 0, 0, 0, 0);
}

void Shooter_Init(
    Shooter_t *s,
    MOTOR_Typedef *motor,
    Vision_Data_t *vision)
{
    s->motor = motor;

    s->vision = vision;

    s->ctrl.single_angle = 36864.0f * 2.0f;

    s->ctrl.target_angle =
        s->motor->DJI_3508_Shoot_G.DATA.Angle_Infinite;

    s->ctrl.last_mouse_l = 0;

    s->ctrl.last_switch = DBUS_DOWN;

    s->ctrl.mouse_press_cnt = 0;

    s->ctrl.auto_aim_enable = 0;

    s->ctrl.jam_cnt = 0;

    MOTOR_PID_Shoot_INIT(motor);
    Heat_Init(boardRxData.dataNeaten.robot_level);
}

void Shooter_Update(Shooter_t *s, DBUS_Typedef *dbus, float dt)
{
    s->dbus = dbus;

    Heat_Update(boardRxData.dataNeaten.robot_level, dt);  

    Shoot_Event_e event =
        Shooter_GetEvent(s, &boardRxData);

    switch (event)
    {
        case SHOOT_EVENT_SINGLE:
            Shooter_FireSingle(s);
            break;

        case SHOOT_EVENT_BURST:
            Shooter_FireBurst(s);
            break;

        case SHOOT_EVENT_NONE:
            Shooter_FireStop(s, &ALL_MOTOR);
            break;

        default:

            break;
    }

    Shooter_Output(s, &ALL_MOTOR);
}


static inline float angle_diff(float now, float last)
{
    float diff = now - last;
    return diff;
}

uint8_t Shooter_IsFired(Shooter_t *s, MOTOR_Typedef *MOTOR)
{
    float now = MOTOR->DJI_3508_Shoot_G.DATA.Angle_Infinite;

    float delta = angle_diff(now, s->ctrl.last_angle);

    s->ctrl.accum_angle += fabsf(delta);
    s->ctrl.last_angle = now;

    // 达到一个弹丸角度 => 认为发射
    if (s->ctrl.accum_angle >= s->ctrl.single_angle)
    {
        s->ctrl.accum_angle = 0;
        return 1;
    }

    return 0;
}