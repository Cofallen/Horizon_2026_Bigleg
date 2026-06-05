#pragma once

#include "main.h"
#include "All_Init.h"

typedef struct
{
    uint8_t target_visible;

} Vision_Data_t;

typedef struct
{
    float single_angle;

    float target_angle;

    uint8_t last_mouse_l;

    uint8_t last_switch;

    uint8_t auto_aim_enable;

    uint16_t mouse_press_cnt;

    uint8_t single_lock; 

    uint16_t jam_cnt;

    float last_angle;
    float accum_angle;

} Shooter_Control_t;

typedef struct
{
    Shooter_Control_t ctrl;

    MOTOR_Typedef *motor;

    DBUS_Typedef *dbus;

    Vision_Data_t *vision;

} Shooter_t;

extern Shooter_t shooter;

void Shooter_Init(
    Shooter_t *s,
    MOTOR_Typedef *motor,
    Vision_Data_t *vision);

void Shooter_Update(Shooter_t *s, DBUS_Typedef *dbus, float dt);
uint8_t Shooter_IsFired(Shooter_t *s, MOTOR_Typedef *MOTOR);

