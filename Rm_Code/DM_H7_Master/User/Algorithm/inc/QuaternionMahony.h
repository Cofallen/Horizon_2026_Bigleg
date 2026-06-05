#ifndef __QUATERNIONMAHONY_H
#define __QUATERNIONMAHONY_H

#pragma once
#include <stdint.h>

typedef struct
{
    float q0, q1, q2, q3;
    float gx, gy, gz;
    float exInt, eyInt, ezInt;
    float Kp;
    float Ki;
    float dt;

    float roll;
    float pitch;
    float yaw;

    float gyroBias[3];

    void (*Init)(float kp, float ki, float dt);
    void (*Update)(float gx, float gy, float gz, float ax, float ay, float az);
    float (*GetRoll)(void);
    float (*GetPitch)(void);
    float (*GetYaw)(void);
} MahonyAHRS_t;

extern MahonyAHRS_t Mahony;

#endif