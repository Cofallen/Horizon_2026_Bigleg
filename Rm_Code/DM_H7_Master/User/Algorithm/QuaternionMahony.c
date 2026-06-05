#include "QuaternionMahony.h"
#include <math.h>

static MahonyAHRS_t *self;

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void Normalize(float *ax, float *ay, float *az)
{
    float norm = invSqrt((*ax)*(*ax) + (*ay)*(*ay) + (*az)*(*az));
    *ax *= norm;
    *ay *= norm;
    *az *= norm;
}

static void Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    self->gx = gx - self->gyroBias[0];
    self->gy = gy - self->gyroBias[1];
    self->gz = gz - self->gyroBias[2];

    Normalize(&ax, &ay, &az);

    float q0 = self->q0, q1 = self->q1, q2 = self->q2, q3 = self->q3;

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    if(self->Ki > 0.0f)
    {
        self->exInt += ex * self->Ki * self->dt;
        self->eyInt += ey * self->Ki * self->dt;
        self->ezInt += ez * self->Ki * self->dt;

        self->gx += self->exInt;
        self->gy += self->eyInt;
        self->gz += self->ezInt;
    }

    self->gx += self->Kp * ex;
    self->gy += self->Kp * ey;
    self->gz += self->Kp * ez;

    gx *= 0.5f * self->dt;
    gy *= 0.5f * self->dt;
    gz *= 0.5f * self->dt;

    float dq0 = (-q1*gx - q2*gy - q3*gz);
    float dq1 = ( q0*gx + q2*gz - q3*gy);
    float dq2 = ( q0*gy - q1*gz + q3*gx);
    float dq3 = ( q0*gz + q1*gy - q2*gx);

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    float norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    self->q0 = q0 * norm;
    self->q1 = q1 * norm;
    self->q2 = q2 * norm;
    self->q3 = q3 * norm;

    self->roll  = atan2f(2.0f*(self->q0*self->q1 + self->q2*self->q3),
                         1 - 2.0f*(self->q1*self->q1 + self->q2*self->q2)) * 57.29578f;

    self->pitch = asinf(2.0f*(self->q0*self->q2 - self->q3*self->q1)) * 57.29578f;

    self->yaw   = atan2f(2.0f*(self->q0*self->q3 + self->q1*self->q2),
                         1 - 2.0f*(self->q2*self->q2 + self->q3*self->q3)) * 57.29578f;
}

static float GetRoll(void){ return self->roll; }
static float GetPitch(void){ return self->pitch; }
static float GetYaw(void){ return self->yaw; }

static void Init(float kp, float ki, float dt)
{
    self = &Mahony;

    self->Kp = kp;
    self->Ki = ki;
    self->dt = dt;

    self->q0 = 1.0f;
    self->q1 = 0.0f;
    self->q2 = 0.0f;
    self->q3 = 0.0f;

    self->exInt = 0;
    self->eyInt = 0;
    self->ezInt = 0;

    self->gyroBias[0] = 0;
    self->gyroBias[1] = 0;
    self->gyroBias[2] = 0;

    self->roll = 0;
    self->pitch = 0;
    self->yaw = 0;
}

MahonyAHRS_t Mahony =
{
    .Init = Init,
    .Update = Update,
    .GetRoll = GetRoll,
    .GetPitch = GetPitch,
    .GetYaw = GetYaw
};