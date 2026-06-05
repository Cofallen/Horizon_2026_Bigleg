#ifndef __OBSERVE_H__
#define __OBSERVE_H__

#include "main.h"
#include "vmc.h"

extern KalmanFilter_t vaEstimateKF;

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF, float dt);
float xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float vel, float acc);
float slip_Check(Leg_Typedef *Leg_l, Leg_Typedef *Leg_r);

#endif // !__OBSERVE_H__