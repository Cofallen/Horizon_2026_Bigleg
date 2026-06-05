//
// Created by CaoKangqi on 2026/2/23.
//

#ifndef G4_FRAMEWORK_POWER_CTRL_H
#define G4_FRAMEWORK_POWER_CTRL_H

#include "All_Init.h"
#include "main.h"
#include "user_lib.h"
#include "powerCap.h"
#include "Referee.h"
#include "vmc.h"

typedef struct {
    float k1, k2, k3, k4;
    float current_convert; // 电流转换系数 (3508: 20/16384, 6020: 3/16384)
} motor_model_t;

typedef struct {
    float Kp;
    float Remaining_Buffer;
    motor_model_t m3508;
    motor_model_t m6020;
    float rpm_to_rad;
} model_t;

void Power_control_init(model_t *model);
void chassis_power_distribute(DJI_MOTOR_Typedef *motor[4],
                                 float I_cmd[4],
                                 float P_limit,
                                 model_t *model);
uint8_t chassis_power_control_2wheel(MOTOR_Typedef *M,
                                     Leg_Typedef *left,
                                     Leg_Typedef *right,
                                     model_t *model,
                                     float P_limit,
                                     float rpm_to_rad);

typedef struct {
    float shunt_volt;
    float bus_volt;
    float current;
    float power;
    float buffer_energy;
} Power_Typedef;

typedef struct {
    Power_Typedef P_Chassis;
} ALL_POWER_RX;

extern ALL_POWER_RX All_Power;
extern model_t model;

void CAN_POWER_Rx(Power_Typedef* pPower, uint8_t *rx_data);
void Buffer_Calc(Power_Typedef* Power,User_Data_T *user_data);

#endif //G4_FRAMEWORK_POWER_CTRL_H