//
// Created by CaoKangqi on 2026/2/23.
//

#include "powerControl.h"
// #include <stdint.h>
#include "DJI_Motor.h"
#include "All_Init.h"
#include <math.h>
#include "vmc.h"

ALL_POWER_RX All_Power;
model_t model;

typedef struct
{
    float P_left;
    float P_right;
    float P_total;

    float scale;

    float limit;

} Power_Debug_t;

Power_Debug_t Power_Debug;

void Power_control_init(model_t *model) {
    model->Kp = 3.0f;
    model->Remaining_Buffer = 25.0f;
    model->rpm_to_rad = 2.0f * 3.14159265f / 60.0f;

    // M3508 模型参数
    model->m3508.k1 = 1.5756e-02f;
    model->m3508.k2 = 1.94e-01f;
    model->m3508.k3 = 1.9202e-05f;
    model->m3508.k4 = 1.15f;
    model->m3508.current_convert = 20.0f / 16384.0f;
}


float motor_power(DJI_MOTOR_Typedef *m, Leg_Typedef *leg, motor_model_t *p, float rpm_to_rad) {
    float w = m->DATA.Speed_now * rpm_to_rad;
    float I = leg->torque_send.Tw * 2084.437069138358730844994838f * p->current_convert;
    if (w == 0 || I == 0) return 0;      
    return p->k1 * w * I + p->k2 * I * I + p->k3 * w * w + p->k4;
}

float solve_scale(float PL, float PR, float limit) {
    float A = 0, B = 0, C = PL + PR - limit;

    A = 1e-6f + 0.5f * (PL + PR);
    B = 0;

    float delta = B * B - 4.0f * A * C;

    float s;
    if (delta <= 0) s = 0.0f;
    else s = (-B + sqrtf(delta)) / (2.0f * A);

    if (s > 1) s = 1;
    if (s < 0) s = 0;

    return s;
}

uint8_t chassis_power_control_2wheel(MOTOR_Typedef *M,
                                     Leg_Typedef *left,
                                     Leg_Typedef *right,
                                     model_t *model,
                                     float P_limit,
                                     float rpm_to_rad) {

    float PL = motor_power(&M->left_wheel, left, &model->m3508, model->rpm_to_rad);
    float PR = motor_power(&M->right_wheel, right, &model->m3508, model->rpm_to_rad);

    float P = PL + PR;

    // if (P <= P_limit) return 1;

    float s = solve_scale(PL, PR, P_limit);

    // 应改目标值，先用放缩测试
    left->torque_send.Tw *= s;
    right->torque_send.Tw *= s;

    // 测试输出
    Power_Debug.P_left  = PL;
    Power_Debug.P_right = PR;
    Power_Debug.P_total = P;
    Power_Debug.limit   = P_limit;
    Power_Debug.scale = s;
                                    
    VOFA_justfloat(Power_Debug.P_left,
                   Power_Debug.P_right,
                   Power_Debug.P_total,
                   Power_Debug.limit,
                   Power_Debug.scale,
                   0.0f,0,0,
                   All_Power.P_Chassis.current,
                   All_Power.P_Chassis.power);
    return 1;
}

//功率计接收解算函数
void CAN_POWER_Rx(Power_Typedef* Power, uint8_t *rx_data)
{
    int16_t raw_shunt = (int16_t)((int16_t)rx_data[0] << 8 | rx_data[1]);
    int16_t raw_bus   = (int16_t)((int16_t)rx_data[2] << 8 | rx_data[3]);
    int16_t raw_curr  = (int16_t)((int16_t)rx_data[4] << 8 | rx_data[5]);
    //int16_t raw_pwr   = (int16_t)((int16_t)rx_data[6] << 8 | rx_data[7]);

    Power->shunt_volt = (float)raw_shunt / 1000.0f;
    Power->bus_volt   = (float)raw_bus   / 1000.0f;
    Power->current    = (float)raw_curr  / 1000.0f;
    //Power->power      = (float)raw_pwr   / 100.0f;
    Power->power      = Power->bus_volt * Power->current;
}
//缓冲能量计算
/*void Buffer_Calc(Power_Typedef* Power, User_Data_T *user_data)
{
    static uint8_t is_initialized = 0;
    static uint8_t calc_counter = 0;

    if (!is_initialized) {
        Power->buffer_energy = 60.0f;
        is_initialized = 1;
    }

    float power_limit = user_data->robot_status.chassis_power_limit;
    float max_buffer_energy = 60.0f;
    float now_power = Power->power;

    calc_counter++;

    if (calc_counter >= 10)
    {
        Power->buffer_energy += (power_limit - now_power) * 0.01f;
        calc_counter = 0;
    }

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}*/
void Buffer_Calc(Power_Typedef* Power, User_Data_T *user_data)
{
    static uint8_t is_initialized = 0;

    if (!is_initialized) {
        Power->buffer_energy = 60.0f;
        is_initialized = 1;
    }
    float power_limit = 50.0f;
    float max_buffer_energy = 60.0f;
    //power_limit = user_data->robot_status.chassis_power_limit;
    float now_power = Power->power;
    Power->buffer_energy += (power_limit - now_power) * 0.001f;

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}