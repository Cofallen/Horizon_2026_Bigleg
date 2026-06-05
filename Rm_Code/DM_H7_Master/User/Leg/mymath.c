#include "mymath.h"
#include "Motors.h"
#include "vmc.h"

void Discreteness_Init(Discreteness_TypeDef *object, float alpha)
{
    object->last_sum = 0;
    object->sum_num = 0;
    object->last_diff = 0;
    object->diff_num = 0;
    object->dt = 0;
    object->alpha = alpha;
}

float Discreteness_Sum(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->sum_num += input * object->dt;
    if (object->alpha != 0.0f)
    {
        object->sum_num = Lowpass_Filter(&object->last_sum, object->sum_num, object->alpha);
    }
    object->last_sum = object->sum_num; 
    return object->sum_num;
}


float Discreteness_Diff(Discreteness_TypeDef *object, float input, float dt)
{
    object->dt = dt;
    object->diff_num = (input - object->last_diff) / object->dt;
    if (object->alpha != 0.0f)
    {
        object->diff_num = Lowpass_Filter(&object->last_diff, object->diff_num, object->alpha);
    }
    object->last_diff = input;
    return object->diff_num;
}

float Luenberger_Init(Luenberger_Typedef *object, float l1, float l2)
{
    object->l1 = l1;
    object->l2 = l2;
}

float Luenberger_Calc(Luenberger_Typedef *object, float input, float dt)
{
    object->dt = dt;
    object->x  = input;

    object->x_hat += object->dt * object->v_hat + object->l1 * (object->x - object->x_hat) * object->dt;
    object->v_hat += object->l2 * (object->x - object->x_hat) * object->dt;

    return object->v_hat;
}



#define TUEQUE2NUM 2730.6666666666666666666666666667f
void DJI_Torque_Control(hcan_t* hcan, uint16_t stdid, float t1, float t2, float t3, float t4)
{
    uint8_t Data[8];

    int16_t num1, num2, num3, num4;
    (fabsf(t1) < 0.0f) ? (t1 = 0.0f) : (t1 = t1);
    (fabsf(t2) < 0.0f) ? (t2 = 0.0f) : (t2 = t2);
    (fabsf(t3) < 0.0f) ? (t3 = 0.0f) : (t3 = t3);
    (fabsf(t4) < 0.0f) ? (t4 = 0.0f) : (t4 = t4);
    
    num1 = (int16_t)(t1 * TUEQUE2NUM * TORQUE_RATE);
    num2 = (int16_t)(t2 * TUEQUE2NUM * TORQUE_RATE);
    num3 = (int16_t)(t3 * TUEQUE2NUM * TORQUE_RATE);
    num4 = (int16_t)(t4 * TUEQUE2NUM * TORQUE_RATE);
    Data[0] = ((num1) >> 8);
    Data[1] = (num1);
    Data[2] = ((num2) >> 8);
    Data[3] = (num2);
    Data[4] = ((num3) >> 8);
    Data[5] = (num3);
    Data[6] = ((num4) >> 8);
    Data[7] = (num4);

    canx_send_data(hcan, stdid, Data, 8);
}

float Lowpass_Filter(float *last_output, float input, float alpha)
{
    *last_output = alpha * input + (1.0f - alpha) * (*last_output);
    return *last_output;
}