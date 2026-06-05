#include "main.h"
#include "All_Init.h"
#include "chassisL.h"


void Chassis_SendTorque()
{
    RUI_V_CONTAL.DWT_TIME.TIM7_Dtime = DWT_GetDeltaT(&RUI_V_CONTAL.DWT_TIME.TIM7_DWT_Count);
    static uint8_t temp = 1;
    uint8_t count = 0;
    // if (temp == 1){
    //     mit_ctrl(&hcan1, 0x02, 0, 0, 0, 0, 0);
    //     temp = -temp;
    // }
    // else{
    //     mit_ctrl(&hcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //     temp = -temp;
    // }
    //     mit_ctrl(&hcan1, 0x01, 0, 0, 0, 0, 0);
    // mit_ctrl(&hcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DM_send_iq(CAN_HandleTypeDef *hcan, uint16_t motor_id, float iq)
{
    float torque = iq * 0.00061035f;
    mit_ctrl(hcan, motor_id, 0, 0, 0, 0, torque);
}