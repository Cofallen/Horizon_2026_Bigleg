#ifndef __MOTORS_H
#define __MOTORS_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "BM_Motor.h"
// typedef struct
// {
//     DM_MOTOR_Typedef   DM_YAW;
//     DM_MOTOR_Typedef   DM_PITCH;
//     DJI_MOTOR_Typedef DJI_ATTACK_left;
//     DJI_MOTOR_Typedef DJI_ATTACK_right;
//     DJI_MOTOR_Typedef DJI_2006;
//     DJI_MOTOR_Typedef DJI_Motor_Test;
// }MOTOR_Typedef;
typedef struct
{
    BM_motor_t  left_front;
    BM_motor_t  left_back;
    BM_motor_t  right_front;
    BM_motor_t  right_back;
    DJI_MOTOR_Typedef left_wheel;
    DJI_MOTOR_Typedef right_wheel;
}MOTOR_Typedef;

typedef struct
{
    /******************************遥控********************************/
    uint8_t RM_DBUS;
    /***************************自定义控制器********************************/
    uint8_t RM_RX;
    /******************************头部电机********************************/
    uint8_t MOTOR_HEAD_Pitch;
    uint8_t MOTOR_HEAD_Roll;
    /******************************主控位置********************************/
    uint8_t MASTER_LOCATION;
    /******************************运行模式********************************/
    uint8_t RM_MOD;

}RUI_ROOT_STATUS_Typedef;

//2023-05-13 17:17 | 遥控键鼠转换结构体
typedef struct
{
    struct
    {
        float VX;
        float VY;
        float VW;
        float wheel1;
        float wheel2;
        float wheel3;
        float wheel4;
        uint8_t CAP;

    } BOTTOM;

    struct
    {
        float Pitch;
        float Pitch_MAX;//常量
        float Pitch_MIN;//常量
        float Yaw;

    } HEAD;

    struct
    {
        float SHOOT_L;
        float SHOOT_R;
        float SHOOT_M;
        float Shoot_Speed;//常量
        int64_t Single_Angle;//常量
    } SHOOT;

    struct
    {
        int16_t YAW_INIT_ANGLE;//常量
        int16_t YAW_ANGLE;
        int16_t RELATIVE_ANGLE;
        int16_t YAW_SPEED;
        float TOP_ANGLE;
    } CG;

    struct
    {
        float Speed_err_L;
        float Speed_err_R;
        int32_t Angle;
        float Speed_Aim_L;
        float Speed_Aim_R;
        uint8_t JAM_Flag;
        uint32_t Shoot_Number;
        uint32_t Shoot_Number_Last;
    }SHOOT_Bask;

    struct 
    {
        float RobotUI_Dtime;
        float Move_Dtime;
        float Defiant_Dtime;
        float IMU_Dtime;
        float StartRoot_Dtime;
        float TIM7_Dtime;
        uint32_t RobotUI_DWT_Count;
        uint32_t Move_DWT_Count;
        uint32_t Defiant_DWT_Count;
        uint32_t IMU_DWT_Count;
        uint32_t Root_DWT_Count;
        uint32_t TIM7_DWT_Count;
    }DWT_TIME;
    
    uint8_t MOD[2];
    uint8_t ORE;
}CONTAL_Typedef;

#endif
