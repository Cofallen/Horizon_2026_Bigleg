// #include "K3debug_Task.h"

// #define WHW_DF_PID_F_test  {   0.0f,   0.0f,   0.0f   }
// #define WHW_DF_PID_P_test  {   0.39163f,   0.0086323f,   4.0177f   }
// #define WHW_DF_PID_S_test  {   85.6365f,   7914.5213f,   0.10548f   }

// void k3debug_init(MOTOR_Typedef *MOTOR)
// {
// 	//电机一切初始化
//     float PID_F_Test[3] = WHW_DF_PID_F_test;
//     float PID_P_Test[3] = WHW_DF_PID_P_test;
//     float PID_S_Test[3] = WHW_DF_PID_S_test;

//     /*前馈*/
//     float b = 2.0f*0.001f*3.1415926f*2.0f;
//     float alpha = b/(b+1);
//     Feedforward_Init(&MOTOR->DJI_Motor_Test.PID_F, 3000, PID_F_Test,
//                      1.0f, 2, 2);

//     /*角度环*/
//     PID_Init(&MOTOR->DJI_Motor_Test.PID_P, 6000.0f, 3.0f,
//              PID_P_Test, 1000.0f, 1000.0f,
//              100.0f, 3.7449f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

//     /*速度环*/
//     PID_Init(&MOTOR->DJI_Motor_Test.PID_S, 20000.0f, 100.0f,
//              PID_S_Test, 1000.0f, 1000.0f,
//              1.0f, 327.5868f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
// }

// void k3debug_task(MOTOR_Typedef *MOTOR, DBUS_Typedef *DBUS)
// {
//     /*目标值*/
// 	MOTOR->DJI_Motor_Test.DATA.Aim = (float)(DBUS->Remote.CH0_int16 * 5);

//     /*前馈*/
// 	Feedforward_Calculate(&MOTOR->DJI_Motor_Test.PID_F,
//                           MOTOR->DJI_Motor_Test.DATA.Aim);

//     /*角度环*/
//     PID_Calculate(&MOTOR->DJI_Motor_Test.PID_P,
//                   (float)MOTOR->DJI_Motor_Test.DATA.Angle_Infinite,
//                   MOTOR->DJI_Motor_Test.DATA.Aim);

//     /*速度环*/
//     PID_Calculate(&MOTOR->DJI_Motor_Test.PID_S,
//                   MOTOR->DJI_Motor_Test.DATA.Speed_now,
//                   MOTOR->DJI_Motor_Test.PID_P.Output);

//     /*总输出*/
// 	float tmp1 = MOTOR->DJI_Motor_Test.PID_F.Output +
//                  MOTOR->DJI_Motor_Test.PID_S.Output;

//     /*CAN发送*/
// 	DJI_Power_Ctrl(&hfdcan1,
//                    0x1ff,
//                    (int16_t)tmp1,
//                    (int16_t)0,
//                    (int16_t)0,
//                    (int16_t)0);

//     /*上位机发送*/
// 	Anonymity_TX((int16_t)(DBUS->Remote.CH0_int16*5), MOTOR->DJI_Motor_Test.DATA.Angle_now,
// 					0,0,0,0,0,0,0,0);
// }
