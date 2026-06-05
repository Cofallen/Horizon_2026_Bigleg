// /*
//  *                                                     __----~~~~~~~~~~~------___
//  *                                    .  .   ~~//====......          __--~ ~~
//  *                    -.            \_|//     |||\\  ~~~~~~::::... /~
//  *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
//  *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
//  *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
//  *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
//  *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
//  *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
//  *           '         ~-|      /|    |-~\~~       __--~~
//  *                       |-~~-_/ |    |   ~\_   _-~            /\
//  *                            /  \     \__   \/~                \__
//  *                        _--~ _/ | .-~~____--~-/                  ~~==.
//  *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
//  *                                  -_     ~\      ~~---l__i__i__i--~~_/
//  *                                  _-~-__   ~)  \--______________--~~
//  *                                //.-~~~-~_--~- |-------~~~~~~~~
//  *                                       //.-~~~--\
//  *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  * 
//  *                               神兽保佑            永无BUG
//  */

// #include "Gimbal_Task.h"
// #include "All_Init.h"

// /************************************************************万能分隔符**************************************************************
//  * 	@author:			//小瑞
//  *	@performance:	    //头部PID+前馈总初始化函数
//  *	@parameter:		    //
//  *	@time:				//23-04-13 12:42
//  *	@ReadMe:			//
//  ************************************************************万能分隔符**************************************************************/
// uint8_t RUI_F_MOTOR_PID_HEAD_INIT(MOTOR_Typedef *MOTOR)
// {
//     //电机一切初始化
//     float PID_F_Pitch[3] = WHW_DF_PID_F_Pitch;
//     float PID_P_Pitch[3] = WHW_DF_PID_P_Pitch;
//     float PID_S_Pitch[3] = WHW_DF_PID_S_Pitch;
//     Feedforward_Init(&MOTOR->DJI_2006_Pitch.PID_F, 3000, PID_F_Pitch,
//                      0.5f, 2, 2);
//     PID_Init(&MOTOR->DJI_2006_Pitch.PID_P, 6000.0f, 2000.0f,
//              PID_P_Pitch, 1000.0f, 1000.0f,
//              0.7f, 0.7f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
//     PID_Init(&MOTOR->DJI_2006_Pitch.PID_S, 20000.0f, 200.0f,
//              PID_S_Pitch, 1000.0f, 1000.0f,
//              30.5453f, 0.7f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

//     float PID_F_Roll[3] = WHW_DF_PID_F_Roll;
//     float PID_P_Roll[3] = WHW_DF_PID_P_Roll;
//     float PID_S_Roll[3] = WHW_DF_PID_S_Roll;
//     Feedforward_Init(&MOTOR->DJI_2006_Roll.PID_F, 3000, PID_F_Roll,
//                      0.5f, 2, 2);
//     PID_Init(&MOTOR->DJI_2006_Roll.PID_P, 6000.0f, 2000.0f,
//              PID_P_Roll, 1000.0f, 1000.0f,
//              0.7f, 0.7f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
//     PID_Init(&MOTOR->DJI_2006_Roll.PID_S, 6000.0f, 2000.0f,
//              PID_S_Roll, 1000.0f, 1000.0f,
//              0.7f, 0.7f, 2,
//              Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//              |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//              |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

//     return RUI_DF_READY;
// }

// uint8_t RUI_F_HEAD_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typedef *MOTOR)
// {
//     //检查离线
//     if (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
//         Root->MOTOR_HEAD_Roll      == RUI_DF_OFFLINE)
//         return RUI_DF_ERROR;

//     //电机清空
//     RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_2006_Pitch, 1);
//     RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_2006_Roll, 1);

//     return RUI_DF_READY;
// }

// uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
//                     RUI_ROOT_STATUS_Typedef *Root,
//                     MOTOR_Typdef *MOTOR)
// {
//     static uint8_t PID_INIT=RUI_DF_ERROR;
//     static uint8_t AIM_INIT = RUI_DF_ERROR;

//     /*电机PID赋值*/
//     if (PID_INIT != RUI_DF_READY)
//     {
// 		PID_INIT = RUI_F_MOTOR_PID_HEAD_INIT(MOTOR);
// 		return RUI_DF_ERROR;
//     }

//     /*电机在线检测*/
//     if (AIM_INIT != RUI_DF_READY)
//     {
// 		AIM_INIT = RUI_F_HEAD_AIM_INIT(Root, MOTOR);
// 		return RUI_DF_ERROR;
//     }

//     /*目标值赋值*/
//     MOTOR->DJI_2006_Pitch.DATA.Aim = CONTAL->HEAD.Pitch;
//     MOTOR->DJI_2006_Roll.DATA.Aim = CONTAL->HEAD.Roll;

//     /*遥控离线保护*/
//     if(!Root->RM_DBUS)
//     {
//         MOTOR->DJI_2006_Pitch.PID_P.IntegralLimit = 0;
//         MOTOR->DJI_2006_Pitch.PID_S.IntegralLimit = 0;
//         MOTOR->DJI_2006_Pitch.DATA.Aim = (float)MOTOR->DJI_2006_Pitch.DATA.Angle_Infinite;

//         MOTOR->DJI_2006_Roll.PID_P.IntegralLimit = 0;
//         MOTOR->DJI_2006_Roll.PID_S.IntegralLimit = 0;
//         MOTOR->DJI_2006_Roll.DATA.Aim = (float)MOTOR->DJI_2006_Roll.DATA.Angle_Infinite;
		
// 		PID_INIT=RUI_DF_ERROR;
//     }

//     /*堵转处理*/
//     if(MOTOR->DJI_2006_Pitch.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//         RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_2006_Pitch, 0);
//     if(MOTOR->DJI_2006_Roll.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//         RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_2006_Roll, 0);

//     /*Pitch*/
// 	Feedforward_Calculate(&MOTOR->DJI_2006_Pitch.PID_F,
//                           MOTOR->DJI_2006_Pitch.DATA.Aim);
//     PID_Calculate(&MOTOR->DJI_2006_Pitch.PID_P,
//                   (float)MOTOR->DJI_2006_Pitch.DATA.Angle_Infinite,
//                   MOTOR->DJI_2006_Pitch.DATA.Aim);
//     PID_Calculate(&MOTOR->DJI_2006_Pitch.PID_S,
//                   MOTOR->DJI_2006_Pitch.DATA.Speed_now,
//                   MOTOR->DJI_2006_Pitch.PID_P.Output);
	
//     /*Roll*/
// 	Feedforward_Calculate(&MOTOR->DJI_2006_Roll.PID_F,
//                           MOTOR->DJI_2006_Roll.DATA.Aim);
//     PID_Calculate(&MOTOR->DJI_2006_Roll.PID_P,
//                   (float)MOTOR->DJI_2006_Roll.DATA.Angle_Infinite,
//                   MOTOR->DJI_2006_Roll.DATA.Aim);
//     PID_Calculate(&MOTOR->DJI_2006_Roll.PID_S,
//                   MOTOR->DJI_2006_Roll.DATA.Speed_now,
//                   MOTOR->DJI_2006_Roll.PID_P.Output);

//     /*总输出计算*/
//     float tmp1 = MOTOR->DJI_2006_Pitch.PID_F.Output +
//                  MOTOR->DJI_2006_Pitch.PID_S.Output;

//     float tmp2 = MOTOR->DJI_2006_Roll.PID_F.Output +
//                  MOTOR->DJI_2006_Roll.PID_S.Output;

//     /*FDCAN发送*/
//     DJI_Power_Ctrl(&hfdcan1,
//                    0x200,
//                    (int16_t)tmp1,
//                    (int16_t)tmp2,
//                    (int16_t)0,
//                    (int16_t)0);

//     return RUI_DF_READY;
// }
