/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-08 17:08:38
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-14 20:38:45
 */
/*
 *                   ___====-_  _-====___
 *             _--^^^#####//      \\#####^^^--_
 *          _-^##########// (    ) \\##########^-_
 *         -############//  |\^^/|  \\############-
 *       _/############//   (@::@)   \############\_
 *      /#############((     \\//     ))#############\
 *     -###############\\    (oo)    //###############-
 *    -#################\\  / VV \  //#################-
 *   -###################\\/      \//###################-
 *  _#/|##########/\######(   /\   )######/\##########|\#_
 *  |/ |#/\#/\#/\/  \#/\##\  |  |  /##/\#/  \/\#/\#/\#| \|
 *  `  |/  V  V  `   V  \#\| |  | |/#/  V   '  V  V  \|  '
 *     `   `  `      `   / | |  | | \   '      '  '   '
 *                      (  | |  | |  )
 *                     __\ | |  | | /__
 *                    (vvv(VVV)(VVV)vvv)
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                神兽保佑            永无BUG
 */


#include "All_Init.h"

//遥控相关变量
uint8_t DBUS_RX_DATA[18] __attribute__((section(".ARM.__at_0x24000000")));
DBUS_Typedef WHW_V_DBUS = { 0 };
CONTAL_Typedef RUI_V_CONTAL = { 0 };

//裁判系统相关变量
ALL_RX_Data_T ALL_RX = { 0 };
User_Data_T User_data = { 0 };

//测试
uint8_t RX[20] = { 0 };

//电机
MOTOR_Typedef ALL_MOTOR = { 0 };

//陀螺仪
FuzzyRule_t fuzzyRules = { 0 };
PID_t temppid = { 0 };
float imu_temp_fuzzy[3] = TEMPERATURE_Fuzzy;
float imu_temp_PID[3] = TEMPERATURE_PID;
//pid_type_def imu_temp = { 0 };
IMU_Data_t IMU_Data = { 0 };

//电源电压变量
V_Input_t V_Input = { 0 };

//运动控制监测结构体
RUI_ROOT_STATUS_Typedef RUI_ROOT_STATUS = { 0 };

//图传串口
select_t selects = { 0 };
Data_RX_0x302_t Data_RX_0x302 = { 0 };
Data_RX_0x304_t Data_RX_0x304 = { 0 };

uint8_t Referee_Rx_Buf[REFEREE_RXFRAME_LENGTH];

void Everying_Init(void)
{
    //DWT初始化
    DWT_Init(500);

    //陀螺仪初始化
	Fuzzy_Rule_Init(&fuzzyRules, NULL, NULL, NULL, 
					imu_temp_fuzzy[0], imu_temp_fuzzy[1], imu_temp_fuzzy[2], 
					0, 0);
	temppid.FuzzyRule = &fuzzyRules;	
	PID_Init(&temppid,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_MAX_IOUT,
				imu_temp_PID,0,0,0.5f,0.5f,2,Integral_Limit);
    IMU_QuaternionEKF_Init(10, 0.001f, 10000000, 1, 0.001f,0); //ekf初始化
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init()){}
		
	//ADC初始化	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)V_Input.adc_val,2);

    //定时器初始化
	HAL_TIM_Base_Start_IT(&htim7);
		
	//FDCAN初始化
    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
    //开启XT30 2+2 可控输出
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);

    //开启对外5V
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);

	//舵机PWM初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	//蜂鸣器PWM初始化
    BSP_Buzzer_Init();
		
	//串口初始化
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 510);//图传串口
	HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, Referee_Rx_Buf, REFEREE_RXFRAME_LENGTH);//裁判系统串口
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);//关闭 DMA 半传中断
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90);//遥控串口	
}

void StartDefaultTask(void const * argument)
{
	osDelay(50);
	
	MX_USB_DEVICE_Init();
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)selects.Data, 255);//图传串口
	HAL_UARTEx_ReceiveToIdle_DMA(&huart10, (uint8_t *)RX, 40);//上位机串口
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)ALL_RX.Data, 510);//裁判系统串口
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, DBUS_RX_DATA, 90);//遥控串口
	
	for(;;)
	{
		osDelay(1);
	}
}

