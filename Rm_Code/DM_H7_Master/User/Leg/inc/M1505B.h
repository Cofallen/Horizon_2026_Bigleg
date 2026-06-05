#ifndef _M1505B_H
#define _M1505B_H
#include "stdint.h"
#include "can_bsp.h"

#define df_motor_mode_vol     0x00
#define df_motor_mode_iq      0x01
#define df_motor_mode_vel     0x02
#define df_motor_mode_disable 0x09
#define df_motor_mode_enable  0x0A

#define df_M1505B_TORQUE_CONSTANT 0.8f  //转矩常数

#define df_M1505B_iq_set_max    16383
#define df_M1505B_iq_set_min   -16383
#define df_M1505B_vel_set_max    7500
#define df_M1505B_vel_set_min   -7500
#define df_M1505B_vol_set_max   16384
#define df_M1505B_vol_set_min  -16384

typedef enum
{
	mode_vol    = 0x00,
	mode_iq     = 0x01,
	mode_vel    = 0x02,
	mode_disable= 0x09,
	mode_enable = 0x0A,
	mode_unknow = 0xFF
}mode_t;
typedef enum
{
	fault_none           =0x00,  //0x00 无故障
	fault_under_voltage1 =0x01,  //0x01 欠压 1（18V < 母线电压 < 20V）
	fault_under_voltage2 =0x02,  //0x02 欠压 2（母线电压 < 18V）
	fault_over_voltage   =0x03,  //0x03 过压（母线电压 > 36V）
	fault_over_current   =0x0A,  //0x0A 过流（默认：母线电流 > 35A）
	fault_over_temp1     =0x20,  //0x20 过温 1（电机绕组温度 > 80℃）
	fault_over_temp2     =0x1F,  //0x1F 过温 2（电机绕组温度 > 110℃）
	fault_sample_resistor=0x29,  //0X29 采样电阻故障
	fault_encode_self    =0x2A,  //0x2A 位置传感器自身故障
	fault_encode_noise   =0x2B,  //0x2B 位置传感器信号被干扰
	fault_temp_sensor    =0x2D,  //0X2D 温度传感器超出量程
	fault_comm_timeout   =0x3C,  //0x3C 通信超时(默认无保护，需用户自行开启)
	fault_stall          =0x62,  //0x62 堵转(默认:电流 > 5A 并且 转速为 0)
	fault_unknow         =0xFF   //未知
}fault_t;
typedef struct
{
	uint16_t id;      //电机ID
	uint8_t set_mod;  //电机目标模式
	uint8_t real_mod; //电机实际模式
	float vel;       	//电机转速
	float iq;        	//电机转矩电流
	float pos;       	//电机角度
	fault_t fault;   	//电机状态
	uint8_t fdbcak_mode;
}Weel_Motor_t;



void M1505B_set_control_mode(hcan_t* hcan,mode_t mode1,mode_t mode2,mode_t mode3,mode_t mode4,mode_t mode5,mode_t mode6,mode_t mode7,mode_t mode8);
void M1505B_set_control_mode_fdback(Weel_Motor_t *motor,uint8_t *rx_data);
void M1505B_control(hcan_t *hcan,uint32_t id,uint8_t mode,float num1,float num2,float num3,float num4);
void M1505B_control_fdback(Weel_Motor_t *motor,uint8_t *rx_data);
void M1505B_calibration(hcan_t *hcan);
void M1505B_set_fdback_mode(hcan_t *hcan,uint8_t mode1,uint8_t mode2,uint8_t mode3,uint8_t mode4,uint8_t mode5,uint8_t mode6,uint8_t mode7,uint8_t mode8);
void M1505B_set_fdback_mode_fdback(Weel_Motor_t *motor,uint8_t *rx_data);

#endif
