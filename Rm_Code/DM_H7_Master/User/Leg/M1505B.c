#include "M1505B.h"
#include "can_bsp.h"

/*
M505B使用：上电后默认使能，在使能且转速为0时才可以设置模式，腿设置成电流环。
使用流程：上电→→→→→→设置反馈方式（根据需求）→→→→→→设置模式→→→→→→控制
                                                             ↑                 ↓
                                                            使能←失能←进入保护←																									 
*/


/*------电机设置模式------*/
//切换模式时转速需为0，使能后才能切换模式
void M1505B_set_control_mode(hcan_t* hcan,mode_t mode1,mode_t mode2,mode_t mode3,mode_t mode4,mode_t mode5,mode_t mode6,mode_t mode7,mode_t mode8)
{
	uint32_t stdid = 0x105;
	uint8_t tx_data[8] = {0};
	
	tx_data[0] = mode1;
	tx_data[1] = mode2;
	tx_data[2] = mode3;
	tx_data[3] = mode4;
	tx_data[4] = mode5;
	tx_data[5] = mode6;
	tx_data[6] = mode7;
	tx_data[7] = mode8;
	
	canx_send_data(hcan, stdid, tx_data, 8);
}

/*------电机设置模式反馈------*/
void M1505B_set_control_mode_fdback(Weel_Motor_t *motor,uint8_t *rx_data)
{
	motor->real_mod = rx_data[0];
}

/*------电机转矩转换为电流------*/
float M1505B_torque_to_iq(float torque)
{
	return torque/df_M1505B_TORQUE_CONSTANT;
}
/*------限幅float------*/
int16_t M1505B_limit_float(int16_t num,int16_t max,int16_t min)
{
	if(num >= max)
	{return max;}
	else if(num <= min)
	{return min;}
	else
	{return num;}
}



/*------电机控制------*/
void M1505B_control(hcan_t *hcan,uint32_t id,uint8_t mode,float num1,float num2,float num3,float num4)
{
	uint32_t stdid = 0;
	uint8_t tx_data[8] = {0};
	
	stdid = id;
	
	int16_t num1_temp = 0, num2_temp = 0, num3_temp = 0, num4_temp = 0;
	switch(mode)
	{
		case df_motor_mode_iq:
		{
			num1_temp = (int16_t)lroundf(M1505B_torque_to_iq(num1) * 32767.0f / 55.0f);
			num2_temp = (int16_t)lroundf(M1505B_torque_to_iq(num2) * 32767.0f / 55.0f);
			num3_temp = (int16_t)lroundf(M1505B_torque_to_iq(num3) * 32767.0f / 55.0f);
			num4_temp = (int16_t)lroundf(M1505B_torque_to_iq(num4) * 32767.0f / 55.0f);
			/*限幅*/
			num1_temp = M1505B_limit_float(num1_temp,df_M1505B_iq_set_max,df_M1505B_iq_set_min);
			num2_temp = M1505B_limit_float(num2_temp,df_M1505B_iq_set_max,df_M1505B_iq_set_min);
			num3_temp = M1505B_limit_float(num3_temp,df_M1505B_iq_set_max,df_M1505B_iq_set_min);
			num4_temp = M1505B_limit_float(num4_temp,df_M1505B_iq_set_max,df_M1505B_iq_set_min);
		}break;
		case df_motor_mode_vel:
		{
			num1_temp = (int16_t)lroundf(num1 * 10.0f);  // 分辨率0.1RPM
      num2_temp = (int16_t)lroundf(num2 * 10.0f);
      num3_temp = (int16_t)lroundf(num3 * 10.0f);
      num4_temp = (int16_t)lroundf(num4 * 10.0f);
			/*限幅*/
			num1_temp = M1505B_limit_float(num1_temp,df_M1505B_vel_set_max,df_M1505B_vel_set_min);
			num2_temp = M1505B_limit_float(num2_temp,df_M1505B_vel_set_max,df_M1505B_vel_set_min);
			num3_temp = M1505B_limit_float(num3_temp,df_M1505B_vel_set_max,df_M1505B_vel_set_min);
			num4_temp = M1505B_limit_float(num4_temp,df_M1505B_vel_set_max,df_M1505B_vel_set_min);
		}break;
		case df_motor_mode_vol:
		{
			 num1_temp = (int16_t)lroundf(num1 * 16384.0f);
       num2_temp = (int16_t)lroundf(num2 * 16384.0f);
       num3_temp = (int16_t)lroundf(num3 * 16384.0f);
       num4_temp = (int16_t)lroundf(num4 * 16384.0f);
			/*限幅*/
			num1_temp = M1505B_limit_float(num1_temp,df_M1505B_vol_set_max,df_M1505B_vol_set_min);
			num2_temp = M1505B_limit_float(num2_temp,df_M1505B_vol_set_max,df_M1505B_vol_set_min);
			num3_temp = M1505B_limit_float(num3_temp,df_M1505B_vol_set_max,df_M1505B_vol_set_min);
			num4_temp = M1505B_limit_float(num4_temp,df_M1505B_vol_set_max,df_M1505B_vol_set_min);
		}break;
	}
	
	
	tx_data[0] = (uint8_t)((num1_temp >> 8) & 0xFF);
	tx_data[1] = (uint8_t)(num1_temp & 0xFF);
	tx_data[2] = (uint8_t)((num2_temp >> 8) & 0xFF);
	tx_data[3] = (uint8_t)(num2_temp & 0xFF);
	tx_data[4] = (uint8_t)((num3_temp >> 8) & 0xFF);
	tx_data[5] = (uint8_t)(num3_temp & 0xFF);
	tx_data[6] = (uint8_t)((num4_temp >> 8) & 0xFF);
	tx_data[7] = (uint8_t)(num4_temp & 0xFF);
	
	canx_send_data(hcan, stdid, tx_data, 8);
}

/* 故障值转换函数 - 安全处理未知故障码 */
static fault_t safe_fault_convert(uint8_t fault_code)
{
    switch(fault_code) {
        case 0x00: return fault_none;
        case 0x01: return fault_under_voltage1;
        case 0x02: return fault_under_voltage2;
        case 0x03: return fault_over_voltage;
        case 0x0A: return fault_over_current;
        case 0x1F: return fault_over_temp2;
        case 0x20: return fault_over_temp1;
        case 0x29: return fault_sample_resistor;
        case 0x2A: return fault_encode_self;
        case 0x2B: return fault_encode_noise;
        case 0x2D: return fault_temp_sensor;
        case 0x3C: return fault_comm_timeout;
        case 0x62: return fault_stall;
        default:   return fault_unknow;
    }
}

/* 模式值转换函数 - 安全处理未知模式码 */
static mode_t safe_mode_convert(uint8_t mode_code)
{
    switch(mode_code) {
        case 0x00: return mode_vol;
        case 0x01: return mode_iq;
        case 0x02: return mode_vel;
        case 0x09: return mode_disable;
        case 0x0A: return mode_enable;
        default:   return mode_unknow;
    }
}

/*------电机控制反馈------*/
void M1505B_control_fdback(Weel_Motor_t *motor,uint8_t *rx_data)
{
    int16_t speed_raw = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    motor->vel = (float)speed_raw * 0.1f;
    
    int16_t current_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    motor->iq = (float)current_raw * 55.0f / 32767.0f;
    
    uint16_t position_raw = (uint16_t)((rx_data[4] << 8) | rx_data[5]);
    motor->pos = (float)position_raw * 360.0f / 32767.0f;
    
    motor->fault = safe_fault_convert(rx_data[6]);
    
    motor->real_mod = safe_mode_convert(rx_data[7]);
}


/*------电机校准------*/
void M1505B_calibration(hcan_t *hcan)
{
	uint32_t stdid = 0x104;
	uint8_t tx_data[8] = {0};
	canx_send_data(hcan, stdid, tx_data, 8);
}

/*------设置电机反馈模式------*/
void M1505B_set_fdback_mode(hcan_t *hcan,uint8_t mode1,uint8_t mode2,uint8_t mode3,uint8_t mode4,uint8_t mode5,uint8_t mode6,uint8_t mode7,uint8_t mode8)
{
	uint32_t stdid = 0x106;
	uint8_t tx_data[8] = {0};
	
	tx_data[0] = mode1;
	tx_data[1] = mode2;
	tx_data[2] = mode3;
	tx_data[3] = mode4;
	tx_data[4] = mode5;
	tx_data[5] = mode6;
	tx_data[6] = mode7;
	tx_data[7] = mode8;
	
	canx_send_data(hcan, stdid, tx_data, 8);
}

/*------设置电机反馈模式反馈------*/
void M1505B_set_fdback_mode_fdback(Weel_Motor_t *motor,uint8_t *rx_data)
{
	motor->fdbcak_mode = rx_data[1];
}