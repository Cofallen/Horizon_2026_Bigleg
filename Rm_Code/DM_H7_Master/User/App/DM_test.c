#include "DM_test.h"

DM_MOTOR_Typedef DM_Motor_Test;

float Yaw_Aim = 0.0f;
float Yaw_Angle;
float DM_Kp = 0.03f;

float Yaw_OUT = 0.0f;
uint8_t aaaaaa = 0;

void DM_test_init(void) {
    //使能
    motor_mode(&hfdcan1, 0x03, 0x100, DM_CMD_MOTOR_MODE);
    DM_Motor_Test.PV.SPE = 30;
}

void DM_test(IMU_Data_t *imu_data) {
	
	aaaaaa++;
    
    Yaw_Angle = imu_data->yaw*(3.1415926f/180.0f);

    Yaw_OUT = (Yaw_Aim - Yaw_Angle) * DM_Kp;
	
	if(Yaw_OUT > 3.1415926f*4.0f)	Yaw_OUT = 3.1415926f*4.0f;
	if(Yaw_OUT < -3.1415926f*4.0f)	Yaw_OUT = -3.1415926f*4.0f;

    DM_Motor_Test.PV.POS += Yaw_OUT;
	


	if(aaaaaa%2)
		pos_speed_ctrl(&hfdcan1, 0x03, &DM_Motor_Test.PV);//发送
	else
		motor_mode(&hfdcan1, 0x03, 0x100, DM_CMD_MOTOR_MODE);//使能
}
