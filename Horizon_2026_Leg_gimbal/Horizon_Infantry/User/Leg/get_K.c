#include "get_K.h"
#include "can_bsp.h"
#include "All_Init.h"

boardTxData_t BoardTXData = {0};
boardRxData_t boardRxData = {0};

void Board_to_board_send(boardTxData_t *send, float yaw)
{
    send->dataNeaten.yaw_imu = yaw;
    canx_send_data(&hcan1, 0x1C, send->txData);
}

void Board_to_board_recv(boardRxData_t *recv, uint8_t *data)
{
    // memcpy(recv->rxData, data, sizeof(recv->rxData));
    uint64_t packed = 0;
    for (int i = 0; i < 8; i++) {
        packed |= (uint64_t)data[i] << (i * 8);
    }
    uint16_t u_ch2 = (packed >> 0)  & 0x07FF;
    uint16_t u_ch3 = (packed >> 11) & 0x07FF;
    uint16_t u_dir = (packed >> 22) & 0x07FF;
    recv->dataNeaten.ch2 = (int16_t)(u_ch2 <= 1023 ? u_ch2 : u_ch2 - 2048);
    recv->dataNeaten.ch3 = (int16_t)(u_ch3 <= 1023 ? u_ch3 : u_ch3 - 2048);
    recv->dataNeaten.dir = (int16_t)(u_dir <= 1023 ? u_dir : u_dir - 2048);
    recv->dataNeaten.pitch = (int8_t)((packed >> 33) & 0xFF);
    recv->dataNeaten.s1 = (packed >> 41) & 0x03;
    recv->dataNeaten.s2 = (packed >> 43) & 0x03;
    recv->dataNeaten.mouseL = (packed >> 45) & 0x03;
    recv->dataNeaten.mouseR = (packed >> 47) & 0x03;
	recv->dataNeaten.mouse_X_FIT = (float)((packed >> 49) & 0x03) - 1.0f;
    recv->dataNeaten.mouse_Y_FIT = (float)((packed >> 51) & 0x03) - 1.0f;
    recv->dataNeaten.robot_level = (packed >> 53) & 0x0f;
    recv->dataNeaten.status = (packed >> 57) & 0x07;
		
    WHW_V_DBUS.Remote.S1_u8 = recv->dataNeaten.s1;
    WHW_V_DBUS.Remote.S1_u8 = recv->dataNeaten.s1;
    WHW_V_DBUS.Remote.CH2_int16 = recv->dataNeaten.ch2;
    WHW_V_DBUS.Remote.CH3_int16 = recv->dataNeaten.ch3;
    WHW_V_DBUS.Remote.Dial_int16 = recv->dataNeaten.dir;
    WHW_V_DBUS.Mouse.L_State = recv->dataNeaten.mouseL;
    WHW_V_DBUS.Mouse.R_State = recv->dataNeaten.mouseR;
}