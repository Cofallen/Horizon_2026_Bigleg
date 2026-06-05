#include "board2board.h"
#include "main.h"
#include "can_bsp.h"
#include "string.h"

boardRxData_t boardRxData = {0};
boardTxData_t boardTxData = {0};

static inline uint8_t encode_tri(int8_t v)
{
    return (uint8_t)(v + 1);   // -1,0,1 -> 0,1,2
}

static inline int8_t decode_tri(uint8_t v)
{
    return (int8_t)v - 1;
}

// 目标值 dbus: yaw pitch s1 s2
void Board_to_board_send(boardTxData_t* send, int16_t ch2, int16_t ch3, int16_t dir, uint8_t s1, uint8_t s2, uint8_t mouseL, uint8_t mouseR, 
                         int8_t mouse_X_fit, int8_t mouse_Y_fit, uint8_t robot_level, uint8_t status, float pitch)
{
    uint64_t packed = 0;
    packed |= (uint64_t)(ch2 & 0x07ff) << 0;
    packed |= (uint64_t)(ch3 & 0x07ff) << 11;
    packed |= (uint64_t)(dir & 0x07ff) << 22;
    packed |= (uint64_t)((int8_t)(pitch) & 0xff) << 33;
    packed |= (uint64_t)(s1 & 0x03) << 41;
    packed |= (uint64_t)(s2 & 0x03) << 43;
    packed |= (uint64_t)(mouseL & 0x03) << 45;
    packed |= (uint64_t)(mouseR & 0x03) << 47;
    packed |= (uint64_t)(encode_tri(mouse_X_fit) & 0x03) << 49;
    packed |= (uint64_t)(encode_tri(mouse_Y_fit) & 0x03) << 51;
    packed |= (uint64_t)(robot_level & 0x0f) << 53;
    packed |= (uint64_t)(status & 0x07) << 57;

    for (int i = 0; i < 8; i++) {
        send->sendData[i] = (packed >> (i * 8)) & 0xFF;
    }
    // canx_send_data(&hfdcan3, 0x1A, send->sendData, sizeof(send->sendData));
    canx_send_data(&hfdcan3, 0x1A, send->sendData, sizeof(send->sendData));
    // canx_send_data(&hfdcan3, 0x200, send->sendData, sizeof(send->sendData));
}

// 上板 imu: yaw
void Board_to_board_recv(boardRxData_t* recv, uint8_t *data)
{
    // memcpy(recv, data, 8);
    uint32_t tmp = (data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24);
    recv->dataNeaten.yaw_imu = *(float*) &tmp;
}