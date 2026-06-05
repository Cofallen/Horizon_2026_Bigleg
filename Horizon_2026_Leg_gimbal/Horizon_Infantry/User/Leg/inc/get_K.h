#ifndef __GET_K_H
#define __GET_K_H

#include "main.h"

typedef union 
{
    struct __packed
    {
        float yaw_imu;
    } dataNeaten;
    uint8_t txData[8];
}boardTxData_t;

typedef union 
{
    struct __packed
    {
        int16_t ch2;
        int16_t ch3;
        int16_t dir;
        int16_t pitch;  // 100倍
        uint8_t s1;
        uint8_t s2;
        uint8_t mouseL;
        uint8_t mouseR;
        float mouse_X_FIT;
        float mouse_Y_FIT;
        uint8_t robot_level;
        uint8_t status;
    } dataNeaten;
    // uint8_t rxData[8];
}boardRxData_t;

void Board_to_board_send(boardTxData_t *send, float yaw);
void Board_to_board_recv(boardRxData_t *recv, uint8_t *data);

extern boardTxData_t BoardTXData;
extern boardRxData_t boardRxData;
#endif // !__GET_K_H