#ifndef __BOARD2BOARD_H
#define __BOARD2BOARD_H

#include "main.h"

typedef union 
{
    struct __packed
    {
        int16_t ch2: 11;
        int16_t ch3: 11;
        int16_t dir: 11;
        int16_t pitch :16;
        uint8_t s1: 2;
        uint8_t s2: 2;
    } dataNeaten;
    uint8_t sendData[8];
}boardTxData_t;

typedef union 
{
    struct __packed
    {
        float yaw_imu;
    } dataNeaten;
    uint8_t rxData[8];
}boardRxData_t;
void Board_to_board_send(boardTxData_t* send, int16_t ch2, int16_t ch3, int16_t dir, uint8_t s1, uint8_t s2, uint8_t mouseL, uint8_t mouseR, 
                         int8_t mouse_X_fit, int8_t mouse_Y_fit, uint8_t robot_level, uint8_t status, float pitch);
void Board_to_board_recv(boardRxData_t* recv, uint8_t *data);

extern boardRxData_t boardRxData;
extern boardTxData_t boardTxData;

#endif // !__BOARD2BOARD_H