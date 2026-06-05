#ifndef __CRC_DJI_H
#define __CRC_DJI_H
#include "main.h"

uint32_t Verify_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
void Append_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

extern const uint8_t CRC8_TAB[256];
extern const uint16_t wCRC_Table[256];

#endif
