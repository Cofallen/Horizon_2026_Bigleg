#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "usbd_cdc_if.h"

typedef union
{
  uint8_t Data[4];
  float Data_f;
  uint32_t Data_u32;
}VisionTemp;


typedef struct
{
    uint8_t OriginData[21];
    struct Data
    {
      uint8_t Head_frame;
      uint8_t End_frame;
      float PitchAngle;
      float YawAngle;
      float PitchOmega;
      float YawOmega;
      float VisionTime;
      uint16_t OffCounter; // 在线检测
      uint8_t  isOnline;
      uint8_t target_visible;
    } Data;
}VisionRxDataUnion;

typedef struct
{
  uint8_t data[16];

  uint8_t Head_frame;
  float PitchAngle;
  float YawAngle;
  uint32_t VisionTime;
  uint8_t VisionState : 3;
  uint8_t Rate_of_fire;
  uint8_t End_frame;
}VisionTxDataUnion;

int8_t Vision_Rx_Data(uint8_t* buffer, VisionRxDataUnion *VisionRx);
void Vision_Tx_Data(float PitchAngle, float YawAngle, uint32_t Time, uint8_t State, uint8_t Rate_of_fire);

#endif
