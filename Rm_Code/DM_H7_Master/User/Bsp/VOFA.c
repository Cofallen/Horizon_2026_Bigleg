/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-04-09 13:26:42
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:40:21
 */
#include "VOFA.h"
#include "usart.h"

 union vofa_Transmit
  {
    float data1[10];
    uint8_t data2[44];
  }data;

/**
 * @brief VOFA+使用串口发送justfloat格式数据
 * 
 * @param a 
 * @param b 
 * @param c 
 * @param d 
 * @param e 
 * @param f 
 * @param g 
 * @param h 
 * @param j 
 * @param k 
 */
void VOFA_justfloat(float a,float b,float c,float d,float e,float f,float g,float h,float j,float k)
{
   uint8_t i=0;
   data.data1[i++]=a;
   data.data1[i++]=b;
   data.data1[i++]=c;
   data.data1[i++]=d;
   data.data1[i++]=e;
   data.data1[i++]=f;
   data.data1[i++]=g;
   data.data1[i++]=h;
   data.data1[i++]=j;
   data.data1[i++]=k;
   data.data2[40]=0x00;
   data.data2[41]=0x00;
   data.data2[42]=0x80;
   data.data2[43]=0x7f;
  HAL_UART_Transmit_DMA(&huart1,data.data2,44);
  //  CDC_Transmit_HS(data.data2,44);
}


union vofa_Transmit_ML
  {
    float data1[12];
    uint8_t data2[52];
  }dataML;

void VOFA_justfloat_ML(float a,float b,float c,float d,float e,float f,float g,float h,float j,float k, float l, float m)
{
   uint8_t i=0;
   dataML.data1[i++]=a;
   dataML.data1[i++]=b;
   dataML.data1[i++]=c;
   dataML.data1[i++]=d;
   dataML.data1[i++]=e;
   dataML.data1[i++]=f;
   dataML.data1[i++]=g;
   dataML.data1[i++]=h;
   dataML.data1[i++]=j;
   dataML.data1[i++]=k;
   dataML.data1[i++]=l;
   dataML.data1[i++]=m;
   dataML.data2[48]=0x00;
   dataML.data2[49]=0x00;
   dataML.data2[50]=0x80;
   dataML.data2[51]=0x7f;
  HAL_UART_Transmit_DMA(&huart1,dataML.data2,52);
  //  CDC_Transmit_HS(dataML.data2,52);
}