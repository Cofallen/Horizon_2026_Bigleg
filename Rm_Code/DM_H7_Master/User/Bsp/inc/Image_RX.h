#ifndef __IMAGE_RX_H
#define __IMAGE_RX_H

#include "main.h"
#include "Referee.h"
#include "MY_Define.h"

/*0x0302*/
typedef struct __packed
{
    float Pitch;
    float Roll;
    float Yaw;
    float X_length;
    float Y_length;
    float Z_length;
    uint8_t status;
    uint8_t CUP;
} Aix6_t;

/*0x0304*/
typedef struct __packed
{
    int16_t mouse_x;                    //鼠标 x 轴移动速度，负值标识向左移动
    int16_t mouse_y;                    //鼠标 y 轴移动速度，负值标识向下移动
    int16_t mouse_z;                    //鼠标滚轮移动速度，负值标识向后滚动
    int8_t left_button_down;            //鼠标左键是否按下：0 为未按下；1 为按下
    int8_t right_button_down;           //鼠标右键是否按下：0 为未按下，1 为按下
    uint16_t keyboard_value_W : 1;      //W 键
    uint16_t keyboard_value_S : 1;      //S 键 
    uint16_t keyboard_value_A : 1;      //A 键 
    uint16_t keyboard_value_D : 1;      //D 键 
    uint16_t keyboard_value_Shift : 1;  //Shift 键 
    uint16_t keyboard_value_Ctrl : 1;   //Ctrl 键 
    uint16_t keyboard_value_Q : 1;      //Q 键 
    uint16_t keyboard_value_E : 1;      //E 键 
    uint16_t keyboard_value_R : 1;      //R 键
    uint16_t keyboard_value_F : 1;      //F 键
    uint16_t keyboard_value_G : 1;      //G 键 
    uint16_t keyboard_value_Z : 1;      //Z 键 
    uint16_t keyboard_value_X : 1;      //X 键 
    uint16_t keyboard_value_C : 1;      //C 键 
    uint16_t keyboard_value_V : 1;      //V 键 
    uint16_t keyboard_value_B : 1;      //B 键 
    uint16_t reserved;                  //保留位 

}remote_control_t;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef FrameHeader;
        uint16_t CmdId;//命令码
        Aix6_t Aix6;
        uint16_t CRC16;
        uint16_t ONLINE_JUDGE_TIME_0x302;
    }channal;
    uint8_t Data[35];
	
}Data_RX_0x302_t;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef FrameHeader;
        uint16_t CmdId;//命令码
        remote_control_t remote;
        uint16_t CRC16;
    }channal;
    uint8_t Data[19];
	
}Data_RX_0x304_t;

typedef union
{
	frame_header_R_Typdef FrameHeader;
	uint8_t Data[255];
}select_t;

void Read_Data_second(select_t *sel, Data_RX_0x302_t *RX_0x302, Data_RX_0x304_t *RX_0x304);

#endif
