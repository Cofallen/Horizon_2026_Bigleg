#include "Image_RX.h"

void Read_Data_second(select_t *sel, Data_RX_0x302_t *RX_0x302, Data_RX_0x304_t *RX_0x304)
{
	if((sel->FrameHeader.SOF == (uint16_t)0xA5)
        && (Verify_CRC8_Check_Sum(sel->Data , 5) == 1))
    {
		//0x302
		if(Verify_CRC16_Check_Sum(sel->Data, 35)&&(sel->Data[5] == 0x02))
		{
			memcpy(RX_0x302->Data, sel->Data, 35);
                        RX_0x302->channal.ONLINE_JUDGE_TIME_0x302 = DF_0x302_OFFLINE_TIME;
		}
		
		//0x304
		if(Verify_CRC16_Check_Sum(sel->Data, 19)&&(sel->Data[5] == 0x04))
		{
			memcpy(RX_0x304->Data, sel->Data, 19);
		}
	}
}

