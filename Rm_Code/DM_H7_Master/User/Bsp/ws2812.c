#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0Ã‚Ã«
#define WS2812_HighLevel   0xF0     // 1Ã‚Ã«

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

void V0_V1(int8_t *tmp)
{
    if(*tmp != 0)   *tmp = 1;
    if(*tmp == 0)   *tmp = 0;
}

void RGB_Ctrl(uint8_t COLOR, float TIME, uint8_t Power)
{
    float TIME_NOW = DWT_GetTimeline_ms();
    static float TIME_LAST = 0;

    static int8_t BIT = 0;

    static uint8_t LED_CONFIG[7][3] ={
        {1, 0, 0},    // RED
        {0, 1, 0},    // GREEN
        {0, 0, 1},    // BLUE
        {1, 1, 0},    // YELLOW
        {1, 0, 1},    // PURPLE
        {0, 1, 1},    // CYAN
        {1, 1, 1},    // write
    };
	
    if ((TIME_NOW - TIME_LAST) > TIME)
    {
        TIME_LAST = TIME_NOW;
		BIT = !BIT;
        V0_V1(&BIT);
    }

    WS2812_Ctrl(LED_CONFIG[COLOR][0]*Power*BIT,
                LED_CONFIG[COLOR][1]*Power*BIT,
                LED_CONFIG[COLOR][2]*Power*BIT);
}
