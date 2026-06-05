#ifndef __HEAT_TASK_H
#define __HEAT_TASK_H

#include <stdint.h>

typedef struct
{
    uint16_t heat_limit;

    uint16_t cooling_rate;

} HeatConfig_t;

typedef struct
{
    float heat;

    float heat_limit;

    float cooling_rate;

    uint8_t level;

} Heat_t;

extern Heat_t g_heat;

void Heat_Init(uint8_t level);

void Heat_SetLevel(uint8_t level);

void Heat_Update(uint8_t level, float dt);


#endif
