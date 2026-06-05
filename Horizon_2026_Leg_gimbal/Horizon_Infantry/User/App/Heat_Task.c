#include "Heat_Task.h"
#include "Shoot_Task.h"

static const HeatConfig_t heat_table[11] =
{
    {170, 5},
    {180, 7},
    {190, 9},
    {200, 11},
    {210, 12},
    {220, 13},
    {230, 14},
    {240, 16},
    {250, 18},
    {260, 20},
    {0, 0}
};

Heat_t g_heat = {0};

static inline float clampf(
    float x,
    float min,
    float max)
{
    if (x < min)
        return min;

    if (x > max)
        return max;

    return x;
}

void Heat_Init(uint8_t level)
{
    g_heat.heat = 100.0f;

    Heat_SetLevel(level);
}

void Heat_SetLevel(uint8_t level)
{
    if (level < 1)
        level = 1;

    if (level > 11)
        level = 11;

    g_heat.level = level;

    g_heat.heat_limit =
        heat_table[level - 1].heat_limit;

    g_heat.cooling_rate =
        heat_table[level - 1].cooling_rate;
}

void Heat_Update(uint8_t level, float dt)
{
    Heat_SetLevel(level);
    g_heat.heat +=
        g_heat.cooling_rate * dt;

    if(Shooter_IsFired(&shooter, &ALL_MOTOR))
    {
        g_heat.heat -= 10.0f;
    }

    g_heat.heat =
        clampf(
            g_heat.heat,
            0.0f,
            g_heat.heat_limit);
}
