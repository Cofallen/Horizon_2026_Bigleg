#include "bsp_imu_pwm.h"

void SPI1_imu_pwm_set(uint16_t pwm)
{
    TIM3->CCR4 = (pwm);
}
