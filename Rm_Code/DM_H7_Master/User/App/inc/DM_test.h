#ifndef DM_TEST_H
#define DM_TEST_H

#include "DM_Motor.h"
#include "IMU_Task.h"

extern DM_MOTOR_Typedef DM_Motor_Test;

void DM_test_init(void);
void DM_test(IMU_Data_t *imu_data);

#endif /* DM_TEST_H */
