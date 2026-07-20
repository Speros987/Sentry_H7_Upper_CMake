#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "imu_temp_ctrl.h"

#include <math.h>
#include <stdbool.h>

#define cheat TRUE
#define correct_Time_define 1
#define temp_times 1
#define RefTemp 40

INS_t INS;

float gyro_correct[3] = {0};
uint8_t attitude_flag = 0;
uint32_t correct_times = 0;

void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0);

    while (BMI088_init())
    {
        osDelay(10);
    }
}

void INS_Task(void)
{
    static uint32_t count = 0;

    if ((count % 1) == 0)
    {
        BMI088_read(INS.gyro, INS.accel, &INS.temp);

        if (attitude_flag == 2)
        {
            gyro_correct[0] = -0.0013251846;
            gyro_correct[1] = -0.00224610348;
            gyro_correct[2] = 0.000502272975;

            INS.gyro[0] -= gyro_correct[0];
            INS.gyro[1] -= gyro_correct[1];
            INS.gyro[2] -= gyro_correct[2];

#if cheat
            if (fabsf(INS.gyro[2]) < 0.003f)
            {
                INS.gyro[2] = 0;
            }
#endif
			IMU_QuaternionEKF_Update(INS.gyro[1],-INS.gyro[0],INS.gyro[2],INS.accel[1],-INS.accel[0],INS.accel[2]);
            INS.pitch = Get_Pitch();
            INS.roll = Get_Roll();
            INS.yaw = Get_Yaw();
        }
        else if (attitude_flag == 1)
        {
            gyro_correct[0] += INS.gyro[0];
            gyro_correct[1] += INS.gyro[1];
            gyro_correct[2] += INS.gyro[2];
            correct_times++;

            if (correct_times >= correct_Time_define)
            {
                gyro_correct[0] /= correct_Time_define;
                gyro_correct[1] /= correct_Time_define;
                gyro_correct[2] /= correct_Time_define;
                attitude_flag = 2;
            }
        }
    }

    if ((count % 10) == 0)
    {
        static uint32_t temp_Ticks = 0;
        if ((fabsf(INS.temp - RefTemp) < 30.0f) && attitude_flag == 0)
        {
            temp_Ticks++;
            if (temp_Ticks > temp_times)
            {
                attitude_flag = 1;
            }
        }
    }

    count++;
}

void OS_IMUCallback(void const *argument)
{
    INS_Init();

    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}
