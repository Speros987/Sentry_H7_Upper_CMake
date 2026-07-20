#include "Vision.h"
#include "Shooter.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "Shooter.h"
#include "Judge.h"
#include "arm_math.h"
#include "USER_B2B.h"

#include <stdint.h>
#include <float.h>

VisionTransmit vision_transmit;
VisionReceive vision_receive;
VisionTarget vision_target;

void Vision_Init(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;
}

//接收来自视觉的信息
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
    if (read_from_usart == NULL)
        return;
    
    while (length >= sizeof(vision_receive))
    {
        if (*read_from_usart == VISION_FRAME_HEADER_RX)
        {
            memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
            Vision_ParseData();
            return;
        }
        
        ++read_from_usart;
        --length;
    }
}

//对发送的数据更新
void Vision_DataUpdate(void)
{    
    vision_transmit.header = 0x5A;
    vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
    vision_transmit.mode = 0; //0为打装甲板 1为打符
    vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
    vision_transmit.pitch = INS.pitch;
    vision_transmit.roll = INS.roll;
    vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
    vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;
    vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
    vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
    memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));
    vision_transmit.see_enemy = vision_target.vaild;
    vision_transmit.end_frame = 0xA5;
}

void Vision_ParseData(void)
{
    vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y
    vy=-vision_receive.linear_x*1000;
    vw=vision_receive.angular_z;
    chassis.rotate.align_yaw = vision_receive.align_yaw;
    USER_SentryCmd.sentry_mode = vision_receive.sentry_mode;
    USER_SentryCmd.energy_activation = vision_receive.energy_activation;
    USER_SentryCmd.buy_projectile = vision_receive.buy_projectile;
    USER_SentryCmd.buy_life = vision_receive.buy_life;
    USER_SentryCmd.remote_buy_bullet = vision_receive.remote_buy_bullet;
    USER_SentryCmd.remote_buy_blood = vision_receive.remote_buy_blood;

    if (vision_receive.mode == VISION_RECEIVE_ARMOR)
    {
        vision_target.vaild = true;
        vision_target.base_yaw = vision_receive.armor.base_yaw;
        vision_target.yaw = vision_receive.armor.yaw;
        vision_target.pitch = vision_receive.armor.pitch;

        float a = INS.yaw / 180.0f * PI - vision_receive.armor.yaw / 180 * PI, b = vision_receive.armor.incident_yaw;
        //a是云台角度和目标角度error  b是装甲板的入射角
        float D = vision_receive.armor.distance, r = vision_receive.armor.hit_radius;
        float rcosb = r * cos(b), rsinb = r * sin(b), tana = tan(a);
        
        if (tana < rcosb / (D - rsinb) && tana > -rcosb / (D + rsinb))
        {
            shooter.workState = TRIGGER_CONTINUE;
        }
        else
        {
            shooter.workState = IDLE;
        }
    }
    else if (vision_receive.mode == VISION_RECEIVE_RUNE)
    {
        static uint32_t last_trigger_tick[5];

        uint32_t current_tick = HAL_GetTick();
        int target_i = -1;
        float min_angle_distance = FLT_MAX;
        
        for (int i = 0; i < 2; ++i)
        {
            int j = vision_receive.rune.index[i];

            if (j != -1 && current_tick - last_trigger_tick[j] >= 1000 * vision_receive.rune.hit_time[i])
            {
                float angle_distance = hypotf(INS.yaw - vision_receive.rune.yaw[i], INS.pitch - vision_receive.rune.pitch[i]);

                if (angle_distance < min_angle_distance)
                {
                    target_i = i;
                    min_angle_distance = angle_distance;
                }
            }
        }

        if (target_i != -1)
        {
            vision_target.vaild = true;
            vision_target.base_yaw = vision_receive.rune.base_yaw;
            vision_target.yaw = vision_receive.rune.yaw[target_i];
            vision_target.pitch = vision_receive.rune.pitch[target_i];

            float a = INS.yaw / 180.0f * PI - vision_receive.rune.yaw[target_i] / 180 * PI;
            float b = vision_receive.rune.incident_yaw[target_i];
            //a是云台角度和目标角度error  b是符的入射角
            float D = vision_receive.rune.distance[target_i], r = vision_receive.rune.hit_radius;
            float R = D / cos(a - b), y = R * sin(a), x = R * cos(b);

            if (hypotf(y, x * tan(INS.pitch / 180.0f * PI) - D * tan(vision_receive.rune.pitch[target_i] / 180.0f * PI)) <= r)
            {
                shooter.workState = TRIGGER;
                last_trigger_tick[vision_receive.rune.index[target_i]] = current_tick;
            }
            else
            {
                shooter.workState = IDLE;
            }
        }
        else
        {
            vision_target.vaild = false;
            shooter.workState = IDLE;
        }
    }
    else
    {
        vision_target.vaild = false;
        shooter.workState = IDLE;
    }
}

void Vision_DataTransmit(void)
{    
    Vision_DataUpdate();
    CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

void OS_VisionCallback(void const * argument)
{
    Vision_Init();

    for(;;)
    {
        Vision_DataTransmit();
        osDelay(1);
    }
}
