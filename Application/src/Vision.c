#include "vision.h"
#include "Shooter.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "chassis.h"
#include "gimbal.h"
#include "imu_temp_ctrl.h"
#include "shooter.h"
#include "chassis.h"
#include "Judge.h"
#include "USER_B2B.h"
#include <stdint.h>

VisionTransmit vision_transmit;
VisionReceive vision_receive;
Vision_t vision;

void Vision_Init(void)
{
	vision_transmit.header = VISION_FRAME_HEADER_TX;
}   

//接收来自视觉的信息
void Vision_DataReceive(uint8_t *read_from_usart, uint32_t length)
{
	if (read_from_usart == NULL)
		return;
	// 查找帧头
	while (length) 
	{
		if (*read_from_usart != VISION_FRAME_HEADER_RX) 
		{
			++read_from_usart;
			--length;
		}
		else
		{
			break;
		}
	}
	if (length == 0)
		return;
	//判断帧头数据是否正确
	if(read_from_usart[0] == VISION_FRAME_HEADER_RX)
	{
		//将数据存入接收buffer
		memcpy(&vision_receive, read_from_usart, sizeof(vision_receive));
		Vision_ParseData();
	}
}

//对发送的数据更新
void Vision_DataUpdate(void)
{	
	vision_transmit.header = 0x5A;
	vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
	vision_transmit.mode = vision_receive.armor_mode; //0为打车 1为打前哨 2为打符
	vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
	vision_transmit.pitch = INS.pitch;
	vision_transmit.roll = INS.roll;
	vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
  	vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;
	vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
	vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
	memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));
	vision_transmit.see_enemy = vision.tracking;
	vision_transmit.end_frame = 0xA5;
}



void Vision_ParseData(void)
{
		vision.pitch = vision_receive.pitch;
		vision.base_yaw = vision_receive.base_yaw;
		vision.top_yaw = vision_receive.top_yaw;
		vision.tracking = vision_receive.tracking;
		vision.distance = vision_receive.distance;//视觉部分解包	
	
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

		if(vision.tracking && shooter.fricOpenFlag==1 && gimbal.visionEnable)
		{		
			float a = INS.yaw / 180.0f * PI - vision.top_yaw / 180 * PI, b = vision_receive.incident_yaw ; //a是云台角度和目标角度error  b是装甲板的入射角
			float D = vision.distance, r = vision_receive.armor_radius;
			
			if(vision.tracking == 1)//此处为电控装甲板火控 判断是否瞄到装甲板
			{
				float rcosb = r * cos(b), rsinb = r * sin(b), tana = tan(a);
				
				if (tana < rcosb / (D - rsinb) && tana > -rcosb / (D + rsinb))
				{
					shooter.workState=TRIGGER_CONTINUE;
				}
				else
				{
					shooter.workState=IDLE;
				}
			}
			else //此处为电控符火控 判断是否瞄到符
			{
				static uint8_t last_rune_number = 0;
				static uint32_t last_trigger_tick = 0;
				static bool allows_trigger = false;

				allows_trigger = allows_trigger
					|| vision_receive.rune_number != last_rune_number //如果rune_number变化了 就允许开火
					|| HAL_GetTick() - last_trigger_tick > 700; //如果上次开火已经超过1秒了 就允许开火
				
				last_rune_number = vision_receive.rune_number;

				if (allows_trigger)
				{
					float R = D / cos(a - b), y = R * sin(a), x = R * cos(b);

					if (hypotf(y, x * tan(INS.pitch / 180.0f * PI) - D * tan(vision.pitch / 180.0f * PI)) <= r)
					{
						shooter.workState=TRIGGER_CLICK;
						last_trigger_tick = HAL_GetTick();
						allows_trigger = false;
					}
					else
					{
						shooter.workState=IDLE;
					}
				}
			}
		}
	
		else
		{
			shooter.workState=IDLE;
		}
}


void Vision_DataTransmit(void)
{	
	Vision_DataUpdate();
	CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

void OS_VisionCallback(void const * argument)
{
	

	for(;;)
	{
		Vision_DataTransmit();
		osDelay(1);
	}
}
