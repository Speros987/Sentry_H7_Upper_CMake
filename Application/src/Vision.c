#include "vision.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "chassis.h"
#include "gimbal.h"
#include "imu_temp_ctrl.h"
#include "shooter.h"
#include "chassis.h"
#include "Judge.h"
#include "USER_B2B.h"

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
	while (length) {
		if (*read_from_usart != VISION_FRAME_HEADER_RX) {
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
	vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
	vision_transmit.pitch = INS.pitch;
	vision_transmit.roll = INS.roll;
	vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
  	vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;
	vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
	vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
	vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
	memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));
	vision_transmit.end_frame = 0xA5;
}



void Vision_ParseData(void)
{
		vision.pitch = vision_receive.pitch;
		vision.base_yaw = vision_receive.base_yaw;
		vision.top_yaw = vision_receive.top_yaw;
		vision.found = vision_receive.tracking;
		vision.distance = vision_receive.distance;//视觉部分解包	
	
		vx=-vision_receive.linear_y*1000; //ai部分解包 同时转换坐标系 ai坐标系下向前为x 左右为y 电控坐标系下 左右为x 前后为y
		vy=-vision_receive.linear_x*1000;
		vw=vision_receive.angular_z;
		USER_SentryCmd.sentry_mode = vision_receive.sentry_mode;
		USER_SentryCmd.energy_activation = vision_receive.energy_activation;
		USER_SentryCmd.survive = vision_receive.survive;
		USER_SentryCmd.buy_bullet = vision_receive.buy_bullet;

		if(vision_receive.tracking && shooter.fricOpenFlag==1 && gimbal.visionEnable)
		{
				float D = vision.distance, R = 0 /*vision_receive.spin_radius*/, r = vision_receive.armor_radius;//0.108 : 0.061
				float a = INS.yaw / 180.0f * PI - vision.top_yaw / 180 * PI, b = vision_receive.armor_yaw ; //a是云台角度和目标角度error  b是装甲板的入射角
				float rcosb = r * cos(b), rsinb = r * sin(b), Rcosb = R * cos(b), Rsinb = R * sin(b), tana = tan(a);
				
				if (tana < (rcosb - Rsinb) / (D - Rcosb - rsinb) &&
				    tana > (-rcosb - Rsinb) / (D - Rcosb + rsinb))
				{	
						shooter.workState=TRIGGER_CONTINUE;
				}
				else
				{
						shooter.workState=IDLE;
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
	Vision_Init();

	for(;;)
	{
		Vision_DataTransmit();
		osDelay(1);
	}
}
