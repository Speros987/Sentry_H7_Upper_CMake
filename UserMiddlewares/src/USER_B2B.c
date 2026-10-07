#include "USER_B2B.h"
#include "Chassis.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "Gimbal.h"
#include <string.h>
#include "Vision.h"
#include "Judge.h"

extern DMA_HandleTypeDef hdma_usart2_rx;
float chassis_yaw = 0;

/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;   //是1下板急停
uint8_t FEEDBACK = 0;		//是0下板急停

uint32_t receive_times;

/* 需要用到的发送变量*/
uint8_t usart2TxBuf[64];

// 板间通信初始化
void B2B_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void B2B_Transmit()
{
	usart2TxBuf[0] = 0xAA;				 // 帧头
	for(uint8_t i = 0; i < 4; i++)
	{
		float v = chassis.motors[i].targetTurnAngle - chassis.motors[i].now_angle;   // 要发送的那个 float
		uint8_t *p = (uint8_t *)&v;
		usart2TxBuf[1 + 4*i] = p[0];
		usart2TxBuf[2 + 4*i] = p[1];
		usart2TxBuf[3 + 4*i] = p[2];
		usart2TxBuf[4 + 4*i] = p[3];
	} //1-16 舵电机误差角度
	for (uint8_t i = 0; i < 4; i++)
	{
		usart2TxBuf[17 + i * 2] = chassis.motors[i].targetDriveSpeed;
		usart2TxBuf[17 + i * 2 + 1] = chassis.motors[i].targetDriveSpeed >> 8;
	} // 17-24 轮电机目标速度
	{
		float v = gimbal.base_yaw.imuPID.output;   // 要发送的那个 float
		v = gimbal.base_yaw.imuPID.outer.output;
		uint8_t *p = (uint8_t *)&v;
		usart2TxBuf[25] = p[0];
		usart2TxBuf[26] = p[1];
		usart2TxBuf[27] = p[2];
		usart2TxBuf[28] = p[3];
	}	//25-28 yaw电机目标速度

	memcpy(&usart2TxBuf[29], &USER_SentryCmd, sizeof(USER_SentryCmd_t)); //29-35 哨兵自主指令

	usart2TxBuf[62]  = STOPFLAG;    // 急停标志
	usart2TxBuf[63] = 0xFE;				 // 帧尾

	HAL_UART_Transmit_DMA(&huart2, usart2TxBuf, sizeof(usart2TxBuf));
}


void B2B_Receive(void)
{
    if (usart2RxBuf[0] == 0xAB && usart2RxBuf[63] == 0xFD)
	{
	    for(uint8_t i = 0; i < 4; i++)
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v;
		    p[0] = usart2RxBuf[1 + 4*i];
		    p[1] = usart2RxBuf[2 + 4*i];
		    p[2] = usart2RxBuf[3 + 4*i];
		    p[3] = usart2RxBuf[4 + 4*i];
		    chassis.motors[i].TurnAngle = v;
		} //解析舵电机当前角度（单圈）单位° 1-16

	    for (uint8_t i = 0; i < 4; i++)
		{
		    chassis.motors[i].now_Speed = (int16_t)usart2RxBuf[17 + i * 2] | (int16_t)usart2RxBuf[17 + i * 2 + 1] << 8;
		} //轮电机当前速度	17-24 //轮电机当前速度    17-24
		{
		    float v;
		    uint8_t *p = (uint8_t *)&v;
		    p[0] = usart2RxBuf[25];
		    p[1] = usart2RxBuf[26];
		    p[2] = usart2RxBuf[27];
		    p[3] = usart2RxBuf[28];
		    gimbal.base_yawMotor.nowAngle = v;
		}	//解析大yaw电机当前角度（单圈）单位° 25-28
		// {
		// 	    float v;
		// 	    uint8_t *p = (uint8_t *)&v;
		// 	    p[0] = usart2RxBuf[29];
		// 	    p[1] = usart2RxBuf[30];
		// 	    p[2] = usart2RxBuf[31];
		// 	    p[3] = usart2RxBuf[32];
		// 	    gimbal.base_yawMotor.para.vel = v;
		// }	//解析大yaw电机当前速度  单位rad/s 29-32
	    // 		float v;
	    // 		uint8_t *p = (uint8_t *)&v;
	    // 		p[0] = usart2RxBuf[29];
	    // 		p[1] = usart2RxBuf[30];
	    // 		p[2] = usart2RxBuf[31];
	    // 		p[3] = usart2RxBuf[32];
	    // 		gimbal.base_yawMotor.para.vel = v;
	    memcpy(&USER_JudgeData, &usart2RxBuf[29], sizeof(JudgeData_t)); //解析裁判系统数据 29-58

	    FEEDBACK = usart2RxBuf[62];
	}
}

/************************freertos任务****************************/
void OS_Board2BoardCallback(void const *argument)
{
    while (1)
	{
	    B2B_Transmit();
	    osDelay(1);
	}
}
