#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "main.h"
#include "pid.h"
#include "Slope.h"
#include "USER_Moto.h"

#define WHEELSPEED_MAX 8000

typedef enum
{
	ChassisMode_Follow,	  // 底盘跟随云台模式
	ChassisMode_Spin,	  // 小陀螺模式
} ChassisMode;

typedef enum{
    Chassis_control,
    Chassis_AI
}chassis_pattern;

typedef struct _Chassis
{
	// 底盘尺寸信息
	struct Info
	{
		float wheelbase;   // 轴距
		float wheeltrack;  // 轮距
		float wheelRadius; // 轮半径
		float offsetX;	   // 重心在xy轴上的偏移
		float offsetY;
	} info;
	// 8个电机
	Double_motor_t motors[4];
	
	// 底盘移动信息
	struct Move
	{
		float vx; // 当前左右平移速度 mm/s
		float vy; // 当前前后移动速度 mm/s
		float vw; // 当前旋转速度 rad/s

		float maxVx, maxVy, maxVw; // 三个分量最大速度

		float Wheelangle[4];
		Slope xSlope, ySlope, outputSlope, chargeSlope, spinSlope; // 斜坡
	} move;
	
	// 旋转相关信息
	struct
	{
		PID pid;				// 旋转PID，由relativeAngle计算底盘旋转速度
		float relativeAngle;	// 云台与底盘的偏离角 单位度
    	float align_yaw; 		//ai传来云台与垂直起伏路段方向的角度 用于底盘与起伏路段对齐
		float InitAngle;		// 云台与底盘对齐时的编码器度数 
		int16_t InitpitchAngle; // 云台水平时编码器值
		float nowAngle;			// 此时云台的编码器换算为°值
		ChassisMode mode;		// 底盘模式 小陀螺或者底盘跟随
	} rotate;
	chassis_pattern pattern; // 模式
} Chassis_t;

extern Chassis_t chassis;
extern float vx,vy,vw;

void Chassis_InitPID(void);

#endif
