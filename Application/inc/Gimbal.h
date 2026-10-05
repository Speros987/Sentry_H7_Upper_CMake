#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "USER_Moto.h"
#include "stdbool.h"
#include "slope.h"
#include "PID.h"
#include "Filter.h"

#define TOP_YAW_OFFSET 2710  //此处校准小yaw
#define INIT_YAW_ANGLE -85.0f   //此处校准大yaw
#define PITCH_MOTOR_ZERO_POS 2.094f //此处校准pitch

#define PITCH_DIRECTION -1
#define MASS_G 9.81f
#define PITCH_MASS 1.3f
#define PITCH_R 0.07287f   // pitch中心及其距离

typedef struct
{
  struct
  {
    float initAngle;                                // yaw
    float angle, lastAngle, totalAngle, totalRound; // 用于角度统计
    float gyro;
    float targetAngle, lastTargetAngle;
    CascadePID imuPID; // yaw陀螺仪pid
  } base_yaw;
  struct
  {
    float initAngle;                                // yaw

    // 用于角度统计
    float angle;//单位：°
    float lastAngle;//单位：°
    float totalAngle;//单位：°
    float totalRound;//单位：°

    float gyro;
    float targetAngle, lastTargetAngle;
    CascadePID imuPID; // yaw陀螺仪pid
  } top_yaw;
  struct
  {
    float initAngle;
    float angle, lastAngle; // pitch
    float gyro;
    float targetAngle, lastTargetAngle;
    float pitchMax, pitchMin; // 限幅
    PID MIT_PID;
    CascadePID imuPID;        // pitch陀螺仪pid
  } pitch;
	DM_motor_t base_yawMotor; // 大yaw电机
	DJI_Motor_t top_yawMotor; // 小yaw电机
	DM_motor_t pitchMotor;// 云台pitch电机
	
	struct
  {
		AverFilter pitch;//pitch滑块均值滤波器
    AverFilter yaw;//yaw滑块均值滤波器
    AverFilter find;//find滑块均值滤波器
	}visionFilter; //视觉数据滤波器

	bool visionEnable; // 是否视觉控云台，滚轮
	bool scan_flag;
} Gimbal_t;

extern Gimbal_t gimbal;
extern float p_target;

void Gimbal_Init(void);


#endif
