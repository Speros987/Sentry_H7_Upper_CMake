#include "Chassis.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "USER_RC.h"
#include "arm_math.h"
#include "Gimbal.h"
#include "vision.h"
#include "degree.h"

#include <math.h>

float vx,vy,vw = 0;  //AI传来的旋转速度

Chassis_t chassis = {0};

/********************初始化************************/
void Chassis_Init()
{
	// 底盘尺寸信息（用于解算轮速）
	chassis.info.wheelbase = 320;
	chassis.info.wheeltrack = 320;
	chassis.info.wheelRadius = 115;
	chassis.info.offsetX = 0; // 15
	chassis.info.offsetY = 0; //-10
	// 移动参数初始化

	// 旋转参数初始化
	chassis.rotate.InitAngle = INIT_YAW_ANGLE; 
	chassis.rotate.InitpitchAngle = 1290; 

	chassis.motors[0].TurnOffset= -304.57 - 30 + 180; //
	chassis.motors[1].TurnOffset= -44.78 + 30 + 45 + 180;//
	chassis.motors[2].TurnOffset= -119.49 + 30 + 90 + 15 + 180;//  //此处校准舵电机 正常加减30°的倍数
	chassis.motors[3].TurnOffset= -6.5 + 90 - 15 + 180;//

	// 斜坡函数初始化
	Slope_Init(&chassis.move.xSlope, 40, 0);
	Slope_Init(&chassis.move.ySlope, 40, 0);
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);

	Chassis_InitPID();
}

void Chassis_InitPID()
{
	PID_Init(&chassis.rotate.pid, 0.25, 0, 6, 2, 15); // 15	PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15
	PID_SetDeadzone(&chassis.rotate.pid, 0.1);
}

void Spin_SpeedUpdate() //
{
	chassis.move.maxVw = 3.25f; // chassis.move.maxPower * 0.225f + 3.25f
	if (chassis.move.maxVw <= 0.5f)
	{
		chassis.move.maxVw = 0.5f;
	}

	if (chassis.rotate.mode == ChassisMode_Spin)
	{
		chassis.move.maxVx = 2159.6f;//0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f
		if (chassis.move.maxVx <= 0)
			chassis.move.maxVx = 0;
		chassis.move.maxVy = chassis.move.maxVx;
	}
	else
	{
		chassis.move.maxVx = 5500;
		chassis.move.maxVy = 5500;
	}
}

void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);
	Slope_NextVal(&chassis.move.ySlope);
	Slope_NextVal(&chassis.move.spinSlope);
	Spin_SpeedUpdate();
	float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;
	chassis.move.maxVx = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2 * PI * chassis.info.wheelRadius;
	chassis.move.maxVy = chassis.move.maxVx;
	chassis.move.maxVw = WHEELSPEED_MAX / rotateRatio / 60.0f / (268.f / 17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f;
}


void Chassis_ModeCtrl()
{
	// 左拨杆拨到下方进入小陀螺，拨回来进入跟随模式
	if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2 )
			chassis.rotate.mode = ChassisMode_Spin;
	else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
			chassis.rotate.mode = ChassisMode_Follow;
	
	switch(rcInfo.right)
	{
		case 3:
			chassis.pattern = Chassis_AI;
		break;
		case 1:
			chassis.pattern =Chassis_control;
		default:
			break;
	}

	if(chassis.pattern == Chassis_AI)
	{
		if(!vision_receive.spin_mode)//AI确定模式
			chassis.rotate.mode = ChassisMode_Spin;
		else
			chassis.rotate.mode = ChassisMode_Follow;
	}
}

// 底盘任务回调函数
void Task_Chassis_Callback()
{
    chassis.rotate.InitAngle = ModularDegreeTowards(INIT_YAW_ANGLE, 0.0f);
    
	chassis.rotate.nowAngle = gimbal.base_yawMotor.nowAngle;
	chassis.rotate.relativeAngle = chassis.rotate.nowAngle - chassis.rotate.InitAngle;  //解算云台与底盘的夹角 

	Chassis_ModeCtrl();

	if (chassis.pattern == Chassis_AI )
	{
		Slope_SetTarget(&chassis.move.xSlope,1.5f * vx); //x为前后
		Slope_SetTarget(&chassis.move.ySlope,1.5f * vy);
		float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180);
		float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180);
		chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);
		chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
		Chassis_UpdateSlope();

		if (vision_receive.spin_mode == 0)
			chassis.rotate.mode = ChassisMode_Follow;
		else
			chassis.rotate.mode = ChassisMode_Spin;	
	}
	else
	{
		Slope_SetTarget(&chassis.move.xSlope,-(float)rcInfo.ch3*chassis.move.maxVx/660);
		Slope_SetTarget(&chassis.move.ySlope,(float)rcInfo.ch4*chassis.move.maxVy/660);
		// 将云台坐标系下平移速度解算到底盘平移速度(根据云台偏离角)
		float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180);
		float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180);
		chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);
		chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);
		Chassis_UpdateSlope();
	}

		// 旋转相关内容
	if (chassis.rotate.mode == ChassisMode_Follow) 
	{
		Slope_SetTarget(&chassis.move.spinSlope, 0);
        chassis.rotate.relativeAngle = ModularDegreeTowards(chassis.rotate.relativeAngle, 0.0f);
		 
		if (chassis.pattern==Chassis_control)
		{
			PID_SingleCalc(&chassis.rotate.pid, 0, -chassis.rotate.relativeAngle);

			chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
			LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
           
		    if (chassis.pattern == Chassis_AI || Gimbal_VisionForced())
            {
                chassis.move.vw = 0;
            }
		}
		else
		{
            if (vision_receive.align_mode == 1)
            {
                float target = -chassis.rotate.align_yaw;
                float feedback = ModularDegreeTowards(-chassis.rotate.relativeAngle, target);

                PID_SingleCalc(&chassis.rotate.pid, target, feedback);
                chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
                LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
            else
            {
                chassis.move.vw = vw;
                LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
		}
	}
    else if (chassis.rotate.mode == ChassisMode_Spin) //小陀螺模式
    {
        chassis.move.vw = chassis.move.spinSlope.value;
        float ratio;

        if (chassis.pattern == Chassis_control)
        {
            if (fabsf(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + fabsf(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
                ratio = 0.6f;        
            else
                ratio = 1.0f;
            
            Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
        }
        else
        {
            ratio = 0.5;
        }
        
        Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
    }
 
//检测当前角度
	for (uint8_t i = 0; i < 4; i++)
	{
        chassis.motors[i].now_angle = ModularDegreeTowards(chassis.motors[i].TurnAngle - chassis.motors[i].TurnOffset, 0.0f);
	}
	
    /***解算各轮子转速****/
    float rotateRatio[4];
    rotateRatio[0] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotateRatio[1] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotateRatio[2] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    rotateRatio[3] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;
    
    float wheelvx[4], wheelvy[4];

    // 左前
    wheelvx[0] = chassis.move.vx + chassis.move.vw * rotateRatio[1] / 1.414f;
    wheelvy[0] = chassis.move.vy + chassis.move.vw * rotateRatio[1] / 1.414f;
    // 右前
    wheelvx[1] = chassis.move.vx + chassis.move.vw * rotateRatio[2] / 1.414f;
    wheelvy[1] = chassis.move.vy - chassis.move.vw * rotateRatio[2] / 1.414f;
    // 左后
    wheelvx[2] = chassis.move.vx - chassis.move.vw * rotateRatio[0] / 1.414f;
    wheelvy[2] = chassis.move.vy + chassis.move.vw * rotateRatio[0] / 1.414f;
    // 右后
    wheelvx[3] = chassis.move.vx - chassis.move.vw * rotateRatio[3] / 1.414f;
    wheelvy[3] = chassis.move.vy - chassis.move.vw * rotateRatio[3] / 1.414f;

    //    舵轮解算
    for (uint8_t i = 0; i < 4; i++)
    {
        float target_angle;
        int32_t wheelRPM = -hypotf(wheelvx[i], wheelvy[i]) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);

        if (wheelRPM != 0)
        {
            target_angle = atan2f(-wheelvy[i], wheelvx[i]) / PI * 180;
        }
        else //当目标速度为0 且电机速度已经减下来时  舵回到正常角度
        {
            static const float default_angle[4] = {135, 45, 45, 135};
            target_angle = default_angle[i];
        }

        target_angle = ModularDegreeTowards(target_angle, chassis.motors[i].now_angle);
        
        if (target_angle - chassis.motors[i].now_angle >= 90)
        {
            target_angle -= 180.0f;
            wheelRPM *= -1.0f;
        }
        else if (target_angle - chassis.motors[i].now_angle < -90)
        {
            target_angle += 180.0f;
            wheelRPM *= -1.0f;
        }

        chassis.motors[i].targetTurnAngle = target_angle;
        chassis.motors[i].targetDriveSpeed = wheelRPM;
    }
}


void OS_ChassisCallback(void const * argument)
{
	osDelay(500);
	Chassis_Init();
    for(;;)
    {
		Task_Chassis_Callback();
        osDelay(2);
    }
}
