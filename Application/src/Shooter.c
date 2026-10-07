#include "Shooter.h"
#include "bsp_can.h"
#include "Chassis.h"
#include "USER_RC.h"
#include "Beep.h"
#include "Vision.h"
#include "Gimbal.h"
#include "Judge.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

Shooter shooter;
uint32_t t = 0; // 线性火控延时

void Shooter_InitPID(void);

//射击系统初始化
void Shooter_Init()
{
    shooter.fricSpd = 6500; //
Shooter_InitPID();						// m初始化电机pid
    shooter.fricMotor[0].targetSpeed = -0;
    shooter.fricMotor[1].targetSpeed = +0;
    shooter.workState = IDLE;
}

void Shooter_InitPID()
{
Motor_StartCalcAngle(&shooter.triggerMotor);							 // 初始化电机角度累计
    PID_Init(&shooter.triggerMotor.anglePID.inner, 6, 0.15, 8, 6000, 16000); // 6 0.15 8//10000
    PID_Init(&shooter.triggerMotor.anglePID.outer, 0.8, 0, 12, 0.12, 16000); // 1.5 0 2.3   3.27 maxoutput 9000->7000

    PID_Init(&shooter.fricMotor[0].speedPID, 25, 0, 5, 0, 16000); // 摩擦轮
    PID_Init(&shooter.fricMotor[1].speedPID, 25, 0, 5, 0, 16000);
}

bool Heat_Limit()
{
    float d = 0.6; // 阈值
	(void)d;	   // 阈值
    float k = 1;   // 防止浪费未使用热量,增加冷却缩减系数
    if (JUDGE_GetRemainHeat() < 20)
	{
	    k = 5;
	    return true;
	}
    else if (JUDGE_GetRemainHeat() <= 35)
	{
	    k = 1;
	    t = k * 1000 * 10 / JUDGE_GetCoolingValue() * 1.01f;
	    return true;
	}
    else if (JUDGE_GetRemainHeat() <= 100)
	{
	    k = 0.5;
	    t = k * 1000 * 10 * (-JUDGE_GetRemainHeat() + 100) / (100 - 30) / JUDGE_GetCoolingValue();

	    return true;
	}
    else
	{
	    t = 0;
	    k = 0.5;
	    return true;
	}
}

// 摩擦轮控制
// 摇杆控制
void Task_Shooter_Callback()
{
    static uint8_t over_speed, down_speed=0;
//    heat=JUDGE_GetRemainHeat();
//	heat=JUDGE_GetRemainHeat();
    if (USER_JudgeData.initial_speed > 10)
	{
	shooter.bullet_speed = USER_JudgeData.initial_speed;
	}

    if (shooter.bullet_speed != shooter.last_bullet_speed) // 收到弹速消息
	{
	    //22
	    if (shooter.bullet_speed > 22.5f)//23
		{
		    over_speed++;

		    if (over_speed >= 2)
			{
			    over_speed = 0;
			    shooter.fricSpd -= 60;
			    down_speed = 0;
			}
		}
	    else if (shooter.bullet_speed < 21.5f)//21
		{
		    down_speed++;

		    if (down_speed >= 4)
			{
			    down_speed = 0;
			    shooter.fricSpd += 50;
			}
		}
	}

    shooter.last_bullet_speed = shooter.bullet_speed;

    if (shooter.fricOpenFlag)
	{
	    shooter.fricMotor[0].targetSpeed = -shooter.fricSpd; // 摩擦轮速度
	    shooter.fricMotor[1].targetSpeed = shooter.fricSpd;
	}
    else
	{
	    shooter.fricMotor[0].targetSpeed = 0; // 摩擦轮速度
	    shooter.fricMotor[1].targetSpeed = 0;
	}


//堵转处理
//电机角度与目标角度相差超过10度则进行堵转判定
    if (abs(shooter.triggerMotor.totalAngle - shooter.triggerMotor.targetAngle) > MOTOR_M3508_DGR2CODE(10) && shooter.workState != TRIGGER_REVERSE)
	//	Shooter_state(shooter.fricOpenFlag);
	{
	    shooter.block.judgeCnt++; //堵转判定计数器++

	    if (shooter.block.judgeCnt > 100) //计数器达到一定值，则判定为堵转，触发反转
		{
		    shooter.block.judgeCnt = 0;
		    shooter.workState = TRIGGER_REVERSE;
		}
	}
    else //与目标值相差小于10度，拨弹状态正常，将堵转判定计数器归零
	{
	    shooter.block.judgeCnt = 0;
	}
	Heat_Limit();
    //拨弹
	switch(shooter.workState)
	{
	case TRIGGER:
		if (shooter.fricOpenFlag && (!JUDGE_IsValid() || Heat_Limit()))   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if(shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M3508_DGR2CODE(9))
			{
				shooter.triggerMotor.targetAngle+=MOTOR_M3508_DGR2CODE(360*1/9.0*1);  //每次转动1/9圈
				shooter.workState=IDLE;
				shooter.number +=1;
				osDelay(t);
			}
		}
		else
		{
			shooter.workState=IDLE;
		}
	break;

	case TRIGGER_CONTINUE:
		if (shooter.fricOpenFlag && (!JUDGE_IsValid() || Heat_Limit()))   //未安装裁判系统 或 裁判系统剩余热量大于100 允许发射
		{
			if (shooter.triggerMotor.targetAngle-shooter.triggerMotor.totalAngle<MOTOR_M3508_DGR2CODE(9))
			{
				shooter.triggerMotor.targetAngle+=MOTOR_M3508_DGR2CODE(360*1/9.0*2);  //每次转动1/9圈
				shooter.number +=1;
				osDelay(t);
			}
		}
		else
		{
			shooter.workState=IDLE;
		}
	break;

	case TRIGGER_REVERSE:
					//嘀嘀嘀
		Beep_PlayNotes((Note[]){{T_M1,D_Sixteenth},{T_M1,D_Sixteenth},{T_M1,D_Sixteenth}},3);
		shooter.triggerMotor.targetAngle -= MOTOR_M3508_DGR2CODE(360*1.5/9.0*2); //电机反向拨动1.5/9圈
		osDelay(500);
		shooter.triggerMotor.targetAngle += MOTOR_M3508_DGR2CODE(360*0.5/9.0*2); //正转0.5/9圈
		shooter.workState=IDLE;
	break;

	default:
	break;
	}
}

void OS_ShooterCallback(void const * argument)
{
    osDelay(100);
    Shooter_Init();
    PID_Clear(&shooter.triggerMotor.anglePID.inner);
    PID_Clear(&shooter.triggerMotor.anglePID.outer);
    osDelay(100);

    for(;;)
	{
	    if(true /* JUDGE_GetShooterOutputState()||1 */)
		{
		    Task_Shooter_Callback();
		}
	    osDelay(10);
	}
}


