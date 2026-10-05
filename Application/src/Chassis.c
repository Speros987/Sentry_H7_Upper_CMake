/**
 * @file    Chassis.c
 * @brief   哨兵四舵轮底盘控制模块
 * @details 主要功能与执行流程：
 *          1. Chassis_Init          —— 初始化底盘尺寸、舵轮零偏、斜坡发生器和旋转PID；
 *          2. Chassis_ModeCtrl      —— 根据遥控拨杆/视觉信息决定底盘模式（跟随/小陀螺、手动/AI）；
 *          3. Task_Chassis_Callback —— 2ms周期任务：解算云台-底盘相对角 -> 处理速度输入（AI/遥控，含云台系到底盘系的坐标变换）
 *             -> 解算旋转速度（跟随PID回正/小陀螺定速）-> 解算四个舵轮的轮速与舵角；
 *          4. OS_ChassisCallback    —— 底盘FreeRTOS任务入口。
 */

#include "Chassis.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "USER_RC.h"
#include "user_lib.h"
#include "arm_math.h"
#include "Gimbal.h"
#include "vision.h"


/*======================= 全局变量定义 =======================*/

int8_t diagonal_enable = 0;   // 斜行模式使能标志：0=关闭，1=开启（当前工程未使用，预留）
float vx,vy,vw = 0;           // AI视觉速度指令：vx=左右方向平移速度(mm/s)、vy=前后方向平移速度(mm/s)、vw=旋转角速度(rad/s)，在Vision.c中解包并完成坐标系转换
float wheelvx[4];             // 各舵轮速度在底盘x方向的分量(mm/s)，下标0~3依次对应：左前、右前、左后、右后
float wheelvy[4];             // 各舵轮速度在底盘y方向的分量(mm/s)，下标顺序同上
int32_t wheelRPM[4];          // 各舵轮驱动轮的目标转速(RPM，带方向符号)，下标顺序同上
float targetangle[4];         // 各舵轮转向目标角度(°，0~360)，下标顺序同上

Chassis_t chassis = {0};      // 底盘全局状态结构体：尺寸信息/四个舵轮电机/移动速度/旋转信息/模式等（定义详见Chassis.h）

float vx_test;                // 调试预留变量：AI横移速度观测（当前未使用）
float vy_test;                // 调试预留变量：AI前后速度观测（当前未使用）
float mode_test;              // 调试预留变量：模式观测（当前未使用）

/********************初始化************************/
/**
 * @brief  底盘初始化
 * @note   由FreeRTOS底盘任务 OS_ChassisCallback 在上电延时500ms后调用一次，
 *         完成底盘尺寸参数、云台初始角度、四个舵轮转向零偏、斜坡发生器与旋转PID的初始化。
 */
void Chassis_Init()
{
	// 底盘尺寸信息（用于解算轮速）
	chassis.info.wheelbase = 320;      // 轴距：前后轮中心距(mm)
	chassis.info.wheeltrack = 320;     // 轮距：左右轮中心距(mm)
	chassis.info.wheelRadius = 115;    // 轮半径(mm)，用于线速度与轮速之间的换算
	chassis.info.offsetX = 0; // 15   // 重心相对几何中心的x方向偏移(mm)，用于旋转时各轮分量的差异补偿（当前为0，即不补偿）
	chassis.info.offsetY = 0; //-10   // 重心相对几何中心的y方向偏移(mm)，作用同上
	// 移动参数初始化

	// 旋转参数初始化
	chassis.rotate.InitAngle = INIT_YAW_ANGLE;   // 云台与底盘机械对齐时yaw电机的编码器角度(°)，作为相对角解算基准
	if (chassis.rotate.InitAngle > 360)
		chassis.rotate.InitAngle -= 360;         // 归一化到0~360°
	chassis.rotate.InitpitchAngle = 1290;        // 云台水平时pitch电机的编码器值

	chassis.motors[0].TurnOffset= -304.57 - 30 + 180; //        // 左前舵轮转向电机零偏(°)：校零后使now_angle=0对应该轮解算正方向
	chassis.motors[1].TurnOffset= -44.78 + 30 + 45 + 180;//       // 右前舵轮转向电机零偏(°)
	chassis.motors[2].TurnOffset= -119.49 + 30 + 90 + 15 + 180;//  //此处校准舵电机 正常加减30°的倍数   // 左后舵轮转向电机零偏(°)
	chassis.motors[3].TurnOffset= -6.5 + 90 - 15 + 180;//         // 右后舵轮转向电机零偏(°)

	// 斜坡函数初始化：参数依次为（斜坡对象，每周期最大变化量step，死区deadzone）
	Slope_Init(&chassis.move.xSlope, 40, 0);         // x方向平移速度斜坡：限制加速度，使速度变化平滑
	Slope_Init(&chassis.move.ySlope, 40, 0);         // y方向平移速度斜坡
	Slope_Init(&chassis.move.spinSlope, 0.1, 0);     // 旋转速度斜坡：限制角加速度
	Slope_Init(&chassis.move.outputSlope, 0.1, 0);   // 输出斜坡（预留）
	Slope_Init(&chassis.move.chargeSlope, 0.15, 0);  // 充能/功率相关斜坡（预留）

	Chassis_InitPID();   // 初始化旋转PID
}

/**
 * @brief  底盘旋转PID初始化
 * @note   输入为云台-底盘相对角偏差(°)，输出为底盘旋转速度；
 *         参数依次为：Kp=0.25、Ki=0、Kd=6、积分限幅2、输出限幅15。
 */
void Chassis_InitPID()
{
	PID_Init(&chassis.rotate.pid, 0.25, 0, 6, 2, 15); // 15	PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15   // （原调试参数保留）：Kp=0.4、Ki=0.001、Kd=0.15、无限幅、输出限幅15
	PID_SetDeadzone(&chassis.rotate.pid, 0.1);  // 设置PID死区0.1°：偏差小于死区时不调节，避免静止时抖动
}

/**
 * @brief  按当前模式预置底盘最大速度限制（小陀螺/跟随）
 * @note   小陀螺模式下限制最大平移速度，跟随模式下放开到较大值；
 *         后续 Chassis_UpdateSlope 中还会根据电机最大轮速重新计算覆盖。
 */
void Spin_SpeedUpdate() //
{
	chassis.move.maxVw = 3.25f;// chassis.move.maxPower * 0.225f + 3.25f   // 最大旋转速度(rad/s)，原设计随功率动态变化，现固定为3.25
	if (chassis.move.maxVw <= 0.5f)
	{
		chassis.move.maxVw = 0.5f;// 下限保护：保证最大旋转速度不低于0.5rad/s
	}

	if (chassis.rotate.mode == ChassisMode_Spin)
	{
		chassis.move.maxVx = 2159.6f;//0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f   // 小陀螺模式下最大平移速度(mm/s)，原为随功率变化的二次曲线拟合
		if (chassis.move.maxVx <= 0)
			chassis.move.maxVx = 0;
		chassis.move.maxVy = chassis.move.maxVx;// y方向与小陀螺最大平移速度相同
	}
	else
	{
		chassis.move.maxVx = 5500;// 跟随模式：x方向最大平移速度(mm/s)
		chassis.move.maxVy = 5500;// 跟随模式：y方向最大平移速度(mm/s)
	}
}

/**
 * @brief  推进各斜坡发生器，并重新计算底盘三轴最大速度限制
 * @note   maxVx/maxVy 由驱动轮最大转速 WHEELSPEED_MAX 反推的底盘最大平移速度；
 *         maxVw 由最大轮速和等效旋转半径反推的最大旋转角速度。
 */
void Chassis_UpdateSlope()
{
	Slope_NextVal(&chassis.move.xSlope);// 推进一步x方向平移速度斜坡（输出更平滑的当前值）
	Slope_NextVal(&chassis.move.ySlope);// 推进一步y方向平移速度斜坡
	Slope_NextVal(&chassis.move.spinSlope);// 推进一步旋转速度斜坡
	Spin_SpeedUpdate();// 按模式刷新最大速度限制（随后会被下方公式重新覆盖）
	float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f;// 旋转速度->轮速的等效换算系数（平均旋转力臂）
	chassis.move.maxVx = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2 * PI * chassis.info.wheelRadius;// 由最大电机转速(RPM)换算最大轮线速度(mm/s)：RPM/60/减速比*2πR
	chassis.move.maxVy = chassis.move.maxVx;// y方向最大平移速度与x方向相同
	chassis.move.maxVw = WHEELSPEED_MAX / rotateRatio / 60.0f / (268.f / 17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f;// 由最大轮速反推最大旋转角速度(rad/s)，1/1.414为对角轮几何修正
}


/**
 * @brief  底盘模式控制
 * @note   左拨杆控制底盘旋转模式：拨到下方进入小陀螺(Spin)，拨回上方回到跟随(Follow)；
 *         右拨杆控制控制来源：上=手动遥控(Chassis_control)，下=AI自动(Chassis_AI)；
 *         若为AI模式，则最终旋转模式由视觉下发的 spin_mode 决定（0=小陀螺，非0=跟随）。
 */
void Chassis_ModeCtrl()
{
	// 左拨杆拨到下方进入小陀螺，拨回来进入跟随模式
	if (chassis.rotate.mode != ChassisMode_Spin && rcInfo.left == 2 )
			chassis.rotate.mode = ChassisMode_Spin;// 左拨杆=2（下方）：进入小陀螺模式
	else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
			chassis.rotate.mode = ChassisMode_Follow;// 左拨杆=3（上方）：回到底盘跟随云台模式
	
	switch(rcInfo.right)
		{
			case 3:
				 chassis.pattern = Chassis_AI;// 右拨杆中：AI自动模式（视觉接管）
				break;
			case 1:
				 chassis.pattern =Chassis_control;// 右拨杆上：手动遥控模式（此处无break，会落入default，无额外操作）
			default:
				break;
		}
	if(chassis.pattern == Chassis_AI)
	{
		if(!vision_receive.spin_mode)//AI确定模式   // 视觉下发的旋转模式标志：0=小陀螺
			chassis.rotate.mode = ChassisMode_Spin;
		else
			chassis.rotate.mode = ChassisMode_Follow;  // 非0=跟随
	}
}




/************************freertos任务**********************
以下任务受freertos操作系统调度
**********************************************************/

/**
 * @brief  底盘任务回调函数（由底盘任务以2ms周期调用）
 * @note   执行流程：
 *         1. 解算云台与底盘的相对夹角 relativeAngle（用于坐标变换与跟随控制）；
 *         2. 调用 Chassis_ModeCtrl 更新底盘模式；
 *         3. 根据 AI/手动 来源设置平移速度斜坡目标，并将云台坐标系速度旋转到底盘坐标系；
 *         4. 解算旋转速度目标 vw（跟随：相对角PID回正 / 小陀螺：斜坡给定自转速度）；
 *         5. 解算四个舵轮的驱动轮目标转速 wheelRPM 与转向目标角度 targetangle，并写入电机目标。
 */
void Task_Chassis_Callback()
{
	chassis.rotate.InitAngle = INIT_YAW_ANGLE;// 重新装载云台-底盘对齐基准角
	if (chassis.rotate.InitAngle >= 360)
		chassis.rotate.InitAngle -= 360;// 归一化到0~360°
	if (chassis.rotate.InitAngle < 0)
		chassis.rotate.InitAngle += 360;
	//uint16_t a = YawLost;  //判断云台电机是否离线
	uint16_t a = 0;// 云台yaw电机在线标志：0=在线（当前强制为0，即暂不处理云台掉线的情况）
	if (a == 0)
	{
		chassis.rotate.nowAngle = gimbal.base_yawMotor.nowAngle;// 当前云台yaw角度(°)
		chassis.rotate.relativeAngle = chassis.rotate.nowAngle - chassis.rotate.InitAngle;//解算云台与底盘的夹角   // 相对角：云台相对底盘机械零位的偏转角度(°)
	}
	else
	{
		chassis.rotate.relativeAngle = 0;// 云台掉线时相对角按0处理，避免底盘跟随异常
	}

	Chassis_ModeCtrl();// 更新底盘模式（跟随/小陀螺、手动/AI）

	/*---------------- 平移速度输入处理 ----------------*/
	if (chassis.pattern == Chassis_AI )
	{
			Slope_SetTarget(&chassis.move.xSlope,1.5f * vx); //x为前后   // AI模式：视觉速度指令vx乘1.5倍增益后作为x方向平移速度斜坡目标
			Slope_SetTarget(&chassis.move.ySlope,1.5f * vy);             // AI模式：视觉速度指令vy作为y方向平移速度斜坡目标
    		float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180);   // 云台相对角的正弦值（取负号与坐标系方向约定相关）
			float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180);   // 云台相对角的余弦值
			chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);   // 旋转矩阵变换：把云台系平移速度合成为底盘系x方向速度(mm/s)
			chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);    // 旋转矩阵变换：把云台系平移速度合成为底盘系y方向速度(mm/s)
			Chassis_UpdateSlope();   // 推进一步斜坡并刷新最大速度限制
		if (vision_receive.spin_mode == 0)
				chassis.rotate.mode = ChassisMode_Follow;   // 视觉标志spin_mode=0：切换为跟随模式（注意此处判断方向与Chassis_ModeCtrl相反，本处结果生效）
		else
				chassis.rotate.mode = ChassisMode_Spin;	    // 视觉标志spin_mode≠0：切换为小陀螺模式
	}
	else
	{
			Slope_SetTarget(&chassis.move.xSlope,-(float)rcInfo.ch3*chassis.move.maxVx/660);   // 手动模式：遥控器ch3通道映射到x方向速度目标（取反，660为通道半量程，速度按maxVx限幅）
			Slope_SetTarget(&chassis.move.ySlope,(float)rcInfo.ch4*chassis.move.maxVy/660);    // 手动模式：遥控器ch4通道映射到y方向速度目标
			// 将云台坐标系下平移速度解算到底盘平移速度(根据云台偏离角)
			float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180);   // 云台相对角的正弦值
			float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180);   // 云台相对角的余弦值
			chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin);   // 旋转矩阵变换：合成为底盘系x方向速度(mm/s)
			chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos);    // 旋转矩阵变换：合成为底盘系y方向速度(mm/s)
			Chassis_UpdateSlope();   // 推进一步斜坡并刷新最大速度限制
	}

	/*---------------- 旋转速度解算 ----------------*/
	// 跟随模式：底盘朝向跟随云台，让相对角回到0°
	if (chassis.rotate.mode == ChassisMode_Follow) 
	{
		  if (chassis.pattern==Chassis_control)
				{
					Slope_SetTarget(&chassis.move.spinSlope, 0);   // 跟随模式下取消小陀螺自转目标（自转斜坡清零）
					if (chassis.rotate.relativeAngle >= 180)
						chassis.rotate.relativeAngle -= 360;       // 相对角归一化到[-180°,180°]，保证走最短回正路径
					if (chassis.rotate.relativeAngle < -180)
						chassis.rotate.relativeAngle += 360;
					if (a == 0)
					{
						PID_SingleCalc(&chassis.rotate.pid, 0, -chassis.rotate.relativeAngle);   // 目标0°、反馈取-相对角，PID输出驱动底盘向相对角减小的方向旋转
					}
					else
					{
						chassis.rotate.relativeAngle = 0;          // 云台掉线：清零相对角与PID输出，避免底盘误动作
						chassis.rotate.pid.output = 0;
					}
					chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;   // 旋转速度=跟随PID输出+自转斜坡值（跟随模式斜坡为0）
					LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);              // 按最大旋转速度限幅
					if(gimbal.visionEnable == true)
					{
							chassis.move.vw=0;   // 视觉自瞄接管时禁止底盘自转，避免干扰自瞄
					}
				}
			else
				{
					// AI模式下的跟随分支
					Slope_SetTarget(&chassis.move.spinSlope, 0);   // 取消自转
					if (chassis.rotate.relativeAngle >= 180)
						chassis.rotate.relativeAngle -= 360;       // 相对角归一化
					if (chassis.rotate.relativeAngle < -180)
						chassis.rotate.relativeAngle += 360;
					if (vision_receive.align_mode == 1)
					{
						// 对齐模式：底盘与视觉下发的方向(align_yaw，如起伏路段方向)对齐
						float target = -chassis.rotate.align_yaw;           // 目标角度：取反后的视觉对齐角
						float feedback = -chassis.rotate.relativeAngle;     // 反馈量：取反后的云台-底盘相对角
						if (target - feedback > 180.0f)
						{
							feedback += 360.0f;   // 角度差大于180°，修正反馈量避免绕圈
						}
						else if (target - feedback < -180.0f)
						{
							feedback -= 360.0f;   // 角度差小于-180°，反向修正
						}
						PID_SingleCalc(&chassis.rotate.pid, target, feedback);   // PID计算旋转速度
						chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
						LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
					}
					else 
					{
						chassis.move.vw = vw;   // 非对齐模式：直接使用视觉下发的旋转速度指令(rad/s)
						LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
					}
				}
	}
	else if (chassis.rotate.mode == ChassisMode_Spin) //小陀螺模式
	{
		float ratio = 0.8;                                // 自转速度比例系数（相对于最大旋转速度maxVw）
		chassis.move.vw = chassis.move.spinSlope.value;   // 当前旋转速度取自转斜坡输出值
		if(chassis.pattern==Chassis_control)
		{
			// 手动模式：底盘有平移动作时降低自转比例，保证平移机动性能
			if (ABS(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + ABS(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
				ratio = 0.6f;		                      // 有平移指令：自转比例降为0.6
			else
				ratio = 1.0f;                             // 纯自转：全速小陀螺
			Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);   // 自转目标=maxVw×ratio，经斜坡平滑加速
		}
		else
		{
      ratio = 0.5;   // AI模式：自转比例固定0.5，兼顾自瞄稳定性
			Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
		}
	}
 
/*---------------- 四舵轮解算：轮速与舵角 ----------------*/
//检测当前角度：转向电机角度减零偏，并归一化到0~360°，得到舵轮实际朝向
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis.motors[i].now_angle = chassis.motors[i].TurnAngle - chassis.motors[i].TurnOffset;   // 舵轮当前实际角度=转向电机角度-安装零偏
		if (chassis.motors[i].now_angle >= 360.0f)
			chassis.motors[i].now_angle -= 360;
		if (chassis.motors[i].now_angle < 0)
			chassis.motors[i].now_angle += 360;
	}
	/***解算各轮子转速****/
	float rotateRatio[4];   // 各轮自转分量系数：基准旋转力臂+重心偏移补偿（offsetX/offsetY随|自转速度|权重线性补偿）
	rotateRatio[0] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;   // 左前轮
	rotateRatio[1] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;   // 右前轮
	rotateRatio[2] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;   // 左后轮
	rotateRatio[3] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw;   // 右后轮
	// 左前
	wheelvx[0] = chassis.move.vx + chassis.move.vw * rotateRatio[1] / 1.414f;   // 左前轮速度x分量=底盘平移速度+自转线速度分量
	wheelvy[0] = chassis.move.vy + chassis.move.vw * rotateRatio[1] / 1.414f;   // 左前轮速度y分量
	wheelRPM[0] = -sqrt(powf(wheelvx[0], 2) + powf(wheelvy[0], 2)) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);   // 合成线速度->驱动轮转速(RPM)：负号为安装方向约定，268/17为减速比
	// 右前
	wheelvx[1] = chassis.move.vx + chassis.move.vw * rotateRatio[2] / 1.414f;   // 右前轮速度x分量
	wheelvy[1] = chassis.move.vy - chassis.move.vw * rotateRatio[2] / 1.414f;   // 右前轮速度y分量（自转分量取反）
	wheelRPM[1] = -sqrt(powf(wheelvx[1], 2) + powf(wheelvy[1], 2)) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);   // 右前驱动轮转速(RPM)
	// 左后
	wheelvx[2] = chassis.move.vx - chassis.move.vw * rotateRatio[0] / 1.414f;   // 左后轮速度x分量（自转分量取反）
	wheelvy[2] = chassis.move.vy + chassis.move.vw * rotateRatio[0] / 1.414f;   // 左后轮速度y分量
	wheelRPM[2] = -sqrt(powf(wheelvx[2], 2) + powf(wheelvy[2], 2)) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);   // 左后驱动轮转速(RPM)
	// 右后
	wheelvx[3] = chassis.move.vx - chassis.move.vw * rotateRatio[3] / 1.414f;   // 右后轮速度x分量（自转分量取反）
	wheelvy[3] = chassis.move.vy - chassis.move.vw * rotateRatio[3] / 1.414f;   // 右后轮速度y分量（自转分量取反）
	wheelRPM[3] = -sqrt(powf(wheelvx[3], 2) + powf(wheelvy[3], 2)) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);   // 右后驱动轮转速(RPM)
	//	舵轮解算
	for (uint8_t i = 0; i < 4; i++)
	{
		if (wheelRPM[i] != 0)
		{
			// 有目标轮速：根据合速度矢量方向解算舵轮目标角度
			if (wheelvx[i] == 0)   //轮速为0 90度  // x方向分量为0：舵角直接取90°（纯y方向运动）
			{
				targetangle[i] = 90.0f;
			}
			else   // 一般情况：先求合速度与x轴夹角|arctan(vy/vx)|，再按象限修正
			{													//																																					|y/|
				targetangle[i] = fabsf(atanf(1.0f * wheelvy[i] / wheelvx[i]) / 3.1415926535f * 180); //其他情况 ---------x  求出速度与x轴正方向夹角 之后根据轮速
			}													//																																					|
			// 按合速度所在象限把夹角修正为0~360°的舵角（θ为arctan求得的锐角）
			if (wheelvx[i] >= 0 && wheelvy[i] > 0)
				targetangle[i] = 360 - targetangle[i];   // 第一象限：360-θ
			else if (wheelvx[i] < 0 && wheelvy[i] >= 0)
				targetangle[i] = 180 + targetangle[i];   // 第二象限：180+θ
			else if (wheelvx[i] <= 0 && wheelvy[i] < 0)
				targetangle[i] = -targetangle[i] + 180;  // 第三象限：180-θ
			else if (wheelvx[i] > 0 && wheelvy[i] <= 0)
				targetangle[i] = targetangle[i];         // 第四象限：θ
			if (targetangle[i] >= 360)
				targetangle[i] -= 360;                   // 角度归一化到[0,360)
		}
		else
		{
			// 目标轮速为0：无速度分量时让舵轮摆回默认停放角度（对角对称姿态，方便随时起步）
			static const float default_angle[4] = {
				135, 45, 45, 135};   // 默认角度(°)：下标0~3依次为 左前135°、右前45°、左后45°、右后135°
			if(ABS(chassis.motors[i].now_Speed) < 100) /*|| detectList[DeviceID_ChassisMotor1 + i].isLost == 1*/
					targetangle[i] = default_angle[i];   // 仅当驱动轮已减速（转速<100）时才回舵，避免原地猛打舵
		}//当目标速度为0 且电机速度已经减下来时  舵回到正常角度

		// 最短路径处理：把舵角差归一化到[-180°,180°]，让舵机朝最近方向转向
		if (targetangle[i] - chassis.motors[i].now_angle >= 180)
			targetangle[i] = targetangle[i] - 360;
		else if (targetangle[i] - chassis.motors[i].now_angle < -180)
			targetangle[i] = targetangle[i] + 360;
		// 若舵角差超过90°：舵角反转180°并同步反转轮速（等效方向不变），减少舵机转向时间
		if (targetangle[i] - chassis.motors[i].now_angle >= 90)
		{
			targetangle[i] = targetangle[i] - 180;
			wheelRPM[i] = -wheelRPM[i];
		}
		else if (targetangle[i] - chassis.motors[i].now_angle < -90)
		{
			targetangle[i] = targetangle[i] + 180;
			wheelRPM[i] = -wheelRPM[i];
		}
		chassis.motors[i].targetTurnAngle = targetangle[i];    // 输出：转向电机目标角度(°)
		chassis.motors[i].targetDriveSpeed = wheelRPM[i];      // 输出：驱动电机目标转速(RPM)
	}
}


/**
 * @brief  底盘任务入口（FreeRTOS任务函数）
 * @param  argument 任务参数（未使用）
 * @note   上电后延时500ms等待电机/传感器初始化完成，再初始化底盘；
 *         之后以2ms为周期循环执行 Task_Chassis_Callback 完成底盘控制解算。
 */
void OS_ChassisCallback(void const * argument)
{
	osDelay(500);              // 上电延时500ms：等待其他任务与外设初始化完成
	Chassis_Init();            // 底盘初始化（仅执行一次）
    for(;;)
    {
		Task_Chassis_Callback();   // 周期执行底盘控制解算（2ms周期）
        osDelay(2);
    }
}
