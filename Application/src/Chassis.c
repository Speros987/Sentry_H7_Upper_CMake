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
#include "arm_math.h"
#include "Gimbal.h"
#include "Vision.h"
#include "degree.h"

#include <math.h>

float vx, vy, vw;  //AI传来的旋转速度

/*======================= 全局变量定义 =======================*/
Chassis_t chassis = {0}; // 底盘全局状态结构体：尺寸信息/四个舵轮电机/移动速度/旋转信息/模式等（定义详见Chassis.h）

/********************初始化************************/
/**
 * @brief  底盘初始化
 * @note   由FreeRTOS底盘任务 OS_ChassisCallback 在上电延时500ms后调用一次，
 *         完成底盘尺寸参数、云台初始角度、四个舵轮转向零偏、斜坡发生器与旋转PID的初始化。
 */
void Chassis_Init()
{
    // 底盘尺寸信息（用于解算轮速）
    chassis.info.wheelbase = 320; // 轴距：前后轮中心距(mm)
    chassis.info.wheeltrack = 320; // 轮距：左右轮中心距(mm)
    chassis.info.wheelRadius = 115; // 轮半径(mm)，用于线速度与轮速之间的换算
    chassis.info.offsetX = 0; // 15   // 重心相对几何中心的x方向偏移(mm)，用于旋转时各轮分量的差异补偿（当前为0，即不补偿） // 15
    chassis.info.offsetY = 0; //-10   // 重心相对几何中心的y方向偏移(mm)，作用同上 //-10
    // 移动参数初始化

    // 旋转参数初始化
    chassis.rotate.InitAngle = INIT_YAW_ANGLE; // 云台与底盘机械对齐时yaw电机的编码器角度(°)，作为相对角解算基准
    chassis.rotate.InitpitchAngle = 1290; // 云台水平时pitch电机的编码器值

	chassis.motors[0].TurnOffset= -304.57  +0 + 180; //
	chassis.motors[1].TurnOffset= -44.78 + 30 + 45 + 180; //       // 右前舵轮转向电机零偏(°) //
	chassis.motors[2].TurnOffset= -119.49 + 30 + 90 + 15 + 180; //  //此处校准舵电机 正常加减30°的倍数   // 左后舵轮转向电机零偏(°) //  //此处校准舵电机 正常加减30°的倍数
	chassis.motors[3].TurnOffset= -6.5 + 90 - 15 + 180; //         // 右后舵轮转向电机零偏(°) //

    // 斜坡函数初始化
    // 斜坡函数初始化：参数依次为（斜坡对象，每周期最大变化量step，死区deadzone）
    Slope_Init(&chassis.move.xSlope, 40, 0); // x方向平移速度斜坡：限制加速度，使速度变化平滑
    Slope_Init(&chassis.move.ySlope, 40, 0); // y方向平移速度斜坡
    Slope_Init(&chassis.move.spinSlope, 0.1, 0); // 旋转速度斜坡：限制角加速度
    Slope_Init(&chassis.move.outputSlope, 0.1, 0); // 输出斜坡（预留）
    Slope_Init(&chassis.move.chargeSlope, 0.15, 0); // 充能/功率相关斜坡（预留）

    Chassis_InitPID(); // 初始化旋转PID
}

/**
 * @brief  底盘旋转PID初始化
 * @note   输入为云台-底盘相对角偏差(°)，输出为底盘旋转速度；
 *         参数依次为：Kp=0.25、Ki=0、Kd=6、积分限幅2、输出限幅15。
 */
void Chassis_InitPID()
{
    PID_Init(&chassis.rotate.pid, 0.25, 0, 6, 2, 15); // 15	PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15   // （原调试参数保留）：Kp=0.4、Ki=0.001、Kd=0.15、无限幅、输出限幅15 // 15    PID_Init(&chassis.rotate.pid, 0.4, 0.001, 0.15, 0, 15); // 15
    PID_SetDeadzone(&chassis.rotate.pid, 0.1); // 设置PID死区0.1°：偏差小于死区时不调节，避免静止时抖动
}

/**
 * @brief  按当前模式预置底盘最大速度限制（小陀螺/跟随）
 * @note   小陀螺模式下限制最大平移速度，跟随模式下放开到较大值；
 *         后续 Chassis_UpdateSlope 中还会根据电机最大轮速重新计算覆盖。
 */
void Spin_SpeedUpdate() //
{
    chassis.move.maxVw = 3.25f; // chassis.move.maxPower * 0.225f + 3.25f   // 最大旋转速度(rad/s)，原设计随功率动态变化，现固定为3.25 // chassis.move.maxPower * 0.225f + 3.25f
    if (chassis.move.maxVw <= 0.5f)
    {
        chassis.move.maxVw = 0.5f; // 下限保护：保证最大旋转速度不低于0.5rad/s
    }

    if (chassis.rotate.mode == ChassisMode_Spin)
    {
        chassis.move.maxVx = 2159.6f; //0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f   // 小陀螺模式下最大平移速度(mm/s)，原为随功率变化的二次曲线拟合 //0.4985f * chassis.move.maxPower * chassis.move.maxPower - 40.994f * chassis.move.maxPower + 2159.6f
        if (chassis.move.maxVx <= 0)
            chassis.move.maxVx = 0;
        chassis.move.maxVy = chassis.move.maxVx; // y方向与小陀螺最大平移速度相同
    }
    else
    {
        chassis.move.maxVx = 5500; // 跟随模式：x方向最大平移速度(mm/s)
        chassis.move.maxVy = 5500; // 跟随模式：y方向最大平移速度(mm/s)
    }
}

/**
 * @brief  推进各斜坡发生器，并重新计算底盘三轴最大速度限制
 * @note   maxVx/maxVy 由驱动轮最大转速 WHEELSPEED_MAX 反推的底盘最大平移速度；
 *         maxVw 由最大轮速和等效旋转半径反推的最大旋转角速度。
 */
void Chassis_UpdateSlope()
{
    Slope_NextVal(&chassis.move.xSlope); // 推进一步x方向平移速度斜坡（输出更平滑的当前值）
    Slope_NextVal(&chassis.move.ySlope); // 推进一步y方向平移速度斜坡
    Slope_NextVal(&chassis.move.spinSlope); // 推进一步旋转速度斜坡
    Spin_SpeedUpdate(); // 按模式刷新最大速度限制（随后会被下方公式重新覆盖）
    float rotateRatio = (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f; // 旋转速度->轮速的等效换算系数（平均旋转力臂）
    chassis.move.maxVx = WHEELSPEED_MAX / 60.0f / (268.f / 17.f) * 2 * PI * chassis.info.wheelRadius; // 由最大电机转速(RPM)换算最大轮线速度(mm/s)：RPM/60/减速比*2πR
    chassis.move.maxVy = chassis.move.maxVx; // y方向最大平移速度与x方向相同
    chassis.move.maxVw = WHEELSPEED_MAX / rotateRatio / 60.0f / (268.f / 17.f) * 2.0f * PI * chassis.info.wheelRadius * 1.0f / 1.414f; // 由最大轮速反推最大旋转角速度(rad/s)，1/1.414为对角轮几何修正
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
        chassis.rotate.mode = ChassisMode_Spin; // 左拨杆=2（下方）：进入小陀螺模式
    else if (chassis.rotate.mode == ChassisMode_Spin && rcInfo.left == 3)
        chassis.rotate.mode = ChassisMode_Follow; // 左拨杆=3（上方）：回到底盘跟随云台模式

    switch (rcInfo.right)
    {
    case 3:
        chassis.pattern = Chassis_AI; // 右拨杆中：AI自动模式（视觉接管）
    break;
    case 1:
        chassis.pattern = Chassis_control; // 右拨杆上：手动遥控模式（此处无break，会落入default，无额外操作）
    default:
        break;
    }

    if (chassis.pattern == Chassis_AI)
    {
        if(!vision_receive.spin_mode) //AI确定模式   // 视觉下发的旋转模式标志：0=小陀螺 //AI确定模式
            chassis.rotate.mode = ChassisMode_Spin;
        else
            chassis.rotate.mode = ChassisMode_Follow; // 非0=跟随
    }
}

// 底盘任务回调函数
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
    chassis.rotate.InitAngle = ModularDegreeTowards(INIT_YAW_ANGLE, 0.0f);

    //uint16_t a = YawLost;  //判断云台电机是否离线
    chassis.rotate.nowAngle = gimbal.base_yawMotor.nowAngle; // 当前云台yaw角度(°)
    chassis.rotate.relativeAngle = chassis.rotate.nowAngle - chassis.rotate.InitAngle; //解算云台与底盘的夹角   // 相对角：云台相对底盘机械零位的偏转角度(°) //解算云台与底盘的夹角

    Chassis_ModeCtrl(); // 更新底盘模式（跟随/小陀螺、手动/AI）

    /*---------------- 平移速度输入处理 ----------------*/
    if (chassis.pattern == Chassis_AI )
    {
        Slope_SetTarget(&chassis.move.xSlope,1.5f * vx); //x为前后   // AI模式：视觉速度指令vx乘1.5倍增益后作为x方向平移速度斜坡目标 //x为前后
        Slope_SetTarget(&chassis.move.ySlope,1.5f * vy); // AI模式：视觉速度指令vy作为y方向平移速度斜坡目标
        float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180); // 云台相对角的正弦值（取负号与坐标系方向约定相关）
        float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180); // 云台相对角的余弦值
        chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin); // 旋转矩阵变换：把云台系平移速度合成为底盘系x方向速度(mm/s)
        chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos); // 旋转矩阵变换：把云台系平移速度合成为底盘系y方向速度(mm/s)
        Chassis_UpdateSlope(); // 推进一步斜坡并刷新最大速度限制

        if (vision_receive.spin_mode == 0)
            chassis.rotate.mode = ChassisMode_Follow; // 视觉标志spin_mode=0：切换为跟随模式（注意此处判断方向与Chassis_ModeCtrl相反，本处结果生效）
        else
            chassis.rotate.mode = ChassisMode_Spin; // 视觉标志spin_mode≠0：切换为小陀螺模式
    }
    else
    {
        Slope_SetTarget(&chassis.move.xSlope,-(float)rcInfo.ch3*chassis.move.maxVx/660); // 手动模式：遥控器ch3通道映射到x方向速度目标（取反，660为通道半量程，速度按maxVx限幅）
        Slope_SetTarget(&chassis.move.ySlope,(float)rcInfo.ch4*chassis.move.maxVy/660); // 手动模式：遥控器ch4通道映射到y方向速度目标
        // 将云台坐标系下平移速度解算到底盘平移速度(根据云台偏离角)
        float gimbalAngleSin=sin(-chassis.rotate.relativeAngle*PI/180); // 云台相对角的正弦值
        float gimbalAngleCos=cos(-chassis.rotate.relativeAngle*PI/180);
        chassis.move.vx=-(Slope_GetVal(&chassis.move.xSlope) * gimbalAngleCos + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleSin); // 旋转矩阵变换：合成为底盘系x方向速度(mm/s)
        chassis.move.vy=(-Slope_GetVal(&chassis.move.xSlope) * gimbalAngleSin + Slope_GetVal(&chassis.move.ySlope) * gimbalAngleCos); // 旋转矩阵变换：合成为底盘系y方向速度(mm/s)
        Chassis_UpdateSlope();
    }

        // 旋转相关内容
    /*---------------- 旋转速度解算 ----------------*/
    // 跟随模式：底盘朝向跟随云台，让相对角回到0°
    if (chassis.rotate.mode == ChassisMode_Follow)
    {
        Slope_SetTarget(&chassis.move.spinSlope, 0);
        chassis.rotate.relativeAngle = ModularDegreeTowards(chassis.rotate.relativeAngle, 0.0f);

        if (chassis.pattern==Chassis_control)
        {
            PID_SingleCalc(&chassis.rotate.pid, 0, -chassis.rotate.relativeAngle); // 目标0°、反馈取-相对角，PID输出驱动底盘向相对角减小的方向旋转

            chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value; // 旋转速度=跟随PID输出+自转斜坡值（跟随模式斜坡为0）
            LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw); // 按最大旋转速度限幅

            if (chassis.pattern == Chassis_AI || Gimbal_VisionForced())
            {
                chassis.move.vw = 0; // 视觉自瞄接管时禁止底盘自转，避免干扰自瞄
            }
        }
        else
        {
            // AI模式下的跟随分支
            if (vision_receive.align_mode == 1)
            {
                float target = -chassis.rotate.align_yaw;
                float feedback = ModularDegreeTowards(-chassis.rotate.relativeAngle, target);

                // 对齐模式：底盘与视觉下发的方向(align_yaw，如起伏路段方向)对齐
                PID_SingleCalc(&chassis.rotate.pid, target, feedback); // PID计算旋转速度
                chassis.move.vw = chassis.rotate.pid.output + chassis.move.spinSlope.value;
                LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
            else
            {
                chassis.move.vw = vw; // 非对齐模式：直接使用视觉下发的旋转速度指令(rad/s)
                LIMIT(chassis.move.vw, -chassis.move.maxVw, chassis.move.maxVw);
            }
        }
    }
    else if (chassis.rotate.mode == ChassisMode_Spin) //小陀螺模式
    {
        chassis.move.vw = chassis.move.spinSlope.value; // 当前旋转速度取自转斜坡输出值
        float ratio;

        if (chassis.pattern == Chassis_control)
        {
            if (fabsf(Slope_GetVal(&chassis.move.xSlope)) / chassis.move.maxVx + fabsf(Slope_GetVal(&chassis.move.ySlope)) / chassis.move.maxVy > 0.05f)
                // 手动模式：底盘有平移动作时降低自转比例，保证平移机动性能
                ratio = 0.6f; // 有平移指令：自转比例降为0.6
            else
                ratio = 1.0f; // 纯自转：全速小陀螺

            Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio); // 自转目标=maxVw×ratio，经斜坡平滑加速
        }
        else
        {
            ratio = 0.5; // AI模式：自转比例固定0.5，兼顾自瞄稳定性
        }

        Slope_SetTarget(&chassis.move.spinSlope, chassis.move.maxVw * ratio);
    }

//检测当前角度
    /*---------------- 四舵轮解算：轮速与舵角 ----------------*/
    //检测当前角度：转向电机角度减零偏，并归一化到0~360°，得到舵轮实际朝向
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis.motors[i].now_angle = ModularDegreeTowards(chassis.motors[i].TurnAngle - chassis.motors[i].TurnOffset, 0.0f);
    }

    /***解算各轮子转速****/
    float rotateRatio[4]; // 各轮自转分量系数：基准旋转力臂+重心偏移补偿（offsetX/offsetY随|自转速度|权重线性补偿）
    rotateRatio[0] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw; // 左前轮
    rotateRatio[1] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f - chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw; // 右前轮
    rotateRatio[2] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw + chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw; // 左后轮
    rotateRatio[3] = 1.414f * (chassis.info.wheelbase + chassis.info.wheeltrack) / 4.0f + chassis.info.offsetY * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw - chassis.info.offsetX * fabs(chassis.move.spinSlope.value) / chassis.move.maxVw; // 右后轮

    float wheelvx[4], wheelvy[4];

    // 左前
    wheelvx[0] = chassis.move.vx + chassis.move.vw * rotateRatio[1] / 1.414f; // 左前轮速度x分量=底盘平移速度+自转线速度分量
    wheelvy[0] = chassis.move.vy + chassis.move.vw * rotateRatio[1] / 1.414f; // 左前轮速度y分量
    // 右前
    wheelvx[1] = chassis.move.vx + chassis.move.vw * rotateRatio[2] / 1.414f; // 右前轮速度x分量
    wheelvy[1] = chassis.move.vy - chassis.move.vw * rotateRatio[2] / 1.414f; // 右前轮速度y分量（自转分量取反）
    // 左后
    wheelvx[2] = chassis.move.vx - chassis.move.vw * rotateRatio[0] / 1.414f; // 左后轮速度x分量（自转分量取反）
    wheelvy[2] = chassis.move.vy + chassis.move.vw * rotateRatio[0] / 1.414f; // 左后轮速度y分量
    // 右后
    wheelvx[3] = chassis.move.vx - chassis.move.vw * rotateRatio[3] / 1.414f; // 右后轮速度x分量（自转分量取反）
    wheelvy[3] = chassis.move.vy - chassis.move.vw * rotateRatio[3] / 1.414f; // 右后轮速度y分量（自转分量取反）

    //    舵轮解算
    //	舵轮解算
    for (uint8_t i = 0; i < 4; i++)
    {
        float target_angle;
        int32_t wheelRPM = -hypotf(wheelvx[i], wheelvy[i]) * 60 / (2 * PI * chassis.info.wheelRadius) * (268.f / 17.f);

        if (wheelRPM != 0)
        {
            target_angle = atan2f(-wheelvy[i], wheelvx[i]) / PI * 180;
        // 有目标轮速：根据合速度矢量方向解算舵轮目标角度
        }
        else // 一般情况：先求合速度与x轴夹角|arctan(vy/vx)|，再按象限修正 //当目标速度为0 且电机速度已经减下来时  舵回到正常角度
        { //																																					|y/|
            static const float default_angle[4] = {135, 45, 45, 135};
            target_angle = default_angle[i];
        } //																																					|

        target_angle = ModularDegreeTowards(target_angle, chassis.motors[i].now_angle);

        if (target_angle - chassis.motors[i].now_angle >= 90)
        {
            target_angle -= 180.0f;
            wheelRPM *= -1.0f;
        // 按合速度所在象限把夹角修正为0~360°的舵角（θ为arctan求得的锐角）
        }
        else if (target_angle - chassis.motors[i].now_angle < -90)
        {
            target_angle += 180.0f;
            wheelRPM *= -1.0f;
        // 目标轮速为0：无速度分量时让舵轮摆回默认停放角度（对角对称姿态，方便随时起步）
        }

        chassis.motors[i].targetTurnAngle = target_angle;
        chassis.motors[i].targetDriveSpeed = wheelRPM;
    // 最短路径处理：把舵角差归一化到[-180°,180°]，让舵机朝最近方向转向
    // 若舵角差超过90°：舵角反转180°并同步反转轮速（等效方向不变），减少舵机转向时间
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
    osDelay(500); // 上电延时500ms：等待其他任务与外设初始化完成
    Chassis_Init(); // 底盘初始化（仅执行一次）

    for(;;)
    {
        Task_Chassis_Callback(); // 周期执行底盘控制解算（2ms周期）
        osDelay(2);
    }
}
