#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "USER_RC.h"
#include "Vision.h"
#include "Chassis.h"
#include "Shooter.h"
#include "arm_math.h"
#include "degree.h"

#define DEG_TO_RAD 0.01745329f  // PI / 180
#define TOP_YAW_LIMIT 45.0f
#define BASE_YAW_J 0.04575f

Gimbal_t gimbal;

// float visionFindAver;
void Gimbal_InitPID(void);

/********************初始化************************/
// 初始化云台
void Gimbal_Init()
{
    gimbal.pitch.pitchMax = 34; // 设定pitch角度限幅
    gimbal.pitch.pitchMin = -20;

    gimbal.pitch.initAngle = 0; // 陀螺仪pitch开机角度 *0表示pitch水平
    gimbal.pitch.targetAngle = gimbal.pitch.initAngle;

    gimbal.base_yaw.initAngle = 0; // 陀螺仪yaw开机角度   *0表示yaw不动
    gimbal.base_yaw.targetAngle = gimbal.base_yaw.initAngle;

    gimbal.top_yaw.initAngle = 0;
    gimbal.top_yaw.targetAngle = gimbal.top_yaw.initAngle;

    Gimbal_InitPID();         // 初始化PID参数
}

void Gimbal_InitPID()
{
    /*pitch由陀螺仪控制*/
    PID_Init(&gimbal.pitch.imuPID.inner, 6, 0, 0.4, 500, 7000);
    DEPID_Init(&gimbal.pitch.imuPID.deOuter, 75, 0.7, 120, 200, 7000, 0.6);//45, 0.7, 50
    PID_Init(&gimbal.pitch.imuPID.outer,-0.8,-0.001,-8,1.3,10);
    PID_Init(&gimbal.pitch.MIT_PID,100,0,2,0,0);

    PID_Init(&gimbal.base_yaw.imuPID.inner,5.8,0.02,3.5,1000,7000);
    DEPID_Init(&gimbal.base_yaw.imuPID.deOuter,37,0.03,173,700,2000,0.45);
    PID_Init(&gimbal.base_yaw.imuPID.outer,300,0.5,5000,1000,15000);

    PID_Init(&gimbal.top_yaw.imuPID.inner, 50, 0.1, 10, 7000, 30000);//270 0.3  300
    DEPID_Init(&gimbal.top_yaw.imuPID.deOuter, 150, 0.8, 300, 100, 30000, 0.7);
//
}

// 大小yaw、pitch目标角度计算
void Gimbal_UpdataAngle()
{
    // region 传感器数据传递
    gimbal.top_yaw.gyro = INS.gyro[2];
    gimbal.top_yaw.angle = INS.yaw;
    gimbal.base_yaw.angle = gimbal.top_yaw.angle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0; //得出大yaw的imu角度
    // gimbal.base_yaw.gyro = -gimbal.base_yawMotor.para.vel;   //加负号因为电机倒置 暂时不用 上下板通信不够用
    gimbal.pitch.gyro = -INS.gyro[0];
    gimbal.pitch.angle = INS.pitch;

    gimbal.top_yaw.totalAngle += ModularDegreeTowards(gimbal.top_yaw.angle - gimbal.top_yaw.lastAngle, 0.0f);
    gimbal.top_yaw.targetAngle = ModularDegreeTowards(gimbal.top_yaw.targetAngle, gimbal.top_yaw.totalAngle);

    // endregion
    // region yaw相关角度计算
    // target超出total过半圈时修正
    gimbal.top_yaw.lastAngle = gimbal.top_yaw.angle;
    gimbal.base_yaw.totalAngle = gimbal.top_yaw.totalAngle - (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0 * 360.0;

    if (chassis.pattern == Chassis_AI && rcInfo.left == 2) // 加弹
    {
        gimbal.base_yaw.targetAngle += 45;
    }
}

float yaw_angle = 0.0f;      // 当前扫描角（度）
float yaw_speed = 60.0f;     // 扫描速度（度/秒）

// 扫描参数配置
float pitch_phase = 0.0f;       // 相位
float pitch_freq = 1.0f;        // 频率 Hz
float pitch_amp = 10.0f;        // 振幅 (5 - (-15)) / 2 = 10
float pitch_offset = -5.0f;     // 中心角 (-15 + 5) / 2 = -5

void Gimbal_Scan_Update(void)
{
    static float last_ideal_pitch = 0.0f;
    static uint8_t scan_init_flag = 1; // 首次进入扫描模式的标志位

    // 纯累加，dt = 1ms
    float yaw_delta = yaw_speed * 0.001f;
    gimbal.top_yaw.targetAngle += yaw_delta;
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;

    pitch_phase += 2.0f * PI * pitch_freq * 0.001f;
    if (pitch_phase > 2.0f * PI) {
        pitch_phase -= 2.0f * PI;
    }

    float current_ideal_pitch = pitch_amp * arm_sin_f32(pitch_phase) + pitch_offset;

    if (scan_init_flag) {
        last_ideal_pitch = current_ideal_pitch;
        scan_init_flag = 0;
    }
    float pitch_wave_delta = current_ideal_pitch - last_ideal_pitch;

    float error = current_ideal_pitch - gimbal.pitch.targetAngle;
    float convergence_step = error * 0.02f;

    gimbal.pitch.targetAngle += pitch_wave_delta + convergence_step;

    last_ideal_pitch = current_ideal_pitch;
}

void Gimbal_VisionCtrl()
{
    gimbal.base_yaw.targetAngle = ModularDegreeTowards(vision_target.base_yaw, gimbal.base_yaw.targetAngle);// 大yaw直接瞄就行

    float top_target_unwrap = gimbal.base_yaw.targetAngle + (vision_target.yaw - vision_target.base_yaw);

    // 视觉控制下的大小yaw和pitch的目标角度
    float delta = top_target_unwrap - gimbal.base_yaw.totalAngle; // 计算相对误差 //计算相对误差
    LIMIT(delta, -TOP_YAW_LIMIT, TOP_YAW_LIMIT);

    gimbal.top_yaw.targetAngle = gimbal.base_yaw.totalAngle + delta;//小yaw跟随base

    gimbal.pitch.targetAngle = vision_target.pitch;
    LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

void Gimbal_RockerCtrl()
{
    gimbal.top_yaw.targetAngle -= rcInfo.ch1 * 0.3 / 660.0f;    // yaw
    gimbal.base_yaw.targetAngle = gimbal.top_yaw.targetAngle;
    gimbal.pitch.targetAngle += rcInfo.ch2 * 0.35 / 660.0f; // 旋转云台pitch
    LIMIT(gimbal.pitch.targetAngle, gimbal.pitch.pitchMin, gimbal.pitch.pitchMax);
}

bool Gimbal_VisionForced()
{
    return rcInfo.wheel < -400;
}

// 摇杆控制
void Shooter_RockerCtrl()
{
    static int lastwheel = 0;

    if (rcInfo.wheel > 400 && lastwheel <= 400)
    {
        shooter.fricOpenFlag = !shooter.fricOpenFlag;
    }

    if (!Gimbal_VisionForced())
    {
        if (rcInfo.left == 1)
        {
            shooter.workState = TRIGGER_CONTINUE;
        }
        else
        {
            shooter.workState = IDLE;
        }
    }

	lastwheel = rcInfo.wheel;
//    LIMIT(p_target,MOTOR_ZERO_POS + MOTOR_ZERO_POS - gimbal.pitch.pitchMax * DEG_TO_RAD,MOTOR_ZERO_POS + gimbal.pitch.pitchMin);
} //MOTOR_ZERO_POS

void Task_Gimbal_Callback()
{
    // visionFindAver=Filter_AverCalc(&gimbal.visionFilter.find,vision.tracking);
    // region 非AI控制
    if (chassis.pattern == Chassis_control)
    {
        if (Gimbal_VisionForced() && vision_target.vaild)
        {
            Gimbal_VisionCtrl();
        }
        else
        {
            Gimbal_RockerCtrl();
        }

        Shooter_RockerCtrl();
    }
    else if (chassis.pattern == Chassis_AI)
    {
        if (rcInfo.left == 1)
        {
            if (vision_target.vaild)
            {
                shooter.fricOpenFlag = true;
                Gimbal_VisionCtrl();
            }
            else
            {
                shooter.fricOpenFlag = false;
                Gimbal_Scan_Update();
            }
        }
        else
        {
            Gimbal_RockerCtrl();
        }
    }

    if (chassis.rotate.mode == ChassisMode_Spin)
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 6000;
        gimbal.base_yaw.imuPID.outer.ki = 0.5;
    }
    else
    {
        gimbal.base_yaw.imuPID.outer.maxIntegral = 1000;
        gimbal.base_yaw.imuPID.outer.ki = 0.1;
    }

    // region AI控制
    // 是否开扫描
    //Gimbal_VisionCtrl();
    Gimbal_UpdataAngle();

    //计算小yaw电机输出
    DEPID_CascadeCalc(&gimbal.top_yaw.imuPID,gimbal.top_yaw.targetAngle,gimbal.top_yaw.totalAngle,gimbal.top_yaw.gyro);
    gimbal.top_yaw.imuPID.output = gimbal.top_yaw.imuPID.output;

    // 计算大yaw电机输出
    // DEPID_CascadeCalc(&gimbal.base_yaw.imuPID, gimbal.base_yaw.targetAngle, gimbal.base_yaw.totalAngle, gimbal.base_yaw.gyro);
    // gimbal.base_yaw.imuPID.output = -gimbal.base_yaw.imuPID.output/1000.0f - 2.0f * (gimbal.top_yaw.imuPID.output / 30000.0f) - forwardfeed(gimbal.base_yaw.imuPID.outer.output / 1000.0f);//因为电机倒置 所以输出反向 输出除一千让PID参数乘1000方便调参 再加入前馈
    PID_SingleCalc(&gimbal.base_yaw.imuPID.outer,gimbal.base_yaw.targetAngle,gimbal.base_yaw.totalAngle);
    gimbal.base_yaw.imuPID.outer.output = -gimbal.base_yaw.imuPID.outer.output / 1000.0f;

    // 计算pitch电机输出
//        DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
    //		DEPID_CascadeCalc(&gimbal.pitch.imuPID, gimbal.pitch.targetAngle, gimbal.pitch.angle, gimbal.pitch.gyro);
    PID_SingleCalc(&gimbal.pitch.imuPID.outer,gimbal.pitch.targetAngle,gimbal.pitch.angle);
    gimbal.pitch.imuPID.output = - gimbal.pitch.imuPID.output/1000.0f  - PITCH_MASS * MASS_G * PITCH_R * arm_cos_f32(gimbal.pitch.angle * PI / 180.0f); ////输出除一千让PID参数乘1000方便调参
//		Gimbal_Follow_IMU();
} //-MASS * G * R * arm_cos_f32(gimbal.pitch.angle * PI / 180.0f)

/*************云台操控任务******************/

void OS_GimbalCallback(void const *argument)
{
    osDelay(1500);
    Gimbal_Init();
    for (;;)
    {
        Task_Gimbal_Callback();
        osDelay(1);
    }
}

