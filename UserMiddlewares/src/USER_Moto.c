/**
 ******************************************************************************
 * @file    USER_Moto.c
 * @brief   电机驱动接口层：大疆(DJI)电机与达妙(DM)电机的数据解析、角度累计与控制指令发送
 ******************************************************************************
 * @details 本文件包含以下功能：
 *          1. 大疆电机累计角度计算：Motor_StartCalcAngle / Motor_CalcAngle
 *             （编码器 0~8191 对应 0~360°，含跨零点绕圈识别）
 *          2. 达妙舵电机累计角度计算：Motor_StartCalcAngle_J4310 / Motor_CalcAngle_J4310
 *             （反馈角度约 -180°~180°，含跨零点绕圈识别）
 *          3. 浮点数与整型编码值换算：float_to_uint / uint_to_float（达妙 MIT 协议使用）
 *          4. 反馈数据更新：DJIMotor_Update（大疆）/ dm4310_fbdata（达妙 4310 解析）
 *          5. 达妙电机指令：enable_motor_mode / disable_motor_mode / clear_err / mit_ctrl
 *          6. 大疆电机指令：USER_CAN_SetMotorCurrent
 ******************************************************************************
 */
#include "USER_Moto.h"
#include "degree.h"
#include "dji_angle.h"

/********************积累DJI电机累计角度************************/
/**
 * @brief  大疆电机累计角度计算初始化
 * @param  motor 大疆电机结构体指针
 * @note   在开始进行累计角度计算前调用一次：
 *         - totalAngle 清零，作为累计角度的零点；
 *         - lastAngle 记录当前编码器角度，作为后续差分的基准；
 *         - targetAngle 清零。
 */
void Motor_StartCalcAngle(DJI_Motor_t *motor)
{
    motor->totalAngle = 0; // 累计编码器值清零
    motor->lastAngle = motor->angle; // 记录当前编码器角度，作为差分基准
    motor->targetAngle = 0; // 目标角度(编码器值)清零
}

// 计算电机累计转过的圈数
/**
 * @brief  计算大疆电机累计转过的编码器值（带绕圈识别）
 * @param  motor 大疆电机结构体指针
 * @note   大疆电机编码器为单圈 0~8191（对应 0~360°）。
 *         当本周期角度与上周期角度之差超过 ±4000（约 ±180°）时，判定电机跨越了编码器零点：
 *         - 差值 < -4000（如 8000 -> 100，正向跨零）：增量 = 当前值 + (8191 - 上次值)；
 *         - 差值 > +4000（如 100 -> 8000，反向跨零）：增量 = -上次值 - (8191 - 当前值)。
 *         计算出的增量累加到 totalAngle，最后更新 lastAngle。
 */
void Motor_CalcAngle(DJI_Motor_t *motor)
{
    // 将角度增量加入计数器
    motor->totalAngle += ModularDJIAngleTowards(motor->angle - motor->lastAngle, 0);
    // 记录角度
    motor->lastAngle = motor->angle;
}

/********************积累舵电机累计角度************************/
/**
 * @brief  达妙 J4310 舵电机累计角度计算初始化
 * @param  motor 达妙舵电机结构体指针（Double_motor_t）
 * @note   以当前实际角度 TurnAngle 为起点初始化 totalAngle：
 *         若起点大于 180°，则减去 360°，把起点归一化到 -180°~180° 区间，
 *         保证累计角度在零点附近连续；同时记录 lastAngle 作为差分基准。
 */
void Motor_StartCalcAngle_J4310(Double_motor_t *motor)
{
    motor->totalAngle = ModularDegreeTowards(motor->TurnAngle, 0.0f);
    motor->lastAngle = motor->TurnAngle; // 记录当前角度，作为差分基准
}

/**
 * @brief  计算达妙 J4310 舵电机累计转过的角度（带绕圈识别，单位：°）
 * @param  motor 达妙舵电机结构体指针（Double_motor_t）
 * @note   电机反馈角度范围约 -180°~180°，相邻两周期角度差超过 ±180° 时判定为跨越零点：
 *         - 差值 < -180°（如 170° -> -170°，正向跨零）：增量 = 当前值 + (360 - 上次值)；
 *         - 差值 > +180°（如 -170° -> 170°，反向跨零）：增量 = -上次值 - (360 - 当前值)。
 */
void Motor_CalcAngle_J4310(Double_motor_t *motor)
{
    motor->totalAngle += ModularDegreeTowards(motor->TurnAngle - motor->lastAngle, 0.0f);
    motor->lastAngle = motor->TurnAngle; // 记录本周期角度，供下周期差分
}

/**
 * @brief  浮点数线性编码为无符号整型（达妙 MIT 协议编码用）
 * @param  x_float 待转换的浮点值（应在 [x_min, x_max] 范围内）
 * @param  x_min   量程下限
 * @param  x_max   量程上限
 * @param  bits    目标整型的有效位数（如 16 / 12）
 * @return 转换后的整型编码值，范围 0 ~ 2^bits-1
 * @note   映射关系：out = (x_float - x_min) * (2^bits - 1) / (x_max - x_min)
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min; // 量程跨度 = 上限 - 下限
    float offset = x_min; // 零点偏移 = 下限
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span); // 归一化映射到整型量程并取整
}

/**
 * @brief  无符号整型反解为浮点数（达妙 MIT 协议解码用）
 * @param  x_int 待转换的整型编码值
 * @param  x_min 量程下限
 * @param  x_max 量程上限
 * @param  bits  整型的有效位数（如 16 / 12）
 * @return 反解出的浮点值，范围 [x_min, x_max]
 * @note   映射关系：out = x_int * (x_max - x_min) / (2^bits - 1) + x_min
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset; // 整型量程映射回浮点量程
}

/**
 * @brief  大疆电机反馈数据更新
 * @param  motor  大疆电机结构体指针
 * @param  angle  机械角度（编码器值，单圈 0~8191，按协议解析后可能为负）
 * @param  speed  转子转速（rpm 原始值）
 * @param  torque 实际转矩电流（原始值）
 * @param  temp   电机温度（℃，有符号）
 * @note   在 CAN 接收回调中调用，将一帧反馈数据写入电机结构体。
 */
void DJIMotor_Update(DJI_Motor_t *motor, int16_t angle, int16_t speed, int16_t torque, int8_t temp) //大疆电机数据更新
{
    motor->angle = angle; // 更新编码器角度
    motor->speed = speed; // 更新转子转速
    motor->torque = torque; // 更新实际转矩电流
    motor->temp = temp; // 更新电机温度
}

/**
 * @brief  达妙 4310（大喵）电机反馈数据解析与更新
 * @param  motor   达妙电机结构体指针
 * @param  rx_data 指向 8 字节 CAN 反馈原始数据的指针
 * @note   MIT 协议反馈帧格式（按位打包）：
 *         - rx_data[0]   ：低 4 位为控制器 ID，高 4 位为电机状态；
 *         - rx_data[1..2]：位置整型量 p_int（16bit）；
 *         - rx_data[3..4]：速度整型量 v_int（12bit，高 4 位位于数据[4]高半字节）；
 *         - rx_data[4..5]：扭矩整型量 t_int（12bit，低 4 位位于数据[4]低半字节）；
 *         - rx_data[6]   ：MOS 管温度（℃）；
 *         - rx_data[7]   ：电机线圈温度（℃）。
 *         各整型量再通过 uint_to_float 按对应量程还原为物理量。
 */
void dm4310_fbdata(DM_motor_t *motor, uint8_t *rx_data)//大喵4310数据更新
{
    motor->para.id = (rx_data[0])&0x0F; // 控制器 ID（数据[0]低 4 位）
    motor->para.state = (rx_data[0])>>4; // 电机状态（数据[0]高 4 位）
    motor->para.p_int=(rx_data[1]<<8)|rx_data[2]; // 位置整型量（16bit）
    motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4); // 速度整型量（12bit）
    motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5]; // 扭矩整型量（12bit）
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // 位置(rad)，量程 (-12.5,12.5) // (-12.5,12.5)
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // 速度(rad/s)，量程 (-30.0,30.0) // (-30.0,30.0)
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); // 扭矩(N·m)，量程 (-10.0,10.0) // (-10.0,10.0)
    motor->para.Tmos = (float)(rx_data[6]); // MOS 管温度（℃）
    motor->para.Tcoil = (float)(rx_data[7]); // 电机线圈温度（℃）
}


/**
 * @brief  达妙电机使能
 * @param  hcan     FDCAN 句柄指针
 * @param  motor_id 电机 CAN ID
 * @param  mode_id  模式偏移（MIT_MODE / POS_MODE / SPD_MODE / PSI_MODE）
 * @note   发送 8 字节数据 FF FF FF FF FF FF FF FC，最后一字节 0xFC 表示使能。
 */
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id) //大喵电机使能
{
    uint8_t data[8]; // CAN 发送数据缓冲区（8 字节）
    uint16_t id = motor_id + mode_id; // 实际发送的 CAN ID = 电机 ID + 模式偏移

    data[0] = 0xFF; // 使能指令数据帧（前 7 字节固定为 0xFF）
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC; // 0xFC：电机使能命令

    USER_CAN_Send(hcan, id, data); // 通过 CAN 发送使能指令
}

/**
 * @brief  达妙电机失能
 * @param  hcan     FDCAN 句柄指针
 * @param  motor_id 电机 CAN ID
 * @param  mode_id  模式偏移（MIT_MODE / POS_MODE / SPD_MODE / PSI_MODE）
 * @note   发送 8 字节数据 FF FF FF FF FF FF FF FD，最后一字节 0xFD 表示失能。
 */
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)//大喵电机失能
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF; // 失能指令数据帧（前 7 字节固定为 0xFF）
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD; // 0xFD：电机失能命令

    USER_CAN_Send(hcan, id, data); // 通过 CAN 发送失能指令
}

/**
 * @brief  达妙电机 MIT 模式控制（位置/速度/力矩混合控制）
 * @param  hcan     FDCAN 句柄指针
 * @param  motor_id 电机 CAN ID
 * @param  pos      目标位置（rad，量程 P_MIN~P_MAX）
 * @param  vel      目标速度（rad/s，量程 V_MIN~V_MAX）
 * @param  kp       位置比例增益（量程 KP_MIN~KP_MAX）
 * @param  kd       速度微分增益（量程 KD_MIN~KD_MAX）
 * @param  torq     前馈力矩（N·m，量程 T_MIN~T_MAX）
 * @note   输出力矩公式：τ = kp*(pos - 实际位置) + kd*(vel - 实际速度) + torq。
 *         各物理量先按量程线性编码为整型（位置 16bit，其余 12bit），
 *         再按 MIT 协议位域打包成 8 字节发送。
 */
void mit_ctrl(FDCAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)//mit模式 控制大喵电机
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp; // 各物理量编码后的整型临时变量
    uint16_t id = motor_id + MIT_MODE; // MIT 模式实际发送的 CAN ID

    pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16); // 目标位置 -> 16bit 整型
    vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12); // 目标速度 -> 12bit 整型
    kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12); // 位置增益 Kp -> 12bit 整型
    kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12); // 速度增益 Kd -> 12bit 整型
    tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12); // 前馈力矩 -> 12bit 整型

    data[0] = (pos_tmp >> 8); // 位置：高 8 位
    data[1] = pos_tmp; // 位置：低 8 位
    data[2] = (vel_tmp >> 4); // 速度：高 8 位
    data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8); // 速度低 4 位 + Kp 高 4 位
    data[4] = kp_tmp; // Kp：低 8 位
    data[5] = (kd_tmp >> 4); // Kd：高 8 位
    data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8); // Kd 低 4 位 + 力矩高 4 位
    data[7] = tor_tmp; // 力矩：低 8 位

    USER_CAN_Send(hcan, id, data); // 通过 CAN 发送 MIT 控制指令
}

/**
 * @brief  达妙电机清除错误
 * @param  hfdcan   FDCAN 句柄指针
 * @param  motor_id 电机 CAN ID
 * @param  mode_id  模式偏移（MIT_MODE / POS_MODE / SPD_MODE / PSI_MODE）
 * @note   发送 8 字节数据 FF FF FF FF FF FF FF FB，最后一字节 0xFB 表示清除错误。
 */
void clear_err(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF; // 清错指令数据帧（前 7 字节固定为 0xFF）
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB; // 0xFB：清除电机错误命令

    USER_CAN_Send(hfdcan, id, data); // 通过 CAN 发送清错指令
}


//发送电机电流信息 控制DJI电机
/**
 * @brief  发送大疆电机电流（转矩电流）控制指令
 * @param  hfdcan FDCAN 句柄指针
 * @param  StdId  标准帧 ID（如 0x1FF 控制 ID 1~4 的电机，0x2FF 控制 ID 5~8 的电机）
 * @param  iq1    第 1 个电机的转矩电流（-16384~16384 对应 -20A~20A）
 * @param  iq2    第 2 个电机的转矩电流
 * @param  iq3    第 3 个电机的转矩电流
 * @param  iq4    第 4 个电机的转矩电流
 * @note   一帧 8 字节可同时控制 4 个电机：每 2 字节存一个大端序 int16 电流值。
 */
void USER_CAN_SetMotorCurrent(FDCAN_HandleTypeDef* hfdcan,int16_t StdId,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t tx_data[8]={0}; // CAN 发送数据缓冲区（8 字节），初值全 0
    tx_data[0] = (iq1 >> 8) & 0xff; // 电机 1 电流：高字节
    tx_data[1] = (iq1) & 0xff; // 电机 1 电流：低字节
    tx_data[2] = (iq2 >> 8) & 0xff; // 电机 2 电流：高字节
    tx_data[3] = (iq2) & 0xff; // 电机 2 电流：低字节
    tx_data[4] = (iq3 >> 8) & 0xff; // 电机 3 电流：高字节
    tx_data[5] = (iq3) & 0xff; // 电机 3 电流：低字节
    tx_data[6] = (iq4 >> 8) & 0xff; // 电机 4 电流：高字节
    tx_data[7] = (iq4) & 0xff; // 电机 4 电流：低字节

    USER_CAN_Send(hfdcan,StdId, tx_data); // 通过 CAN 发送四路电流控制指令
}





