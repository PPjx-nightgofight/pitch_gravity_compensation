#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "bsp_can.h"
#include <stdint.h>
#include <math.h>

/* 最大电机数量 */
#define DM_MOTOR_CNT 4

/* 达妙电机MIT模式物理量范围 */
#define DM_P_MIN   (-3.141593f)
#define DM_P_MAX   (3.141593f)
#define DM_V_MIN   (-30.0f)
#define DM_V_MAX   (30.0f)
#define DM_T_MIN   (-10.0f)
#define DM_T_MAX   (10.0f)

#ifndef PI
#define PI 3.141592f
#endif

/* 宏：限幅 */
#define LIMIT_MIN_MAX(x, min, max) \
    do { if ((x) < (min)) (x) = (min); else if ((x) > (max)) (x) = (max); } while(0)

/* ---------------------- 电机状态枚举 ---------------------- */
typedef enum
{
    DM_STATE_DISABLE        = 0x00, // 失能
    DM_STATE_ENABLE         = 0x01, // 使能
    DM_STATE_OVER_VOLT      = 0x08, // 超压
    DM_STATE_UNDER_VOLT     = 0x09, // 欠压
    DM_STATE_OVER_CURRENT   = 0x0a, // 过电流
    DM_STATE_MOS_OVER_TEMP  = 0x0b, // MOS过温
    DM_STATE_COIL_OVER_TEMP = 0x0c, // 线圈过温
    DM_STATE_COMM_LOSS      = 0x0d, // 通讯丢失
    DM_STATE_OVER_LOAD      = 0x0e, // 过载
} DMMotor_State_e;

/* ---------------------- 电机模式指令枚举 ---------------------- */
typedef enum
{
    DM_CMD_MOTOR_MODE    = 0xfc, // 使能（进入控制模式）
    DM_CMD_RESET_MODE    = 0xfd, // 停止（退出控制模式）
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前位置设为编码器零位
    DM_CMD_CLEAR_ERROR   = 0xfb, // 清除错误
} DMMotor_Mode_e;

/* ---------------------- 电机测量值结构体 ---------------------- */
typedef struct
{
    uint8_t         id;                 // 电机ID
    DMMotor_State_e state;              // 电机状态
    float           position;           // 当前角度 (rad, 单圈 -π ~ π)
    float           last_position;      // 上一次角度 (rad)
    float           angle_single_round; // 单圈角度 (°, 0~360)
    float           total_angle;        // 累计角度 (°, 可多圈)
    float           velocity;           // 转速 (rad/s)
    float           torque;             // 力矩 (Nm)
    float           T_Mos;              // MOS管温度 (°C)
    float           T_Rotor;            // 转子温度 (°C)
    int32_t         total_round;        // 累计圈数
} DM_Motor_Measure_s;

/* ---------------------- 电机初始化配置 ---------------------- */
typedef struct
{
    CAN_HandleTypeDef *can_handle; // CAN句柄，如 &hcan1
    uint32_t           tx_id;      // 发送ID（即电机ID，通常为 0x01~0x08）
    uint32_t           rx_id;      // 接收ID（通常为 tx_id + 0x10）
} DMMotor_Init_Config_s;

/* ---------------------- 电机实例结构体 ---------------------- */
typedef struct
{
    DM_Motor_Measure_s measure;          // 电机反馈数据
    CANInstance       *motor_can_instace; // CAN实例
    uint8_t            stop_flag;         // 0:运行中, 1:停止
    float              torque_set;        // 当前力矩设定值 (Nm)
} DMMotorInstance;

/* stop_flag 枚举值 */
#define DM_MOTOR_ENABLED 0
#define DM_MOTOR_STOP    1

/* ===================== 对外接口函数 ===================== */

/**
 * @brief  初始化一个达妙电机实例并发送使能指令
 * @param  config  初始化配置指针
 * @return 电机实例指针
 */
DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config);

/**
 * @brief  使能电机（允许输出力矩）
 */
void DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief  停止电机（力矩清零，但仍处于使能通讯状态）
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief  设置目标力矩 (Nm)
 * @param  torque_nm  力矩值，范围 DM_T_MIN ~ DM_T_MAX
 */
void DMMotorSetTorque(DMMotorInstance *motor, float torque_nm);

/**
 * @brief  发送当前力矩指令（需在控制循环中周期调用）
 *         调用前请先通过 DMMotorSetTorque() 设置力矩
 */
void DMMotorSendTorque(DMMotorInstance *motor);

/**
 * @brief  将当前位置设为编码器零点（零位校准）
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);

//位置速度模式使用
/**
 * @param pos_des_deg 目标位置，单位：角度(°)，范围 -180° ~ 180°
 * @param vel_des     目标速度 (rad/s)，保持静止给 0
 * @param kp          位置增益，范围 0 ~ 500
 * @param kd          速度增益（阻尼），范围 0 ~ 5
 * @param torque_ff   力矩前馈 (Nm)
 */
void DMMotorSendPosVelTorque(DMMotorInstance *motor,
                              float pos_des_deg,  // 改：参数名加 _deg 更清晰
                              float vel_des,
                              float kp,
                              float kd,
                              float torque_ff);



#endif /* DMMOTOR_H */
