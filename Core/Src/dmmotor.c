#include "dmmotor.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "string.h"

/* 全局电机实例数组 */
static uint8_t          dm_idx = 0;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

/* =================== 内部工具函数 =================== */

/**
 * @brief  float → uint 映射（用于打包发送帧）
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * (float)((1 << bits) - 1) / span);
}

/**
 * @brief  uint → float 映射（用于解析接收帧）
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

/* =================== 内部通讯函数 =================== */

/**
 * @brief 发送模式切换指令（使能/停止/零位校准等）
 *        达妙协议：前7字节0xFF，最后1字节为命令ID
 */
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xFF, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief CAN接收中断回调，解析电机反馈帧
 *        由 bsp_can 在收到匹配rx_id的报文时自动调用
 *
 * 达妙反馈帧格式（8字节）：
 * Byte[0]        : ID(低4位) | State(高4位)
 * Byte[1~2]      : Position (16bit)
 * Byte[3] + [4高]: Velocity (12bit)
 * Byte[4低] + [5]: Torque   (12bit)
 * Byte[6]        : T_Mos
 * Byte[7]        : T_Rotor
 */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t           tmp;
    uint8_t           *rxbuff = motor_can->rx_buff;
    DMMotorInstance   *motor  = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &motor->measure;

    /* ID 和 状态 */
    measure->id    = rxbuff[0] & 0x0F;
    measure->state = (DMMotor_State_e)((rxbuff[0] >> 4) & 0x0F);

    /* 位置 (16bit) → rad */
    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    /* 单圈角度 (°) */
    measure->angle_single_round = measure->position / (2.0f * PI) * 360.0f;

    /* 累计圈数和累计角度（通过跨圈检测维护） */
    float delta = measure->position - measure->last_position;
    if (delta > PI)
        measure->total_round--;
    else if (delta < -PI)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360.0f
                         + measure->position / (2.0f * PI) * 360.0f;

    /* 速度 (12bit) → rad/s */
    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    /* 力矩 (12bit) → Nm */
    tmp = (uint16_t)(((rxbuff[4] & 0x0F) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    /* 温度 */
    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

/* =================== 对外接口实现 =================== */

DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config)
{
    /* 防止超过最大实例数 */
    if (dm_idx >= DM_MOTOR_CNT)
        while (1); // 超出最大数量，检查 DM_MOTOR_CNT 宏

    /* 分配并清零实例内存 */
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    /* 注册CAN实例，绑定解码回调和电机实例指针 */
    CAN_Init_Config_s can_conf = {
        .can_handle          = config->can_handle,
        .tx_id               = config->tx_id,
        .rx_id               = config->rx_id,
        .can_module_callback = DMMotorDecode,
        .id                  = motor, // 回调时通过 motor_can->id 反向找到本实例
    };
    motor->motor_can_instace = CANRegister(&can_conf);

    motor->stop_flag  = DM_MOTOR_ENABLED;
    motor->torque_set = 0.0f;

    /* 发送使能指令，稍作延时等待电机响应 */
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1f); // 100ms

    /* 保存到全局数组 */
    dm_motor_instance[dm_idx++] = motor;
    return motor;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = DM_MOTOR_ENABLED;
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag  = DM_MOTOR_STOP;
    motor->torque_set = 0.0f;
    /* 立即发一帧零力矩确保电机不再输出 */
    DMMotorSendTorque(motor);
}

void DMMotorSetTorque(DMMotorInstance *motor, float torque_nm)
{
    /* 限幅到电机物理范围 */
    LIMIT_MIN_MAX(torque_nm, DM_T_MIN, DM_T_MAX);
    motor->torque_set = torque_nm;
}

/**
 * @brief 打包 MIT 格式控制帧并通过CAN发送
 *
 * 达妙 MIT 模式发送帧格式（8字节）：
 * Byte[0~1]      : position_des (16bit)
 * Byte[2] + [3高]: velocity_des (12bit)
 * Byte[3低] + [4]: Kp (12bit)
 * Byte[5] + [6高]: Kd (12bit)
 * Byte[6低] + [7]: torque_des (12bit)
 *
 * 此处仅使用纯力矩控制：position=0, velocity=0, Kp=0, Kd=0
 */
void DMMotorSendTorque(DMMotorInstance *motor)
{
    float torque = (motor->stop_flag == DM_MOTOR_STOP) ? 0.0f : motor->torque_set;

    uint16_t pos_uint = float_to_uint(0.0f,   DM_P_MIN, DM_P_MAX, 16);
    uint16_t vel_uint = float_to_uint(0.0f,   DM_V_MIN, DM_V_MAX, 12);
    uint16_t tor_uint = float_to_uint(torque, DM_T_MIN, DM_T_MAX, 12);
    uint16_t kp_uint  = 0;
    uint16_t kd_uint  = 0;

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    buf[0] = (uint8_t)(pos_uint >> 8);
    buf[1] = (uint8_t)(pos_uint);
    buf[2] = (uint8_t)(vel_uint >> 4);
    buf[3] = (uint8_t)(((vel_uint & 0xF) << 4) | (kp_uint >> 8));
    buf[4] = (uint8_t)(kp_uint);
    buf[5] = (uint8_t)(kd_uint >> 4);
    buf[6] = (uint8_t)(((kd_uint & 0xF) << 4) | (tor_uint >> 8));
    buf[7] = (uint8_t)(tor_uint);

    CANTransmit(motor->motor_can_instace, 1);
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1f);
}

void DMMotorSendPosVelTorque(DMMotorInstance *motor,
                              float pos_des_deg,  // 改：参数名加 _deg 更清晰
                              float vel_des,
                              float kp,
                              float kd,
                              float torque_ff)
{
    /* ① 角度转弧度，函数内部处理，外部只需传角度 */
    float pos_des = pos_des_deg * 0.01745329f;

    /* 停止状态下强制清零 */
    if (motor->stop_flag == DM_MOTOR_STOP)
    {
        DMMotorSendTorque(motor);
        return;
    }

    /* 位置限幅 */
    if (pos_des >  DM_P_MAX) pos_des =  DM_P_MAX;
    if (pos_des < -DM_P_MAX) pos_des = -DM_P_MAX;

    LIMIT_MIN_MAX(torque_ff, DM_T_MIN, DM_T_MAX);

    uint16_t pos_uint = float_to_uint(pos_des,   DM_P_MIN, DM_P_MAX, 16);
    uint16_t vel_uint = float_to_uint(vel_des,   DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_uint  = float_to_uint(kp,        0.0f,     500.0f,   12);
    uint16_t kd_uint  = float_to_uint(kd,        0.0f,     5.0f,     12);
    uint16_t tor_uint = float_to_uint(torque_ff, DM_T_MIN, DM_T_MAX, 12);

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    buf[0] = (uint8_t)(pos_uint >> 8);
    buf[1] = (uint8_t)(pos_uint);
    buf[2] = (uint8_t)(vel_uint >> 4);
    buf[3] = (uint8_t)(((vel_uint & 0xF) << 4) | (kp_uint >> 8));
    buf[4] = (uint8_t)(kp_uint);
    buf[5] = (uint8_t)(kd_uint >> 4);
    buf[6] = (uint8_t)(((kd_uint & 0xF) << 4) | (tor_uint >> 8));
    buf[7] = (uint8_t)(tor_uint);

    CANTransmit(motor->motor_can_instace, 1);
}
