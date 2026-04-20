#ifndef BMI088_SIMPLE_H
#define BMI088_SIMPLE_H

#include "main.h"
#include "bsp_spi.h"
#include "spi.h" 
#include <stdint.h>

/* ============================================================
 *  裸机精简版 BMI088 驱动
 *  去除：加热PID、RTOS依赖、中断模式（先用轮询）
 *  保留：加速度计 + 陀螺仪 SPI 读取，用于Pitch角和角速度获取
 * ============================================================ */

/* ---------- 需要根据你的C板原理图填写的硬件配置 ---------- */

/* 加速度计 SPI 句柄，根据CubeMX实际配置修改 */
#define BMI088_ACC_SPI       (&hspi1)
/* 加速度计 CS 引脚，根据原理图修改 */
#define BMI088_ACC_CS_PORT   CS1_ACCEL_GPIO_Port
#define BMI088_ACC_CS_PIN    CS1_ACCEL_Pin

/* 陀螺仪 SPI 句柄（C板上acc和gyro通常共用同一SPI） */
#define BMI088_GYRO_SPI      (&hspi1)
/* 陀螺仪 CS 引脚，根据原理图修改 */
#define BMI088_GYRO_CS_PORT  CS1_GYRO_GPIO_Port
#define BMI088_GYRO_CS_PIN   CS1_GYRO_Pin

/* ---------- 物理量换算系数 ---------- */
/* 加速度计量程 ±6g 时的换算系数 (m/s2/LSB) */
#define BMI088_ACC_6G_SEN    0.00179443359375f
/* 陀螺仪量程 ±2000°/s 时的换算系数 (rad/s/LSB) */
#define BMI088_GYRO_2000_SEN 0.00106526443603f

/* ---------- 关键寄存器地址（精简，仅保留必要的） ---------- */
#define BMI088_ACC_CHIP_ID        0x00
#define BMI088_ACC_CHIP_ID_VALUE  0x1E
#define BMI088_ACCEL_XOUT_L       0x12
#define BMI088_ACC_CONF           0x40
#define BMI088_ACC_RANGE          0x41
#define BMI088_ACC_PWR_CONF       0x7C
#define BMI088_ACC_PWR_CTRL       0x7D
#define BMI088_ACC_SOFTRESET      0x7E
#define BMI088_ACC_SOFTRESET_VAL  0xB6

#define BMI088_GYRO_CHIP_ID       0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F
#define BMI088_GYRO_X_L           0x02
#define BMI088_GYRO_RANGE         0x0F
#define BMI088_GYRO_BANDWIDTH     0x10
#define BMI088_GYRO_LPM1          0x11
#define BMI088_GYRO_SOFTRESET     0x14
#define BMI088_GYRO_SOFTRESET_VAL 0xB6

/* ---------- 数据结构 ---------- */
typedef struct
{
    /* 原始物理量（已换算） */
    float acc[3];   // 加速度 m/s2，索引: 0=X, 1=Y, 2=Z
    float gyro[3];  // 角速度 rad/s，索引: 0=X, 1=Y, 2=Z
    float temp;     // 温度 °C（可选读取）

    /* 姿态角（由 BMI088_UpdateAttitude 计算） */
    float pitch;    // 俯仰角 °（绕Y轴），正值=抬头
    float roll;     // 横滚角 °（绕X轴）

    /* 陀螺仪零偏（上电标定） */
    float gyro_offset[3];

    /* 初始化标志 */
    uint8_t is_ready;
} BMI088_t;

/* ---------- 对外接口 ---------- */

/**
 * @brief 初始化 BMI088（加速度计 + 陀螺仪）
 * @note  调用前需确保：
 *        1. CubeMX 已初始化对应 SPI 外设
 *        2. CS 引脚已配置为 GPIO 输出
 *        3. DWT 已初始化（用于延时）
 *
 * @param bmi 传入已分配的 BMI088_t 结构体指针
 * @return 0: 成功, 非0: 失败（加速度计或陀螺仪ID不匹配）
 */
uint8_t BMI088_Init(BMI088_t *bmi);

/**
 * @brief 读取加速度计和陀螺仪原始数据，更新 bmi->acc 和 bmi->gyro
 * @note  在主循环中周期调用（建议 1ms 或 2ms）
 */
void BMI088_ReadSensor(BMI088_t *bmi);

/**
 * @brief 用加速度计计算静态 Pitch / Roll 角
 * @note  仅适合低速或静止场景，动态需用互补滤波或Mahony
 *        结果存入 bmi->pitch 和 bmi->roll
 */
void BMI088_UpdateAttitude(BMI088_t *bmi);

/**
 * @brief 陀螺仪零偏标定
 * @note  调用时云台必须静止，采样约 500 次取均值
 *        结果存入 bmi->gyro_offset，之后 BMI088_ReadSensor 自动减去偏移
 */
void BMI088_CalibrateGyro(BMI088_t *bmi);

#endif /* BMI088_SIMPLE_H */
