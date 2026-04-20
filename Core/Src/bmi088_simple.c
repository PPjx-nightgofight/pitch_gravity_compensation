#include "bmi088_simple.h"
#include "bsp_dwt.h"
#include <math.h>
#include <string.h>

/* ============================================================
 *  内部辅助宏
 * ============================================================ */
#define ACC_CS_PORT   BMI088_ACC_CS_PORT
#define ACC_CS_PIN    BMI088_ACC_CS_PIN
#define GYRO_CS_PORT  BMI088_GYRO_CS_PORT
#define GYRO_CS_PIN   BMI088_GYRO_CS_PIN
#define ACC_SPI       BMI088_ACC_SPI
#define GYRO_SPI      BMI088_GYRO_SPI

/* ============================================================
 *  加速度计内部函数
 * ============================================================ */

/**
 * @brief 写加速度计寄存器（封装cs）
 */
static void ACC_WriteReg(uint8_t reg, uint8_t data)
{
    BSP_SPI_WriteReg(ACC_SPI, ACC_CS_PORT, ACC_CS_PIN, reg, data);
}

/**
 * @brief 读加速度计寄存器
 * @note  BMI088 加速度计在 SPI 读操作后有一个 dummy byte，需要额外读一次
 */
static void ACC_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t dummy;
    /* 拉低CS */
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);
    /* 发送寄存器地址（最高位=1表示读） */
    BSP_SPI_ReadWriteByte(ACC_SPI, reg | 0x80);
    /* 加速度计特有：读一个dummy byte */
    dummy = BSP_SPI_ReadWriteByte(ACC_SPI, 0x55);
    (void)dummy;
    /* 读取实际数据 */
    for (uint8_t i = 0; i < len; i++)
        data[i] = BSP_SPI_ReadWriteByte(ACC_SPI, 0x55);
    /* 拉高CS */
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
}

static uint8_t ACC_Init(void)
{
    uint8_t chip_id = 0;

    /* 软复位 */
    ACC_WriteReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VAL);
    DWT_Delay_ms(80); // 加速度计复位需要 ~80ms

    /* 读一次dummy以激活SPI模式（加速度计上电后首次SPI读需要哑读） */
    ACC_ReadRegs(BMI088_ACC_CHIP_ID, &chip_id, 1);
    /* 再次读取 */
    ACC_ReadRegs(BMI088_ACC_CHIP_ID, &chip_id, 1);

    if (chip_id != BMI088_ACC_CHIP_ID_VALUE)
        return 1; // ID不匹配

    /* 开启加速度计 */
    ACC_WriteReg(BMI088_ACC_PWR_CTRL, 0x04); // 使能ACC
    DWT_Delay_ms(5);
    ACC_WriteReg(BMI088_ACC_PWR_CONF, 0x00); // 退出低功耗
    DWT_Delay_ms(5);

    /* 配置量程 ±6g，ODR 800Hz，Normal带宽 */
    ACC_WriteReg(BMI088_ACC_CONF,  0x80 | 0x20 | 0x0B); // NORMAL | 800Hz
    ACC_WriteReg(BMI088_ACC_RANGE, 0x01);                // ±6g

    DWT_Delay_ms(5);
    return 0;
}

/* ============================================================
 *  陀螺仪内部函数
 * ============================================================ */

static void GYRO_WriteReg(uint8_t reg, uint8_t data)
{
    BSP_SPI_WriteReg(GYRO_SPI, GYRO_CS_PORT, GYRO_CS_PIN, reg, data);
}

static void GYRO_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    /* 陀螺仪 SPI 读：没有 dummy byte，直接读 */
    BSP_SPI_ReadRegs(GYRO_SPI, GYRO_CS_PORT, GYRO_CS_PIN, reg, data, len);
}

static uint8_t GYRO_Init(void)
{
    uint8_t chip_id = 0;

    /* 软复位 */
    GYRO_WriteReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VAL);
    DWT_Delay_ms(30);

    GYRO_ReadRegs(BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE)
        return 1;

    /* 量程 ±2000°/s */
    GYRO_WriteReg(BMI088_GYRO_RANGE,     0x00);
    /* ODR 1000Hz, 滤波带宽 116Hz */
    GYRO_WriteReg(BMI088_GYRO_BANDWIDTH, 0x02);
    /* 正常工作模式 */
    GYRO_WriteReg(BMI088_GYRO_LPM1,      0x00);

    DWT_Delay_ms(5);
    return 0;
}

/* ============================================================
 *  对外接口实现
 * ============================================================ */

uint8_t BMI088_Init(BMI088_t *bmi)
{
    memset(bmi, 0, sizeof(BMI088_t));

    /* 初始化CS引脚（拉高） */
//    BSP_SPI_CSInit(ACC_CS_PORT,  ACC_CS_PIN);
//    BSP_SPI_CSInit(GYRO_CS_PORT, GYRO_CS_PIN);

    /* 初始化加速度计 */
    if (ACC_Init() != 0)
        return 1; // 加速度计初始化失败，检查SPI接线和CS引脚

    /* 初始化陀螺仪 */
    if (GYRO_Init() != 0)
        return 2; // 陀螺仪初始化失败

    bmi->is_ready = 1;
    return 0;
}

void BMI088_ReadSensor(BMI088_t *bmi)
{
    uint8_t buf[6];
    int16_t raw;

    /* ---- 读加速度计（6字节，XYZ各2字节） ---- */
    ACC_ReadRegs(BMI088_ACCEL_XOUT_L, buf, 6);
    for (int i = 0; i < 3; i++)
    {
        raw = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]);
        bmi->acc[i] = (float)raw * BMI088_ACC_6G_SEN;
    }

    /* ---- 读陀螺仪（6字节，XYZ各2字节） ---- */
    GYRO_ReadRegs(BMI088_GYRO_X_L, buf, 6);
    for (int i = 0; i < 3; i++)
    {
        raw = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]);
        bmi->gyro[i] = (float)raw * BMI088_GYRO_2000_SEN - bmi->gyro_offset[i];
    }
}

void BMI088_UpdateAttitude(BMI088_t *bmi)
{
    /*
     * 用加速度计计算静态俯仰角和横滚角
     * 坐标系：X朝前，Y朝左，Z朝上（右手系）
     * 根据你的C板实际安装方向可能需要调整轴的映射
     *
     * Pitch（俯仰，绕Y轴）= atan2(-acc_x, sqrt(acc_y2 + acc_z2))
     * Roll （横滚，绕X轴）= atan2(acc_y, acc_z)
     */
    float ax = bmi->acc[0];
    float ay = bmi->acc[1];
    float az = bmi->acc[2];

    bmi->pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f);
    bmi->roll  = atan2f( ay, az)                        * (180.0f / 3.14159265f);
}

void BMI088_CalibrateGyro(BMI088_t *bmi)
{
    float sum[3] = {0};
    int   samples = 500;

    for (int i = 0; i < samples; i++)
    {
        BMI088_ReadSensor(bmi);
        /* 注意：此时 gyro_offset 还是 0，ReadSensor 读的是原始值 */
        sum[0] += bmi->gyro[0];
        sum[1] += bmi->gyro[1];
        sum[2] += bmi->gyro[2];
        DWT_Delay_ms(2);
    }

    bmi->gyro_offset[0] = sum[0] / samples;
    bmi->gyro_offset[1] = sum[1] / samples;
    bmi->gyro_offset[2] = sum[2] / samples;
    /* 标定完成后 ReadSensor 会自动减去偏移 */
}
