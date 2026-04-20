#include "bsp_spi.h"

/* CS引脚操作宏 */
#define CS_LOW(port, pin)   HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define CS_HIGH(port, pin)  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)

/* BMI088 SPI 读操作：寄存器地址最高位置1 */
#define SPI_READ_FLAG  0x80

void BSP_SPI_CSInit(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    CS_HIGH(cs_port, cs_pin); // 初始拉高，不选中
}

uint8_t BSP_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t tx)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(hspi, &tx, &rx, 1, 10); // 10ms超时
    return rx;
}

void BSP_SPI_ReadRegs(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      uint8_t reg, uint8_t *data, uint8_t len)
{
    CS_LOW(cs_port, cs_pin);

    BSP_SPI_ReadWriteByte(hspi, reg | SPI_READ_FLAG); // 发送寄存器地址+读标志

    /* BMI088加速度计在SPI读操作后有一个dummy byte，陀螺仪没有 */
    /* 此处统一处理，调用者根据实际情况决定是否跳过第一个返回字节 */
    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = BSP_SPI_ReadWriteByte(hspi, 0x55); // 0x55为dummy发送值
    }

    CS_HIGH(cs_port, cs_pin);
}

void BSP_SPI_WriteReg(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      uint8_t reg, uint8_t data)
{
    CS_LOW(cs_port, cs_pin);

    BSP_SPI_ReadWriteByte(hspi, reg & ~SPI_READ_FLAG); // 最高位清0表示写操作
    BSP_SPI_ReadWriteByte(hspi, data);

    CS_HIGH(cs_port, cs_pin);
}
