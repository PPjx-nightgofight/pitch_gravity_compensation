#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"
#include <stdint.h>

/**
 * @brief 初始化SPI片选引脚（拉高，默认不选中）
 * @note  SPI外设本身由CubeMX的MX_SPIx_Init()初始化，这里只管CS引脚
 *
 * @param cs_port  CS引脚的GPIO端口，如 GPIOB
 * @param cs_pin   CS引脚的引脚号，如 GPIO_PIN_0
 */
void BSP_SPI_CSInit(GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief SPI读写单个字节（全双工）
 *
 * @param hspi  SPI句柄指针，如 &hspi1
 * @param tx    发送的字节
 * @return uint8_t 接收到的字节
 */
uint8_t BSP_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t tx);

/**
 * @brief 通过SPI读取寄存器
 *        自动控制CS片选：拉低→发寄存器地址（读标志）→读数据→拉高
 *
 * @param hspi      SPI句柄
 * @param cs_port   片选端口
 * @param cs_pin    片选引脚
 * @param reg       寄存器地址
 * @param data      存放读取数据的缓冲区
 * @param len       读取长度（字节数）
 */
void BSP_SPI_ReadRegs(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief 通过SPI写入寄存器
 *
 * @param hspi      SPI句柄
 * @param cs_port   片选端口
 * @param cs_pin    片选引脚
 * @param reg       寄存器地址
 * @param data      要写入的数据
 */
void BSP_SPI_WriteReg(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port, uint16_t cs_pin,
                      uint8_t reg, uint8_t data);

#endif /* BSP_SPI_H */
