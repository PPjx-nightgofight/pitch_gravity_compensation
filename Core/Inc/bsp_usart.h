#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

/**
 * @brief 向 vofa+ 发送两个浮点数（ASCII格式）
 *        vofa+ 接收格式：  value1,value2\n
 *        在 vofa+ 里选择 "ASCII" 协议，分隔符为逗号
 *
 * @param val1  第一个数据（如角度°）
 * @param val2  第二个数据（如力矩Nm）
 */
void USART_SendToVofa(float val1, float val2);

/**
 * @brief 向 vofa+ 发送三个浮点数
 */
void USART_SendToVofa3(float val1, float val2, float val3);

#endif /* BSP_USART_H */
