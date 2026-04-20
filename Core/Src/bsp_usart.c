#include "bsp_usart.h"
#include <stdio.h>
#include <string.h>

/* vofa+ 使用的 USART 句柄，与 CubeMX 配置一致 */
#define VOFA_HUART  (&huart1)

void USART_SendToVofa(float val1, float val2)
{
    char buf[64];
    /* vofa+ ASCII 协议：用逗号分隔，换行结束 */
    int len = snprintf(buf, sizeof(buf), "%.4f,%.4f\n", val1, val2);
    HAL_UART_Transmit(VOFA_HUART, (uint8_t *)buf, len, 10);
}

void USART_SendToVofa3(float val1, float val2, float val3)
{
    char buf[80];
    int len = snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f\n", val1, val2, val3);
    HAL_UART_Transmit(VOFA_HUART, (uint8_t *)buf, len, 10);
}
