/**
 * bsp_log.c
 * 将 newlib 的 printf 重定向到 USART3（以太网模组串口2 日志输出）。
 * 若此刻 USART3 正在转发 ESP32 日志（DMA 占用），本次 printf 会被丢弃，避免冲突。
 */
#include "bsp_log.h"
#include "usart.h"
#include <stddef.h>

int _write(int file, char *ptr, int len)
{
    (void)file;
    if (len > 0) {
        if (HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, 100u) != HAL_OK) {
            /* 发送被占用时静默丢弃 */
        }
    }
    return len;
}

void Log_Init(void)
{
    /* huart3 已由 MX_USART3_UART_Init() 配置 */
}
