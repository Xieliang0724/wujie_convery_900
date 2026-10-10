#ifndef __BSP_LOG_H__
#define __BSP_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

void Log_Init(void);

/* 日志宏：输出到 USART3（以太网串口2）并回车换行 */
#define LOG(fmt, ...)  printf(fmt "\r\n", ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LOG_H__ */
