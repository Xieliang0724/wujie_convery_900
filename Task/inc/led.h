#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


void LED_Init(void);    /* 初始化，全部熄灭 */
void LED_Update(void);  /* 主循环中调用，更新 LED 状态 */

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
