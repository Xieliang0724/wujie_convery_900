#include <stdint.h>
#ifndef __MOTION_H__
#define __MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

void Motion_Init(void);
void Motion_Update(void);      /* 主循环中调用 */
uint8_t Motion_IsRunning(void);/* 当前是否在运行 */
uint8_t Motion_SensorTriggered(void);/* 传感器触发停机标志 */
void Motion_Abort(void);       /* 急停联动:清本地运行标志(不碰伺服) */

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H__ */
