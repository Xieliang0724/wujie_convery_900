#ifndef __ALARM_H__
#define __ALARM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    uint8_t flash_fault;        /* Flash 自检失败（W25Q128） */
} AlarmFlags_t;

typedef struct {
    AlarmFlags_t flags;
} Alarm_t;

extern Alarm_t g_alarm;

void Alarm_Init(void);
void Alarm_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __ALARM_H__ */
