#include "alarm.h"
#include <string.h>

Alarm_t g_alarm;

void Alarm_Init(void)
{
    memset(&g_alarm, 0, sizeof(g_alarm));
}

/* flash_fault 由 bsp_W25Q128 自检维护；此处无周期逻辑 */
void Alarm_Update(void)
{
}
