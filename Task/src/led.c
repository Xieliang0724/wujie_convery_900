#include "led.h"
#include "io_logic.h"
#include "motion.h"
#include "bsp_servo.h"
#include "bsp_modbus.h"

/* ER.941(0x0941) 是"参数待断电生效"警告,不是故障,不计入故障闪烁 */
#define LED_ALARM_WARN_ER941   0x0941u

void LED_Init(void)
{
    g_io.output.led.working = 0;
    g_io.output.led.network = 0;
    g_io.output.led.system  = 0;
}

/* working 灯(PE5,高=亮)= 伺服通信/故障状态灯,三态:
 *   伺服真故障(reg2>0 且非 ER.941 警告) -> 500ms 慢闪
 *   伺服通信断开(reg0 bit3,连续485失败>=3) -> 100ms 快闪
 *   正常(通信通+无真故障)                 -> 常亮
 * 优先级: 伺服故障 > 通信断开 > 正常常亮 */
void LED_Update(void)
{
    static uint32_t last_tick = 0;
    static uint8_t  blink = 0;

    uint16_t dev  = mb_holding_regs[SERVO_MB_REG_DEV_STATE];
    uint16_t alarm= mb_holding_regs[SERVO_MB_REG_ALARM];
    uint8_t  comm_fault   = (dev & 0x08u) ? 1u : 0u;               /* reg0 bit3 通信断开 */
    uint8_t  servo_fault  = (alarm != 0u && alarm != LED_ALARM_WARN_ER941) ? 1u : 0u;  /* 真故障 */

    uint32_t period;
    if (servo_fault) {
        period = 500u;                    /* 伺服真故障:慢闪 */
    } else if (comm_fault) {
        period = 100u;                    /* 通信断开:快闪 */
    } else {
        g_io.output.led.working = 1u;     /* 正常:常亮 */
        return;
    }

    /* 闪烁:按当前 period 翻转 */
    if ((HAL_GetTick() - last_tick) >= period) {
        last_tick = HAL_GetTick();
        blink ^= 1u;
        g_io.output.led.working = blink;
    }

    /* system 灯：运行时亮（PE2 镜像 / PE4 反相） */
    g_io.output.led.system = Motion_IsRunning() ? 1u : 0u;
}
