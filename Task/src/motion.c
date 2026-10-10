/**
 * motion.c
 * 单轴伺服运动控制：启动/停止/传感器。
 *  启动  = PB3（常闭，低=按下，消抖后上升沿）或 传感器1 来料信号（上升沿）
 *  停止  = PB4（常闭，低=按下，消抖）或 传感器2 到位信号（立即停机）
 *  传感器1 = 板载 PNP 输入1（PD0 → reg24）：检测到来料 → 启动并锁存运行
 *  传感器2 = 板载 PNP 输入2（PD1 → reg25）：检测到物料到位 → 停机
 *  运行逻辑（边沿触发·锁存运行）：
 *    传感器1/启动按钮触发 → 运行；物料离开传感器1后继续转；
 *    直到传感器2触发或停止按钮按下才停；传感器2未释放（物料未取走）前不允许再次启动。
 * 通过 bsp_servo 驱动伺服（速度模式）。
 */
#include "motion.h"
#include "bsp_servo.h"
#include "bsp_log.h"
#include "gpio.h"
#include "bsp_modbus_master.h"
#include "bsp_modbus.h"
#include <stdint.h>

/* ---- 用户配置 ---- */
#define MOTION_SERVO_RPM      500     /* 启动后伺服转速 (rpm) */
#define MOTION_START_PORT    GPIOB
#define MOTION_START_PIN     GPIO_PIN_3   /* PB3 启动 */
#define MOTION_STOP_PORT     GPIOB
#define MOTION_STOP_PIN      GPIO_PIN_4   /* PB4 停止 */
/* 传感器为 PNP 型,接板载 PNP 输入端子:检测到物体输出 24V → 光耦导通 → reg=1
 * 接线:棕→+24V,蓝→0V(必须与板子共地),黑→输入端子,无需上拉电阻 */
#define MOTION_SENSOR1_REG    SERVO_MB_REG_PNP_IN0 + 1u  /* 传感器1 = 输入1(PD0 → reg24):来料 → 启动 */
#define MOTION_SENSOR2_REG    SERVO_MB_REG_PNP_IN0 + 2u  /* 传感器2 = 输入2(PD1 → reg25):到位 → 停止 */

static uint8_t running = 0;
static uint8_t prev_start = 0;
static uint8_t prev_s1 = 1;      /* 传感器1边沿记忆,上电置1:上电时物料压在传感器1上不自动启动,需来料位先清空再触发 */
static uint8_t sensor_latched = 0;
static uint8_t start_cnt = 0;   /* 启动消抖计数 */
static uint8_t stop_cnt = 0;    /* 停止消抖计数 */
static uint8_t s1_cnt = 0;      /* 传感器1消抖计数 */

static uint8_t start_btn(void)
{
    /* 常闭按钮:松开=闭合(高),按下=断开(低) → 低电平=按下 */
    return (HAL_GPIO_ReadPin(MOTION_START_PORT, MOTION_START_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

static uint8_t stop_btn(void)
{
    /* 常闭按钮:低电平=按下(停止);断线/松开=高=不停止 */
    return (HAL_GPIO_ReadPin(MOTION_STOP_PORT, MOTION_STOP_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
}

static uint8_t sensor1_active(void)
{
    /* PNP 传感器接板上 PNP 输入1端子(PD0):检测到物体输出24V→光耦亮→reg24=1→触发启动 */
    return (mb_holding_regs[MOTION_SENSOR1_REG] == 1u) ? 1u : 0u;
}

static uint8_t sensor2_active(void)
{
    /* PNP 传感器接板上 PNP 输入2端子(PD1):检测到物体输出24V→光耦亮→reg25=1→触发停机 */
    return (mb_holding_regs[MOTION_SENSOR2_REG] == 1u) ? 1u : 0u;
}

void Motion_Init(void)
{
    running = 0;
    prev_start = 0;
    Servo_SetMode(SERVO_MODE_SPEED);
    Servo_SetSpeed((int16_t)mb_holding_regs[SERVO_MB_REG_SPEED]);
    Servo_SetAccelDecel(200u, 200u);
    Servo_Disable();
    LOG("Motion init (speed mode)");
}

uint8_t Motion_IsRunning(void)
{
    return running;
}

uint8_t Motion_SensorTriggered(void)
{
    return sensor_latched;
}

void Motion_Abort(void)
{
    running = 0;
    LOG("Motion ABORT (estop)");
}

void Motion_Update(void)
{
    uint8_t start  = start_btn();
    uint8_t stop   = stop_btn();
    uint8_t s1     = sensor1_active();   /* 传感器1:来料检测(启动) */
    uint8_t s2     = sensor2_active();   /* 传感器2:到位检测(停止) */

    /* 软件消抖:启动/传感器1连续 3 次采样才认定,停止连续 2 次;传感器2立即响应不消抖 */
    start_cnt = start ? (start_cnt < 3u ? (uint8_t)(start_cnt + 1u) : start_cnt) : 0u;
    stop_cnt  = stop  ? (stop_cnt  < 2u ? (uint8_t)(stop_cnt  + 1u) : stop_cnt ) : 0u;
    s1_cnt    = s1    ? (s1_cnt    < 3u ? (uint8_t)(s1_cnt    + 1u) : s1_cnt    ) : 0u;
    uint8_t start_db = (start_cnt >= 3u);
    uint8_t stop_db  = (stop_cnt  >= 2u);
    uint8_t s1_db    = (s1_cnt    >= 3u);
    uint8_t start_edge = start_db && !prev_start;   /* 启动按钮上升沿(消抖后) */
    uint8_t s1_edge    = s1_db    && !prev_s1;      /* 传感器1上升沿(消抖后):来料触发启动 */

    /* 按钮状态映射到 Modbus(每周期覆盖,上位机只读) */
    mb_holding_regs[SERVO_MB_REG_START_BTN] = start_db;   /* 启动按钮(消抖后) 1=按下 */
    mb_holding_regs[SERVO_MB_REG_STOP_BTN]  = stop_db;    /* 停止按钮(消抖后) 1=按下 */
    mb_holding_regs[SERVO_MB_REG_SENSOR1]   = s1 ? 1u : 0u;  /* 传感器1(原始电平) 1=有效 */
    mb_holding_regs[SERVO_MB_REG_SENSOR2]   = s2 ? 1u : 0u;  /* 传感器2(原始电平) 1=有效 */

    if (running) {
        /* 停止：停止按钮(消抖后) 或 传感器2(立即响应,不消抖) */
        if (stop_db || s2) {
            mb_holding_regs[SERVO_MB_REG_RUN] = 0;   /* 停止命令交由 MbPoll 统一下发(含重试) */
            running = 0;
            if (s2) { sensor_latched = 1; LOG("Servo STOP (sensor2)"); }
            else LOG("Servo STOP (button)");
        }
    } else {
        /* 启动：启动按钮上升沿 或 传感器1上升沿(来料)；
         * 前提：传感器2未有效(物料已取走/未到位) 且 停止按钮未按下 */
        if ((start_edge || s1_edge) && !s2 && !stop_db) {
            mb_holding_regs[SERVO_MB_REG_RUN] = 1;   /* 运行命令交由 MbPoll 统一下发(含重试) */
            running = 1;
            sensor_latched = 0;
            LOG("Servo START rpm=%d", (int)mb_holding_regs[SERVO_MB_REG_SPEED]);
        }
    }

    prev_start = start_db;
    prev_s1    = s1_db;
}
