/**
 * motion.c
 * 单轴伺服运动控制：启动/停止/传感器。
 *  启动  = PB3（常闭，低=按下，消抖后上升沿）或 传感器1 来料信号（上升沿）
 *  停止  = PB4（常闭，低=按下，消抖）或 传感器2 到位信号（立即停机）
 *  传感器1 = 板载 输入1（PD0 → reg24）：检测到来料（遮挡=0V，reg=0） → 启动并锁存运行
 *  传感器2 = 板载 输入2（PD1 → reg25）：检测到物料到位（遮挡=0V，reg=0） → 停机
 *  传感器信号（镜面反射式）：空闲输出 24V(有信号) → reg=1；被物料遮挡输出 0V → reg=0
 *    输入电路「端子→R10→PC817」24V 导通：故「检测到(遮挡)」= reg==0
 *  运行逻辑（电平自动 + 武装）：
 *    上电默认「未武装」：即使来料位有物料也不自动启动；
 *    需人工按一次【启动按钮】→ 武装并启动；
 *    已武装后：来料位有料(电平)且传感器2无料 → 自动启动；
 *    传感器2停机不解除武装 —— 物料取走后若来料位已有料,自动接续转动；
 *    【停止按钮】与急停(Estop)解除武装,需重新按启动按钮。
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
/* 传感器为镜面反射式,接板载输入端子:空闲输出 24V(有信号)→输入有效(reg=1);
 * 被物料遮挡输出 0V → 输入无效(reg=0)。故「检测到(遮挡)」= reg==0。
 * 接线:棕→+24V,蓝→0V(必须与板子共地),黑→输入端子,无需上拉电阻 */
#define MOTION_SENSOR1_REG    SERVO_MB_REG_PNP_IN0 + 1u  /* 传感器1 = 输入1(PD0 → reg24):来料 → 启动 */
#define MOTION_SENSOR2_REG    SERVO_MB_REG_PNP_IN0 + 2u  /* 传感器2 = 输入2(PD1 → reg25):到位 → 停止 */

static uint8_t running = 0;
static uint8_t prev_start = 0;
static uint8_t sensor_latched = 0;
static uint8_t armed = 0;        /* 自动模式武装标志:需人工按一次启动按钮后才允许传感器1自动启动 */
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
    /* 镜面反射传感器:空闲输出 24V(有信号)→reg=1;被物料遮挡输出 0V→reg=0。
     * 故「检测到(遮挡)」= reg==0。传感器1 接输入1端子(PD0→reg24) */
    return (mb_holding_regs[MOTION_SENSOR1_REG] == 0u) ? 1u : 0u;   /* reg==0 = 检测到/有效 */
}

static uint8_t sensor2_active(void)
{
    /* 同传感器1:空闲24V→reg=1, 遮挡0V→reg=0。检测到=reg==0。接输入2端子(PD1→reg25) */
    return (mb_holding_regs[MOTION_SENSOR2_REG] == 0u) ? 1u : 0u;   /* reg==0 = 检测到/有效 */
}

void Motion_Init(void)
{
    running = 0;
    prev_start = 0;
    armed = 0;   /* 上电默认未武装:须先按启动按钮,之后才允许传感器1自动启动 */
    Servo_SetMode(SERVO_MODE_SPEED);
    Servo_SetSpeed((int16_t)mb_holding_regs[SERVO_MB_REG_SPEED]);
    Servo_SetAccelDecel(200u, 200u);
    Servo_Disable();
    LOG("Motion init (speed mode, not armed)");
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
    armed = 0;   /* 急停后解除武装:须重新按启动按钮才恢复自动模式 */
    mb_holding_regs[SERVO_MB_REG_RUN] = 0;   /* 清运行命令:防止急停解除后残留命令被远程同步误认为启动 */
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

    /* 按钮状态映射到 Modbus(每周期覆盖,上位机只读) */
    mb_holding_regs[SERVO_MB_REG_START_BTN] = start_db;   /* 启动按钮(消抖后) 1=按下 */
    mb_holding_regs[SERVO_MB_REG_STOP_BTN]  = stop_db;    /* 停止按钮(消抖后) 1=按下 */
    mb_holding_regs[SERVO_MB_REG_SENSOR1]   = s1 ? 1u : 0u;  /* 传感器1(原始电平) 1=有效 */
    mb_holding_regs[SERVO_MB_REG_SENSOR2]   = s2 ? 1u : 0u;  /* 传感器2(原始电平) 1=有效 */

    /* 远程命令同步:上位机经 Modbus 直写 reg13 时状态机跟随(远程启动=武装/远程停止=解除武装),
     * 保证传感器联锁(传感器2立即停、传感器1电平启动、停止优先)对远程操作同样有效 */
    {
        uint8_t run_cmd = mb_holding_regs[SERVO_MB_REG_RUN] ? 1u : 0u;
        if (run_cmd != running) {
            running = run_cmd;
            armed = run_cmd;
            LOG(run_cmd ? "Remote START" : "Remote STOP");
        }
    }

    /* 停止按钮按下 → 解除武装(退出自动模式):防止手动停止后来料位有料又自转 */
    if (stop_db) armed = 0;
    /* 按下启动按钮 → 武装自动模式 */
    if (start_edge) armed = 1;

    if (running) {
        /* 停止：停止按钮(消抖后) 或 传感器2(立即响应,不消抖) */
        if (stop_db || s2) {
            mb_holding_regs[SERVO_MB_REG_RUN] = 0;   /* 停止命令交由 MbPoll 统一下发(含重试) */
            running = 0;
            if (s2) { sensor_latched = 1; LOG("Servo STOP (sensor2)"); }
            else LOG("Servo STOP (button)");
        }
    } else {
        /* 启动：
         *   手动  = 启动按钮上升沿(始终有效,且会武装自动模式)
         *   自动  = 来料位有料(电平,s1_db) 且 已武装(armed)
         * 共同前提：传感器2未有效(物料已取走/未到位) 且 停止按钮未按下
         * 说明：自动启动用电平而非边沿——传感器2停机期间放到传感器1的料,
         *       在传感器2物料取走后能自动接续转动 */
        uint8_t start_manual = start_edge;
        uint8_t start_auto   = s1_db && armed;
        if ((start_manual || start_auto) && !s2 && !stop_db) {
            mb_holding_regs[SERVO_MB_REG_RUN] = 1;   /* 运行命令交由 MbPoll 统一下发(含重试) */
            running = 1;
            sensor_latched = 0;
            LOG("Servo START rpm=%d", (int)mb_holding_regs[SERVO_MB_REG_SPEED]);
        }
    }

    prev_start = start_db;
}
