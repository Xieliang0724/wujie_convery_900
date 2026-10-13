/**
 * bsp_servo.c
 * 四横低压伺服驱动：通过 Modbus RTU 主站（USART6/RS-485）读写伺服寄存器。
 * 并把“上位机 Modbus 从站参数寄存器”转发到伺服，回读状态/报警。
 */
#include "bsp_servo.h"
#include "bsp_modbus.h"
#include "bsp_log.h"
#include "motion.h"
#include "alarm.h"
#include <stdint.h>

static int16_t  last_speed  = -32768;
static uint16_t last_accel  = 0xFFFF;
static uint16_t last_decel  = 0xFFFF;
static uint16_t last_run    = 0xFF;
static uint16_t last_mode   = 0xFF;
static uint16_t last_estop  = 0xFF;
static uint16_t last_fault  = 0xFF;
static uint8_t  comm_err_cnt = 0;   /* 485 连续通信失败计数,>=3 置 reg0 bit3 */
static uint16_t last_alarm_cache = 0;  /* 最近一次回读的故障码(状态字用) */
static uint32_t last_read_tick = 0;    /* 回读限频计时 */
static uint16_t last_jog = 0xFFFF;     /* H06_04 JOG速度影子值 */
static uint16_t last_difunc = 0xFFFF;  /* H03_02 DI1功能影子值 */
static uint16_t last_di2f = 0xFFFF;    /* H03_04 DI2功能影子值 */
static uint16_t last_di3f = 0xFFFF;    /* H03_06 DI3功能影子值 */
static uint16_t last_di4f = 0xFFFF;    /* H03_08 DI4功能影子值 */
static uint16_t last_di5f = 0xFFFF;    /* H03_10 DI5功能影子值 */
static uint16_t last_asrc = 0xFFFF;    /* H06_00 主速度A来源影子值 */
static uint32_t last_run_tick = 0;     /* 最近一次使能命令下发时间 */
static int16_t last_speed_cmd = -1;    /* 最近写入 H06_04 的标定值(漂移检测用) */
static uint16_t last_dir = 0xFFFF;     /* H02_02 旋转方向影子值(0=CCW正,1=CW正) */
static uint16_t last_srcsel = 0xFFFF;  /* H06_02 速度指令选择影子值(0=A路H06_03,1=B路) */
static uint16_t last_commtmo = 0xFFFF; /* H0C_30 通讯超时影子值(出厂1ms过苛,改3000ms) */

/* ================= 参数掉电保存(内部Flash 扇区11,速度/方向/加减速) ================= */
#define PARAM_MAGIC   0x574A5350u   /* "WJSP" */

typedef struct {
    uint32_t magic;
    int16_t  speed;
    uint16_t dir;
    uint16_t accel;
    uint16_t decel;
    uint16_t rsvd;
    uint32_t checksum;      /* 前6个word累加和 */
} ServoParam_t;

static uint8_t param_save_pending = 0;

static void Servo_ParamSave(void)
{
    ServoParam_t p;
    p.magic = PARAM_MAGIC;
    p.speed = (int16_t)mb_holding_regs[SERVO_MB_REG_SPEED];
    p.dir   = mb_holding_regs[SERVO_MB_REG_DIR];
    p.accel = mb_holding_regs[SERVO_MB_REG_ACCEL];
    p.decel = mb_holding_regs[SERVO_MB_REG_DECEL];
    p.rsvd  = 0;
    uint32_t sum = 0;
    uint32_t *w = (uint32_t *)&p;
    for (int i = 0; i < 4; i++) sum += w[i];   /* 仅校验数据区(magic/speed+dir/accel+decel/rsvd) */
    p.checksum = sum;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef ei;
    uint32_t err = 0;
    ei.TypeErase   = FLASH_TYPEERASE_SECTORS;
    ei.Banks       = FLASH_BANK_1;
    ei.Sector      = FLASH_SECTOR_11;
    ei.NbSectors   = 1u;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&ei, &err);
    for (int i = 0; i < 5; i++)   /* 结构体共20字节=5个word */
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x080E0000u + (uint32_t)i * 4u, w[i]);
    HAL_FLASH_Lock();
}

static uint8_t Servo_ParamLoad(void)
{
    const ServoParam_t *p = (const ServoParam_t *)0x080E0000u;
    uint32_t sum = 0;
    uint32_t *w = (uint32_t *)p;
    for (int i = 0; i < 4; i++) sum += w[i];
    if (p->magic == PARAM_MAGIC && p->checksum == sum) {
        mb_holding_regs[SERVO_MB_REG_SPEED]     = (uint16_t)p->speed;
        mb_holding_regs[SERVO_MB_REG_DIR]       = p->dir;
        mb_holding_regs[SERVO_MB_REG_ACCEL]     = p->accel;
        mb_holding_regs[SERVO_MB_REG_DECEL]     = p->decel;
        return 1u;
    }
    return 0u;
}

void Servo_Init(void)
{
    uint8_t loaded = Servo_ParamLoad();   /* 加载掉电保存参数(速度/方向/加减速) */

    /* 默认参数(仅在 Flash 中无有效数据时生效) */
    if (!loaded) {
        mb_holding_regs[SERVO_MB_REG_SPEED] = 500;   /* 目标速度 rpm */
        mb_holding_regs[SERVO_MB_REG_ACCEL] = 200;   /* 加速 ms */
        mb_holding_regs[SERVO_MB_REG_DECEL] = 200;   /* 减速 ms */
        mb_holding_regs[SERVO_MB_REG_DIR] = 0u;      /* 旋转方向 0=CCW为正 */
    }
    mb_holding_regs[SERVO_MB_REG_RUN]       = 0;
    mb_holding_regs[SERVO_MB_REG_MODE]      = SERVO_MODE_SPEED;
    mb_holding_regs[SERVO_MB_REG_ESTOP]     = 0;
    mb_holding_regs[SERVO_MB_REG_FAULT_RST] = 0;

    Servo_SetMode(SERVO_MODE_SPEED);
    Servo_SetAccelDecel(mb_holding_regs[SERVO_MB_REG_ACCEL], mb_holding_regs[SERVO_MB_REG_DECEL]);
    Servo_SetSpeed((int16_t)mb_holding_regs[SERVO_MB_REG_SPEED]);
    Servo_Disable();
    /* 方案要求参数:DI1功能=1(伺服使能) —— 由 MbPoll 转发写入驱动器 */
    mb_holding_regs[SERVO_MB_REG_DI1_FUNC] = 1u;
    /* 速度指令选择:reg18 0=A路(H06_03,支持负速度),1=B路(H06_04) —— 修改方式=使能断开,停止时写入 */
    mb_holding_regs[SERVO_MB_REG_SRC_SEL] = 0u;
    /* DI 功能:DI4 出厂=18(JOG+)、DI5 出厂=19(JOG-) 被误吸合会导致点动失控,均清 0;DI2/DI3 恢复出厂值 */
    mb_holding_regs[SERVO_MB_REG_DI2_FUNC] = 2u;    /* DI2 功能=2(出厂,报警复位) */
    mb_holding_regs[SERVO_MB_REG_DI3_FUNC] = 34u;   /* DI3 功能=34(出厂,急停) */
    mb_holding_regs[SERVO_MB_REG_DI4_FUNC] = 0u;    /* DI4 功能=0(禁用正向点动) */
    mb_holding_regs[SERVO_MB_REG_DI5_FUNC] = 0u;    /* DI5 功能=0(禁用反向点动) */
    mb_holding_regs[SERVO_MB_REG_SPEED_A_SRC] = 0u; /* 主速度A来源=0(H06_03给定) */
    /* 真实下发 DI 功能/方向/速度源 并同步影子值,避免首轮 MbPoll 事务风暴 */
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0302u, mb_holding_regs[SERVO_MB_REG_DI1_FUNC], 200u) == MB_M_OK) last_difunc = mb_holding_regs[SERVO_MB_REG_DI1_FUNC];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0304u, mb_holding_regs[SERVO_MB_REG_DI2_FUNC], 200u) == MB_M_OK) last_di2f = mb_holding_regs[SERVO_MB_REG_DI2_FUNC];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0306u, mb_holding_regs[SERVO_MB_REG_DI3_FUNC], 200u) == MB_M_OK) last_di3f = mb_holding_regs[SERVO_MB_REG_DI3_FUNC];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0308u, mb_holding_regs[SERVO_MB_REG_DI4_FUNC], 200u) == MB_M_OK) last_di4f = mb_holding_regs[SERVO_MB_REG_DI4_FUNC];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x030Au, mb_holding_regs[SERVO_MB_REG_DI5_FUNC], 200u) == MB_M_OK) last_di5f = mb_holding_regs[SERVO_MB_REG_DI5_FUNC];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0202u, mb_holding_regs[SERVO_MB_REG_DIR], 200u) == MB_M_OK) last_dir = mb_holding_regs[SERVO_MB_REG_DIR];
    if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0602u, mb_holding_regs[SERVO_MB_REG_SRC_SEL], 200u) == MB_M_OK) last_srcsel = mb_holding_regs[SERVO_MB_REG_SRC_SEL];

    /* 同步影子值:初始化已直接下发,避免首轮 MbPoll 重发全部参数造成事务风暴 */
    last_speed = (int16_t)mb_holding_regs[SERVO_MB_REG_SPEED];
    last_accel = mb_holding_regs[SERVO_MB_REG_ACCEL];
    last_decel = mb_holding_regs[SERVO_MB_REG_DECEL];
    last_mode  = mb_holding_regs[SERVO_MB_REG_MODE];
    last_estop = 0;   /* 上电默认急停已释放 */
    last_fault = 0;   /* 上电默认无故障 */
    last_run   = 0;   /* 已执行 Disable */
}

MB_Master_Status_t Servo_SetMode(uint16_t mode)
{
    if (mode > SERVO_MODE_POSITION) return MB_M_ERR_PARAM;
    return Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_MODE, mode, 100u);
}

MB_Master_Status_t Servo_SetSpeed(int16_t rpm)
{
    /* 标定:0x0604 为有效速度指令(0.1%单位,2.56rpm/字),实测写 300 转 765(×2.55)。
     * reg100 用户值(rpm) → 0x0604 = rpm×100/256;负值=反转,支持运行中切换。
     * 0x0603 同步写真实 rpm(若驱动器映射与手册一致时该值生效,两边一致避免跳变)。 */
    int32_t jog_cmd = ((int32_t)rpm * 100 + 128) / 256;
    if (jog_cmd > 18000) jog_cmd = 18000;
    if (jog_cmd < -18000) jog_cmd = -18000;
    MB_Master_Status_t st1 = Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0603u, (uint16_t)rpm, 100u);
    MB_Master_Status_t st2 = Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0604u, (uint16_t)jog_cmd, 100u);
    if (st1 == MB_M_OK && st2 == MB_M_OK) {
        last_speed_cmd = (int16_t)jog_cmd;
        return MB_M_OK;
    }
    return MB_M_ERR_TX;
}

MB_Master_Status_t Servo_SetAccelDecel(uint16_t accel_ms, uint16_t decel_ms)
{
    uint16_t data[2] = { accel_ms, decel_ms };
    return Modbus_Master_WriteMultipleRegs(SERVO_SLAVE_ADDR, SERVO_REG_ACCEL, 2u, data, 100u);
}

MB_Master_Status_t Servo_Enable(void)
{
    Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_SPEED_SRC, 0u, 100u);
    return Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_ENABLE, 1u, 100u);
}

MB_Master_Status_t Servo_Disable(void)
{
    return Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_ENABLE, 0u, 100u);
}

/* on=1 紧急停机(H0D_05=1,立即停止并位置锁定); on=0 解除(先写 H03_03=0 失能,防止带使能复跑) */
MB_Master_Status_t Servo_EStop(uint8_t on)
{
    if (!on) {
        (void)Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_ENABLE, 0u, 100u);
    }
    return Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_ESTOP, on ? 1u : 0u, 100u);
}

MB_Master_Status_t Servo_FaultReset(void)
{
    return Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, SERVO_REG_FAULT_RST, 1u, 100u);
}

MB_Master_Status_t Servo_ReadSpeed(int16_t *rpm)
{
    uint16_t v = 0;
    MB_Master_Status_t st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, SERVO_REG_CUR_SPEED, 1u, &v, 100u);
    if (st == MB_M_OK && rpm) *rpm = (int16_t)v;
    return st;
}

MB_Master_Status_t Servo_ReadAlarm(uint16_t *alarm)
{
    uint16_t v = 0;
    MB_Master_Status_t st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, SERVO_REG_ALARM, 1u, &v, 100u);
    if (st == MB_M_OK && alarm) *alarm = v;
    return st;
}

/* 主循环：把从站参数寄存器转发到伺服，并回读状态/报警 */
void Servo_MbPoll(void)
{
    uint16_t speed = mb_holding_regs[SERVO_MB_REG_SPEED];
    uint16_t accel = mb_holding_regs[SERVO_MB_REG_ACCEL];
    uint16_t decel = mb_holding_regs[SERVO_MB_REG_DECEL];
    uint16_t run   = mb_holding_regs[SERVO_MB_REG_RUN];
    uint16_t mode  = mb_holding_regs[SERVO_MB_REG_MODE];
    uint16_t estop = mb_holding_regs[SERVO_MB_REG_ESTOP];
    uint16_t fault = mb_holding_regs[SERVO_MB_REG_FAULT_RST];

    if ((int16_t)speed != last_speed) { if (Servo_SetSpeed((int16_t)speed) == MB_M_OK) { last_speed = (int16_t)speed; param_save_pending = 1; } }   /* 失败保留差值,下轮重试 */
    if (accel != last_accel || decel != last_decel) { if (Servo_SetAccelDecel(accel, decel) == MB_M_OK) { last_accel=accel; last_decel=decel; param_save_pending = 1; } }
    if (mode != last_mode)  { if (Servo_SetMode(mode) == MB_M_OK) last_mode = mode; }
    if (estop != last_estop) {
        if (Servo_EStop(estop ? 1u : 0u) == MB_M_OK) last_estop = estop;   /* 1=急停,0=解除(先失能) */
        if (estop) Motion_Abort();   /* 急停联动:清本地运行标志,解除后需重新按启动 */
        LOG("Servo ESTOP %s", estop ? "ON" : "RELEASE");
    }
    if (fault != last_fault) { if (Servo_FaultReset() == MB_M_OK) { mb_holding_regs[SERVO_MB_REG_FAULT_RST] = 0; last_fault = fault; } }
    if (run != last_run) {
        MB_Master_Status_t st = run ? Servo_Enable() : Servo_Disable();
        if (st == MB_M_OK) { last_run = run; last_run_tick = HAL_GetTick(); }
    } else if ((HAL_GetTick() - last_run_tick) > 5000u) {
        /* 周期重发使能命令:驱动器断电重启后参数回滚,5s 内自动恢复正确使能/停止状态 */
        MB_Master_Status_t st = run ? Servo_Enable() : Servo_Disable();
        if (st == MB_M_OK) last_run_tick = HAL_GetTick();
    }

    /* DI1功能=H03_02(1=伺服使能);H06_04 已由 Servo_SetSpeed 统一写入,此处不再单独写 */
    if (mb_holding_regs[SERVO_MB_REG_DI1_FUNC] != last_difunc) {
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0302u, mb_holding_regs[SERVO_MB_REG_DI1_FUNC], 200u) == MB_M_OK)
            last_difunc = mb_holding_regs[SERVO_MB_REG_DI1_FUNC];
    }
    if (mb_holding_regs[SERVO_MB_REG_DI2_FUNC] != last_di2f) {
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0304u, mb_holding_regs[SERVO_MB_REG_DI2_FUNC], 200u) == MB_M_OK)
            last_di2f = mb_holding_regs[SERVO_MB_REG_DI2_FUNC];
    }
    if (mb_holding_regs[SERVO_MB_REG_DI3_FUNC] != last_di3f) {
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0306u, mb_holding_regs[SERVO_MB_REG_DI3_FUNC], 200u) == MB_M_OK)
            last_di3f = mb_holding_regs[SERVO_MB_REG_DI3_FUNC];
    }
    if (mb_holding_regs[SERVO_MB_REG_DI4_FUNC] != last_di4f) {
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0308u, mb_holding_regs[SERVO_MB_REG_DI4_FUNC], 200u) == MB_M_OK)
            last_di4f = mb_holding_regs[SERVO_MB_REG_DI4_FUNC];
    }
    if (mb_holding_regs[SERVO_MB_REG_DI5_FUNC] != last_di5f) {
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x030Au, mb_holding_regs[SERVO_MB_REG_DI5_FUNC], 200u) == MB_M_OK)
            last_di5f = mb_holding_regs[SERVO_MB_REG_DI5_FUNC];
    }
    if (mb_holding_regs[SERVO_MB_REG_DIR] != last_dir) {
        /* H02_02 修改方式=使能断开:停止时才写得进;生效方式=断电重启 */
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0202u, mb_holding_regs[SERVO_MB_REG_DIR], 200u) == MB_M_OK) {
            last_dir = mb_holding_regs[SERVO_MB_REG_DIR];
            param_save_pending = 1;
        }
    }
    if (mb_holding_regs[SERVO_MB_REG_SRC_SEL] != last_srcsel) {
        /* H06_02 修改方式=使能断开:停止时写入才生效;选 A 路后 H06_03(带符号)成为速度命令 */
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0602u, mb_holding_regs[SERVO_MB_REG_SRC_SEL], 200u) == MB_M_OK)
            last_srcsel = mb_holding_regs[SERVO_MB_REG_SRC_SEL];
    }
    if (last_commtmo == 0xFFFF) {
        /* H0C_30 出厂=1ms:轮询间隔数百 ms,驱动器持续报通讯超时故障(2369)并锁参数写入。
         * 设 3000ms:正常轮询无碍,Flash 保存(~1-2s 暂停)期间也不误报;随 H0C_13 存入 EEPROM */
        if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0C1Eu, 3000u, 200u) == MB_M_OK)
            last_commtmo = 3000u;
    }

    /* 一次性:参数全部下发完成后写 H0C_13=1,把通讯参数(DI功能/模式等)保存进 EEPROM,
     * 否则断电重启后参数滚回出厂值,点动功能会重新抢夺控制权 */
    static uint8_t save_pending = 1;
    static uint16_t save_wait = 0;
    if (save_pending) {
        /* 只在停止状态保存:确保 EEPROM 里使能位(H03_03)=0,驱动器断电重启后不会自转 */
        if (mb_holding_regs[SERVO_MB_REG_RUN] == 0u) {
            if (++save_wait >= 30u) {
                if (Modbus_Master_WriteSingleReg(SERVO_SLAVE_ADDR, 0x0C0Du, 1u, 200u) == MB_M_OK)
                    save_pending = 0;
            }
        }
    }

    /* 状态字:每拍刷新(报警/通信状态用最近一次回读结果) */
    uint16_t dev = 0;
    dev |= Motion_IsRunning() ? 1u : 2u;
    if (last_alarm_cache) dev |= 4u;
    if (comm_err_cnt >= 3u) dev |= 8u;   /* bit3: 485 通信故障 */
    mb_holding_regs[SERVO_MB_REG_DEV_STATE] = dev;
    mb_holding_regs[SERVO_MB_REG_SENSOR_FLAG] = Motion_SensorTriggered() ? 1u : 0u;
    mb_holding_regs[SERVO_MB_REG_FLASH_FAULT] = g_alarm.flags.flash_fault ? 1u : 0u;

    /* 回读轮询:每拍只发一个读请求,响应间隔足够大,避免相邻应答串扰错位 */
    static uint8_t rr = 0;
    int16_t rpm = 0;  uint16_t alarm = 0;
    uint16_t cmd_speed = 0, jog_speed = 0, asrc = 0, torque = 0, di4f = 0, di5f = 0;
    uint16_t vdi1f = 0, vdi2f = 0, vdi3f = 0;
    MB_Master_Status_t st;
    switch (rr) {
    case 0:
        st = Servo_ReadSpeed(&rpm);
        if (st == MB_M_OK) {
            /* H0B_00 单位即 rpm(点动禁用后实测 1:1),直接存储 */
            mb_holding_regs[SERVO_MB_REG_SPEED_FB] = (uint16_t)rpm;   /* reg1 带符号原始值 */
            mb_holding_regs[SERVO_MB_REG_SPEED_ABS] = (uint16_t)((rpm < 0) ? -rpm : rpm);  /* reg5 绝对转速(rpm) */
        }
        break;
    case 1:
        st = Servo_ReadAlarm(&alarm);
        if (st == MB_M_OK) { mb_holding_regs[SERVO_MB_REG_ALARM] = alarm; last_alarm_cache = alarm; }
        break;
    case 2:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, SERVO_REG_SPEED, 1u, &cmd_speed, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_SPEED_CMD_FB] = cmd_speed;   /* H06_03 速度指令(rpm) */
        if (st == MB_M_OK && (int16_t)cmd_speed != last_speed) last_speed = 0x7FFF;  /* 驱动器值漂移→强制重写 */
        break;
    case 3:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0604u, 1u, &jog_speed, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_JOG_SPEED_FB] = jog_speed;   /* H06_04 JOG速度实际值 */
        if (st == MB_M_OK && (int16_t)jog_speed != last_speed_cmd) last_speed = 0x7FFF;
        break;
    case 4:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0600u, 1u, &asrc, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_SPD_SRC_FB] = asrc;        /* H06_00 主速度A来源 */
        if (st == MB_M_OK && asrc != last_asrc) last_asrc = 0xFFFF;
        break;
    case 15:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0B02u, 1u, &torque, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_TORQUE_FB] = torque;      /* H0B_02 实时转矩(0.1%) */
        break;
    default:
        st = MB_M_ERR_RX;   /* 兜底:rr 由 %16 约束,不应到达 */
        break;
    case 5:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0308u, 1u, &di4f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_DI4_FB] = di4f;         /* H03_08 DI4功能(18=点动) */
        if (st == MB_M_OK && di4f != last_di4f) last_di4f = 0xFFFF;
        break;
    case 6:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x030Au, 1u, &di5f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_DI5_FB] = di5f;         /* H03_10 DI5功能(19=反向点动) */
        if (st == MB_M_OK && di5f != last_di5f) last_di5f = 0xFFFF;
        break;
    case 7:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x1700u, 1u, &vdi1f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_VDI1_F] = vdi1f;        /* H17_00 VDI1功能(18/19=点动) */
        break;
    case 8:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x1702u, 1u, &vdi2f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_VDI2_F] = vdi2f;        /* H17_02 VDI2功能 */
        break;
    case 9:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x1704u, 1u, &vdi3f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_VDI3_F] = vdi3f;        /* H17_04 VDI3功能 */
        break;
    case 10:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0305u, 1u, &vdi1f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_DI2_LOGIC] = vdi1f;        /* H03_05 DI2逻辑(疑为VDI2电平) */
        break;
    case 11:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0307u, 1u, &vdi2f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_DI3_LOGIC] = vdi2f;        /* H03_07 DI3逻辑(疑为VDI3电平) */
        break;
    case 12:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x030Bu, 1u, &vdi3f, 200u);
        if (st == MB_M_OK) mb_holding_regs[SERVO_MB_REG_DI5_LOGIC] = vdi3f;        /* H03_11 DI5逻辑(疑为VDI5电平) */
        break;
    case 13:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0202u, 1u, &vdi3f, 200u);
        if (st == MB_M_OK) {
            mb_holding_regs[SERVO_MB_REG_DIR_FB] = vdi3f;                      /* H02_02 旋转方向回读 */
            if (vdi3f != last_dir) last_dir = 0xFFFF;          /* 漂移→强制重写 */
        }
        break;
    case 14:
        st = Modbus_Master_ReadHoldingRegs(SERVO_SLAVE_ADDR, 0x0602u, 1u, &vdi3f, 200u);
        if (st == MB_M_OK) {
            mb_holding_regs[SERVO_MB_REG_SRC_FB] = vdi3f;                      /* H06_02 速度指令选择回读 */
            if (vdi3f != last_srcsel) last_srcsel = 0xFFFF;
        }
        break;
    }
    rr = (uint8_t)((rr + 1u) % 16u);
    if (st == MB_M_OK) { comm_err_cnt = 0; }
    else { if (comm_err_cnt < 255u) comm_err_cnt++; }

    /* 参数变更掉电保存(Flash 扇区擦写约1-2s,期间轮询短暂暂停) */
    if (param_save_pending) {
        Servo_ParamSave();
        param_save_pending = 0;
    }
}
