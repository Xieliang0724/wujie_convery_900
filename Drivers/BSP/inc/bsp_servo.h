#ifndef __BSP_SERVO_H__
#define __BSP_SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_modbus_master.h"

/* ================================================================
 * 通信前提：伺服需先设为 115200 8N1（用上位机调试软件）
 *   H0C_02 = 6  -> 115200
 *   H0C_03 = 3  -> 无校验, 1 停止位 (8N1)
 * 主站 USART6（PC6/PC7）已按 115200 8N1 配置。
 * ================================================================ */
/* ================================================================
 * 伺服驱动层（四横低压伺服，Modbus RTU 从站）
 * 底层通过 bsp_modbus_master（USART6 / RS-485）通信
 * 寄存器地址 = (hex 组号)*256 + (组内十进制编号)
 * ================================================================ */

#define SERVO_SLAVE_ADDR      1u      /* 伺服轴地址（H0C_00，默认 1） */

#define SERVO_MODE_SPEED      0u
#define SERVO_MODE_POSITION   1u

/* 伺服寄存器地址 */
#define SERVO_REG_MODE        0x0200u  /* H02_00 控制模式 */
#define SERVO_REG_ENABLE      0x0303u  /* H03_03 伺服使能/运行 */
#define SERVO_REG_SPEED_SRC   0x0602u  /* H06_02 速度指令来源(0=内部) */
#define SERVO_REG_SPEED       0x0603u  /* H06_03 速度指令 rpm (Int16) */
#define SERVO_REG_ACCEL       0x0605u  /* H06_05 加速时间 ms */
#define SERVO_REG_DECEL       0x0606u  /* H06_06 减速时间 ms */
#define SERVO_REG_SPD_ARRIVE  0x0612u  /* H06_18 速度到达阈值 */
#define SERVO_REG_CUR_SPEED   0x0B00u  /* H0B_00 实际转速 rpm (只读) */
#define SERVO_REG_ALARM       0x0B22u  /* H0B_34 当前故障码 (只读) */
#define SERVO_REG_ESTOP       0x0D05u  /* H0D_05 紧急停机 */
#define SERVO_REG_FAULT_RST   0x0D01u  /* H0D_01 故障复位 */

/* ================================================================
 * 伺服 Modbus 从站寄存器表（上位机 ↔ 伺服参数 / 状态）
 * 集中编排: 0-9 只读状态 / 10-19 操作 / 20-30 输入 / 31-49 DI+诊断
 * ================================================================ */

/* --- 只读状态区 0-9 --- */
#define SERVO_MB_REG_DEV_STATE      0   /* bit0=运行,bit1=停止,bit2=故障,bit3=485通信故障 (只读) */
#define SERVO_MB_REG_SPEED_FB       1   /* 实际转速 rpm(带符号,H0B_00) (只读) */
#define SERVO_MB_REG_ALARM          2   /* 伺服故障/警告码 (只读,H0B_34) */
#define SERVO_MB_REG_FLASH_FAULT    3   /* Flash 故障 (只读) */
#define SERVO_MB_REG_SENSOR_FLAG    4   /* 传感器2触发停机标志 0/1 (只读) */
#define SERVO_MB_REG_SPEED_ABS      5   /* 绝对转速 rpm(无符号) (只读) */
#define SERVO_MB_REG_DIR_FB         6   /* 旋转方向回读 (只读,H02_02) */
#define SERVO_MB_REG_SRC_FB         7   /* 速度源选择回读 (只读,H06_02) */

/* --- 操作区 10-19 (读写) --- */
#define SERVO_MB_REG_SPEED          10  /* 目标速度 rpm (1:1, 双写H06_03+H06_04) */
#define SERVO_MB_REG_ACCEL          11  /* 加速时间 ms */
#define SERVO_MB_REG_DECEL          12  /* 减速时间 ms */
#define SERVO_MB_REG_RUN            13  /* 运行命令 0=停/1=转 */
#define SERVO_MB_REG_MODE           14  /* 控制模式 0=速度/1=位置 */
#define SERVO_MB_REG_ESTOP          15  /* 紧急停机 0=正常/1=急停 */
#define SERVO_MB_REG_FAULT_RST      16  /* 故障复位 0/1 */
#define SERVO_MB_REG_DIR            17  /* 旋转方向 0=CCW正/1=CW正 (H02_02,停止+断电生效) */
#define SERVO_MB_REG_SRC_SEL        18  /* 速度源选择 0=A路/1=B路 (H06_02) */

/* --- 输入区 20-30 --- */
#define SERVO_MB_REG_START_BTN      20  /* 启动按钮(消抖后) 1=按下 (只读) */
#define SERVO_MB_REG_STOP_BTN       21  /* 停止按钮(消抖后) 1=按下 (只读) */
#define SERVO_MB_REG_SENSOR1        22  /* 传感器1原始电平 1=有效 (只读,PD0/输入1,来料→启动) */
#define SERVO_MB_REG_PNP_IN0        23  /* PNP 输入0..7(只读, 23+i) */

/* --- DI 配置 / 诊断回读区 31-49 --- */
#define SERVO_MB_REG_VDI1_F         31  /* VDI1功能读回(H17_00) */
#define SERVO_MB_REG_VDI2_F         32  /* VDI2功能读回(H17_02) */
#define SERVO_MB_REG_VDI3_F         33  /* VDI3功能读回(H17_04) */
#define SERVO_MB_REG_DI2_LOGIC      34  /* DI2逻辑读回(H03_05) */
#define SERVO_MB_REG_DI3_LOGIC      35  /* DI3逻辑读回(H03_07) */
#define SERVO_MB_REG_DI5_LOGIC      36  /* DI5逻辑读回(H03_11) */
#define SERVO_MB_REG_SPEED_CMD_FB   37  /* H06_03 速度指令回读 */
#define SERVO_MB_REG_JOG_SPEED_FB   38  /* H06_04 JOG速度回读 */
#define SERVO_MB_REG_SPD_SRC_FB     39  /* H06_00 主速度A来源回读 */
#define SERVO_MB_REG_TORQUE_FB      40  /* H0B_02 实时转矩(0.1%) */
#define SERVO_MB_REG_DI1_FUNC       41  /* DI1功能设定(1=伺服使能,H03_02,读写) */
#define SERVO_MB_REG_DI2_FUNC       42  /* DI2功能设定(2=报警复位,H03_04,读写) */
#define SERVO_MB_REG_DI3_FUNC       43  /* DI3功能设定(34=急停,H03_06,读写) */
#define SERVO_MB_REG_DI4_FUNC       44  /* DI4功能设定(0=禁正点动,H03_08,读写) */
#define SERVO_MB_REG_DI5_FUNC       45  /* DI5功能设定(0=禁反点动,H03_10,读写) */
#define SERVO_MB_REG_SPEED_A_SRC    46  /* 主速度A来源设定(H06_00) */
#define SERVO_MB_REG_DI4_FB         47  /* DI4功能读回(H03_08) */
#define SERVO_MB_REG_DI5_FB         48  /* DI5功能读回(H03_10) */
#define SERVO_MB_REG_SENSOR2        49  /* 传感器2原始电平 1=有效 (只读,PD1/输入2,到位→停止) */

void Servo_Init(void);
MB_Master_Status_t Servo_SetMode(uint16_t mode);
MB_Master_Status_t Servo_SetSpeed(int16_t rpm);
MB_Master_Status_t Servo_SetAccelDecel(uint16_t accel_ms, uint16_t decel_ms);
MB_Master_Status_t Servo_Enable(void);
MB_Master_Status_t Servo_Disable(void);
MB_Master_Status_t Servo_EStop(uint8_t on);   /* on=1 急停(位置锁定); on=0 解除(先失能) */
MB_Master_Status_t Servo_FaultReset(void);
MB_Master_Status_t Servo_ReadSpeed(int16_t *rpm);
MB_Master_Status_t Servo_ReadAlarm(uint16_t *alarm);

/* 主循环中调用：把从站参数寄存器转发到伺服，并回读状态/报警 */
void Servo_MbPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SERVO_H__ */
