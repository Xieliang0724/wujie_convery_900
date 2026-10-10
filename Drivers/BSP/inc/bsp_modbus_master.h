#ifndef __BSP_MODBUS_MASTER_H__
#define __BSP_MODBUS_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

/* ================================================================
 * Modbus RTU 主站（用于 USART6 / RS-485，驱动伺服等从站）
 * 依赖：CRC16_Modbus()（Tools），USSART6 已初始化（115200 8N1）
 * 注意：RS-485 收发由硬件自动换向（Q5），无需软件控制方向
 * ================================================================ */

/* 主站结果码 */
typedef enum {
    MB_M_OK            =  0,   /* 成功 */
    MB_M_ERR_TIMEOUT   = -1,   /* 等待应答超时 */
    MB_M_ERR_CRC       = -2,   /* 响应 CRC 错误 */
    MB_M_ERR_EXCEPTION = -3,   /* 从站返回异常码 */
    MB_M_ERR_TX        = -4,   /* 发送失败 */
    MB_M_ERR_RX        = -5,   /* 接收失败 */
    MB_M_ERR_PARAM     = -6    /* 参数非法 */
} MB_Master_Status_t;

void Modbus_Master_Init(void);
void Modbus_Master_BurstTest(void);   /* 调试:复位后连续发送10秒(量DE/A-B用) */

/* 读保持寄存器 FC03 */
MB_Master_Status_t Modbus_Master_ReadHoldingRegs(uint8_t slave, uint16_t start,
                                                 uint16_t count,
                                                 uint16_t *data,
                                                 uint32_t timeout_ms);

/* 写单个寄存器 FC06 */
MB_Master_Status_t Modbus_Master_WriteSingleReg(uint8_t slave, uint16_t reg,
                                                uint16_t val, uint32_t timeout_ms);

/* 写多个寄存器 FC16 */
MB_Master_Status_t Modbus_Master_WriteMultipleRegs(uint8_t slave, uint16_t start,
                                                   uint16_t count,
                                                   const uint16_t *data,
                                                   uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MODBUS_MASTER_H__ */
