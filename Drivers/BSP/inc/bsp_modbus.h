#ifndef __BSP_MODBUS_H__
#define __BSP_MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "tim.h"
#include "crc.h"

/* ================================================================
 * 用户配置区 — 根据实际需求修改以下宏
 * ================================================================ */
#define MODBUS_SLAVE_ADDR       1       /* 从站地址 */
#define MODBUS_REG_NUM          128      /* 保持寄存器数量 */ 
#define MB_BUF_SIZE             256     /* 单路串口收发缓冲区大小（字节）*/
#define MB_FRAME_TIMEOUT_MS     4       /* 帧间隔静默时间 (ms)，115200bps 下 3.5T≈0.3ms，取 4ms */
/* ================================================================ */

/* Modbus 功能码 */
#define MB_FC_READ_HOLDING_REGS     0x03
#define MB_FC_WRITE_SINGLE_REG      0x06
#define MB_FC_WRITE_MULTI_REGS      0x10

/* Modbus 异常码 */
#define MB_EX_ILLEGAL_FUNCTION      0x01
#define MB_EX_ILLEGAL_DATA_ADDR     0x02
#define MB_EX_ILLEGAL_DATA_VALUE    0x03

/* DMA 收发缓冲区（供中断文件引用）*/
extern uint8_t  mb_uart1_rx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart1_tx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart2_rx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart2_tx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart3_rx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart3_tx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart4_rx_buf[MB_BUF_SIZE];
extern uint8_t  mb_uart4_tx_buf[MB_BUF_SIZE];

/* 保持寄存器表（供应用层读写）*/
extern uint16_t mb_holding_regs[MODBUS_REG_NUM];

/* 公开接口 */
void Modbus_Init(void);
void Modbus_RxHandler(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t len);
void Modbus_TimerTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MODBUS_H__ */
