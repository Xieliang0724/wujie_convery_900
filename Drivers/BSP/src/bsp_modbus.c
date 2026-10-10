#include "bsp_modbus.h"
#include <string.h>

/* ---- DMA 收发缓冲区 ---- */
uint8_t  mb_uart1_rx_buf[MB_BUF_SIZE];
uint8_t  mb_uart1_tx_buf[MB_BUF_SIZE];
uint8_t  mb_uart2_rx_buf[MB_BUF_SIZE];
uint8_t  mb_uart2_tx_buf[MB_BUF_SIZE];
uint8_t  mb_uart3_rx_buf[MB_BUF_SIZE];
uint8_t  mb_uart3_tx_buf[MB_BUF_SIZE];
uint8_t  mb_uart4_rx_buf[MB_BUF_SIZE];
uint8_t  mb_uart4_tx_buf[MB_BUF_SIZE];

/* ---- 保持寄存器表 ---- */
uint16_t mb_holding_regs[MODBUS_REG_NUM];

/* ---- 各端口独立的帧间隔计时器（单位：ms，由 TIM2 递减）---- */
static volatile uint32_t mb_uart1_silence = 0;
static volatile uint32_t mb_uart2_silence = 0;
static volatile uint32_t mb_uart3_silence = 0;
static volatile uint32_t mb_uart4_silence = 0;

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/* 构造异常响应，返回响应长度 */
static uint16_t mb_exception(uint8_t addr, uint8_t fc, uint8_t code, uint8_t *tx)
{
    tx[0] = addr;
    tx[1] = fc | 0x80;
    tx[2] = code;
    uint16_t crc = CRC16_Modbus(tx, 3);
    tx[3] = (uint8_t)(crc & 0xFF);
    tx[4] = (uint8_t)(crc >> 8);
    return 5;
}

/* FC03: 读保持寄存器 */
static uint16_t mb_fc03(const uint8_t *rx, uint16_t rx_len, uint8_t *tx)
{
    if (rx_len < 8) return 0;

    uint16_t start = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t count = ((uint16_t)rx[4] << 8) | rx[5];

    if (count == 0 || count > 125)
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_VALUE, tx);
    if ((uint32_t)start + count > MODBUS_REG_NUM)
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_ADDR, tx);

    uint16_t idx = 0;
    tx[idx++] = rx[0];
    tx[idx++] = rx[1];
    tx[idx++] = (uint8_t)(count * 2);
    for (uint16_t i = 0; i < count; i++) {
        tx[idx++] = (uint8_t)(mb_holding_regs[start + i] >> 8);
        tx[idx++] = (uint8_t)(mb_holding_regs[start + i] & 0xFF);
    }
    uint16_t crc = CRC16_Modbus(tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

/* FC06: 写单个寄存器 */
static uint16_t mb_fc06(const uint8_t *rx, uint16_t rx_len, uint8_t *tx)
{
    if (rx_len < 8) return 0;

    uint16_t reg = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t val = ((uint16_t)rx[4] << 8) | rx[5];

    if (reg >= MODBUS_REG_NUM)
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_ADDR, tx);

    mb_holding_regs[reg] = val;

    /* 响应：原样回显前 6 字节，重新附加 CRC */
    uint16_t idx = 0;
    tx[idx++] = rx[0];
    tx[idx++] = rx[1];
    tx[idx++] = rx[2];
    tx[idx++] = rx[3];
    tx[idx++] = rx[4];
    tx[idx++] = rx[5];
    uint16_t crc = CRC16_Modbus(tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

/* FC16: 写多个寄存器 */
static uint16_t mb_fc16(const uint8_t *rx, uint16_t rx_len, uint8_t *tx)
{
    if (rx_len < 11) return 0;

    uint16_t start    = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t count    = ((uint16_t)rx[4] << 8) | rx[5];
    uint8_t  byte_cnt = rx[6];

    if (count == 0 || count > 123)
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_VALUE, tx);
    if ((uint32_t)start + count > MODBUS_REG_NUM)
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_ADDR, tx);
    if (byte_cnt != (uint8_t)(count * 2))
        return mb_exception(rx[0], rx[1], MB_EX_ILLEGAL_DATA_VALUE, tx);
    if (rx_len < (uint16_t)(7u + byte_cnt + 2u))
        return 0; /* 帧不完整，丢弃 */

    for (uint16_t i = 0; i < count; i++) {
        mb_holding_regs[start + i] =
            ((uint16_t)rx[7 + i * 2] << 8) | rx[7 + i * 2 + 1];
    }

    /* 响应：地址 + FC16 + 起始地址 + 寄存器数量 + CRC */
    uint16_t idx = 0;
    tx[idx++] = rx[0];
    tx[idx++] = 0x10;
    tx[idx++] = rx[2];
    tx[idx++] = rx[3];
    tx[idx++] = rx[4];
    tx[idx++] = rx[5];
    uint16_t crc = CRC16_Modbus(tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

/**
 * Modbus_Init
 * 初始化寄存器表，启动 TIM2，使能两路串口 IDLE 中断并启动 DMA 接收
 * 在 MX_xxx_Init 之后调用一次
 */
void Modbus_Init(void)
{
    memset(mb_holding_regs, 0, sizeof(mb_holding_regs));
    mb_uart1_silence = 0;
    mb_uart3_silence = 0;

    HAL_TIM_Base_Start_IT(&htim2);

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, mb_uart2_rx_buf, MB_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, mb_uart4_rx_buf, MB_BUF_SIZE);
}

/**
 * Modbus_TimerTick
 * 在 TIM2 中断中调用，每 1ms 执行一次
 * 递减各端口的帧间隔静默计时器
 */
void Modbus_TimerTick(void)
{
    if (mb_uart1_silence > 0) mb_uart1_silence--;
if (mb_uart2_silence > 0) mb_uart2_silence--;
    if (mb_uart3_silence > 0) mb_uart3_silence--;
if (mb_uart4_silence > 0) mb_uart4_silence--;
}

/**
 * Modbus_RxHandler
 * 在 USART IDLE 中断中调用
 * 解析 Modbus RTU 帧，构造响应，通过 DMA 发送；无论是否响应都重启 DMA 接收
 *
 * @param huart   触发中断的串口句柄（huart1 或 huart3）
 * @param rx_buf  DMA 接收缓冲区指针
 * @param len     本帧实际接收字节数
 */
void Modbus_RxHandler(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t len)
{
    uint8_t           *tx_buf;
    volatile uint32_t *silence;

    if (huart->Instance == USART2) {
        tx_buf  = mb_uart2_tx_buf;
        silence = &mb_uart2_silence;
    } else if (huart->Instance == UART4) {
        tx_buf  = mb_uart4_tx_buf;
        silence = &mb_uart4_silence;
    } else {
        tx_buf  = mb_uart3_tx_buf;
        silence = &mb_uart3_silence;
    }

    uint16_t tx_len = 0;

    /* 帧间隔静默期内忽略收到的数据 */
    if (*silence == 0 && len >= 8) {
        /* 校验地址 */
        if (rx_buf[0] == MODBUS_SLAVE_ADDR) {
            /* 校验 CRC（帧尾低字节在前、高字节在后）*/
            uint16_t crc_recv = (uint16_t)rx_buf[len - 2] |
                                ((uint16_t)rx_buf[len - 1] << 8);
            if (CRC16_Modbus(rx_buf, len - 2) == crc_recv) {
                switch (rx_buf[1]) {
                    case MB_FC_READ_HOLDING_REGS:
                        tx_len = mb_fc03(rx_buf, len, tx_buf);
                        break;
                    case MB_FC_WRITE_SINGLE_REG:
                        tx_len = mb_fc06(rx_buf, len, tx_buf);
                        break;
                    case MB_FC_WRITE_MULTI_REGS:
                        tx_len = mb_fc16(rx_buf, len, tx_buf);
                        break;
                    default:
                        tx_len = mb_exception(rx_buf[0], rx_buf[1],
                                              MB_EX_ILLEGAL_FUNCTION, tx_buf);
                        break;
                }
            }
        }
    }

    if (tx_len > 0) {
        HAL_UART_Transmit_DMA(huart, tx_buf, tx_len);
        *silence = MB_FRAME_TIMEOUT_MS;
    }

    /* 无论是否响应，都重新启动 DMA 接收，准备下一帧 */
    HAL_UART_Receive_DMA(huart, rx_buf, MB_BUF_SIZE);
}
