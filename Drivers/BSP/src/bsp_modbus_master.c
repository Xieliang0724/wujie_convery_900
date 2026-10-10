/**
 * bsp_modbus_master.c
 * 标准 Modbus RTU 主站：用于 USART6(RS-485) 驱动伺服等从站。
 * 依赖 CRC16_Modbus()，无需软件控制 485 方向（硬件 Q5 自动换向）。
 */
#include "bsp_modbus_master.h"
#include "crc.h"
#include <string.h>

#define MB_M_MAX_RESPONSE   64u    /* 伺服响应最长 8 字节;限长防噪声喂满长收包阻塞主循环 */
#define MB_M_INTERBYTE_MS   3u     /* 帧内字节间隔超时(3ms≈26字节周期),限制噪声长收包 */
#define MB_M_FIRSTBYTE_MS   200u   /* 首字节等待(配合发送前清残留,正常周转几十 ms 即可对齐) */

/* 诊断计数器(J-Link 可直接读,正常量产可删) */
volatile uint32_t g_mb_m_tx_frames = 0;
volatile uint32_t g_mb_m_rx_bytes = 0;

/* 构造 Modbus RTU 请求帧（返回帧长），CRC 附加在末尾（低字节在前） */
static uint16_t mb_build_request(uint8_t slave, uint8_t fc, const uint8_t *payload,
                                 uint16_t payload_len, uint8_t *tx)
{
    uint16_t idx = 0;
    tx[idx++] = slave;
    tx[idx++] = fc;
    if (payload_len && payload) {
        memcpy(&tx[idx], payload, payload_len);
        idx += payload_len;
    }
    uint16_t crc = CRC16_Modbus(tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

/* 以字节间隔超时方式接收一帧响应，返回接收字节数（0=超时/无数据） */
static uint16_t mb_recv_frame(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t max)
{
    uint16_t n = 0;
    /* 首字节用长超时等周转,帧内字节用短超时判帧尾 */
    if (HAL_UART_Receive(huart, &buf[0], 1u, MB_M_FIRSTBYTE_MS) != HAL_OK) {
        return 0;   /* 50ms 内无首字节 = 从站无应答 */
    }
    n = 1;
    while (n < max) {
        if (HAL_UART_Receive(huart, &buf[n], 1u, MB_M_INTERBYTE_MS) != HAL_OK) {
            break;   /* 超过帧内字节间隔，认为帧结束 */
        }
        n++;
    }
    g_mb_m_rx_bytes += n;   /* 诊断:累计主站收到字节数 */
    return n;
}

/* 校验响应：地址+功能码 匹配、CRC 正确；返回帧长度或状态 */
static MB_Master_Status_t mb_check_response(uint8_t slave, uint8_t fc,
                                            const uint8_t *rx, uint16_t len)
{
    if (len < 5u) return MB_M_ERR_RX;
    if (rx[0] != slave)   return MB_M_ERR_RX;
    if (rx[1] == (fc | 0x80u)) return MB_M_ERR_EXCEPTION;   /* 异常响应 */
    if (rx[1] != fc)      return MB_M_ERR_RX;

    uint16_t crc_recv = (uint16_t)rx[len - 2] | ((uint16_t)rx[len - 1] << 8);
    if (CRC16_Modbus(rx, len - 2) != crc_recv) return MB_M_ERR_CRC;
    return MB_M_OK;
}

/* 一次主站事务：发请求 -> 收响应 -> 校验 */
static MB_Master_Status_t mb_master_xact(uint8_t slave, uint8_t fc,
                                         const uint8_t *req, uint16_t req_len,
                                         uint8_t *resp, uint16_t *resp_len,
                                         uint32_t timeout_ms)
{
    (void)timeout_ms;   /* 当前以字节间隔超时为主，这里保留接口 */

    /* 清残留:上一事务晚到的应答字节若不清,会错位混入本次响应导致校验失败 */
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
        (void)(uint8_t)(huart6.Instance->DR & 0xFFu);
    __HAL_UART_CLEAR_OREFLAG(&huart6);

    if (HAL_UART_Transmit(&huart6, (uint8_t *)req, req_len, 20u) != HAL_OK)
        return MB_M_ERR_TX;
    g_mb_m_tx_frames++;   /* 诊断:累计主站发送帧数 */

    /* 发送完成(HAL 已等 TC)后立即开收:伺服响应周转可能只有几百 µs,不能留延时窗口 */
    uint16_t len = mb_recv_frame(&huart6, resp, MB_M_MAX_RESPONSE);
    if (len == 0u) return MB_M_ERR_TIMEOUT;

    /* 帧头对齐:主站自身发送期间 RO 高阻会在响应前混入毛刺字节,扫描到 从站地址+功能码 为止 */
    for (uint16_t i = 0; i + 5u <= len; i++) {
        if (resp[i] == slave && resp[i + 1] == fc) {
            if (i > 0) {
                memmove(resp, &resp[i], (size_t)(len - i));
                len -= i;
            }
            break;
        }
    }

    MB_Master_Status_t st = mb_check_response(slave, fc, resp, len);
    if (st == MB_M_OK) *resp_len = len;
    return st;
}

/* FC03：读保持寄存器 */
MB_Master_Status_t Modbus_Master_ReadHoldingRegs(uint8_t slave, uint16_t start,
                                                 uint16_t count,
                                                 uint16_t *data,
                                                 uint32_t timeout_ms)
{
    if (data == NULL || count == 0u || count > 125u) return MB_M_ERR_PARAM;

    uint8_t req[8];
    uint8_t p[4] = { (uint8_t)(start >> 8), (uint8_t)start,
                     (uint8_t)(count >> 8), (uint8_t)count };
    uint16_t req_len = mb_build_request(slave, 0x03, p, 4u, req);

    uint8_t resp[MB_M_MAX_RESPONSE];
    uint16_t resp_len = 0;
    MB_Master_Status_t st = mb_master_xact(slave, 0x03, req, req_len, resp, &resp_len, timeout_ms);
    if (st != MB_M_OK) return st;

    uint8_t byte_cnt = resp[2];
    if (resp_len != (uint16_t)(3u + byte_cnt + 2u) || byte_cnt != (uint8_t)(count * 2u))
        return MB_M_ERR_RX;

    for (uint16_t i = 0; i < count; i++) {
        data[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1];
    }
    return MB_M_OK;
}

/* FC06：写单个寄存器 */
MB_Master_Status_t Modbus_Master_WriteSingleReg(uint8_t slave, uint16_t reg,
                                                uint16_t val, uint32_t timeout_ms)
{
    uint8_t req[8];
    uint8_t p[4] = { (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(val >> 8), (uint8_t)val };
    uint16_t req_len = mb_build_request(slave, 0x06, p, 4u, req);

    uint8_t resp[8];
    uint16_t resp_len = 0;
    MB_Master_Status_t st = mb_master_xact(slave, 0x06, req, req_len, resp, &resp_len, timeout_ms);
    if (st != MB_M_OK) return st;
    if (resp_len != 8u || resp[2] != req[2] || resp[3] != req[3]) return MB_M_ERR_RX;
    return MB_M_OK;
}

/* FC16：写多个寄存器 */
MB_Master_Status_t Modbus_Master_WriteMultipleRegs(uint8_t slave, uint16_t start,
                                                   uint16_t count,
                                                   const uint16_t *data,
                                                   uint32_t timeout_ms)
{
    if (data == NULL || count == 0u || count > 123u) return MB_M_ERR_PARAM;

    uint8_t req[260];
    uint8_t p[5 + count * 2];
    p[0] = (uint8_t)(start >> 8); p[1] = (uint8_t)start;
    p[2] = (uint8_t)(count >> 8); p[3] = (uint8_t)count;
    p[4] = (uint8_t)(count * 2u);
    for (uint16_t i = 0; i < count; i++) {
        p[5 + i * 2]     = (uint8_t)(data[i] >> 8);
        p[5 + i * 2 + 1] = (uint8_t)data[i];
    }
    uint16_t req_len = mb_build_request(slave, 0x10, p, (uint16_t)(5u + count * 2u), req);

    uint8_t resp[8];
    uint16_t resp_len = 0;
    MB_Master_Status_t st = mb_master_xact(slave, 0x10, req, req_len, resp, &resp_len, timeout_ms);
    if (st != MB_M_OK) return st;
    if (resp_len != 8u || resp[2] != req[2] || resp[3] != req[3] ||
        resp[4] != req[4] || resp[5] != req[5]) return MB_M_ERR_RX;
    return MB_M_OK;
}

/* 主站初始化：预留（USART6 已在 usart.c 初始化） */
void Modbus_Master_Init(void)
{
    /* 无额外初始化；USART6 由 MX_USART6_UART_Init() 配置 */
}

/* 调试用:连续背靠背发送 10 秒(满占空比),用于万用表测 DE/A-B;测完自动返回 */
void Modbus_Master_BurstTest(void)
{
    uint8_t req[8];
    uint8_t p[4] = { 0x0Bu, 0x00u, 0x00u, 0x01u };   /* FC03 读 H0B_00 */
    uint16_t len = mb_build_request(0x01u /*SERVO_SLAVE_ADDR*/, 0x03, p, 4u, req);
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 60000u) {   /* 调试窗口 60 秒 */
        if (HAL_UART_Transmit(&huart6, req, len, 20u) == HAL_OK) g_mb_m_tx_frames++;
    }
}
