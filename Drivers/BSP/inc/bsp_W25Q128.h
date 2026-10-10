#ifndef __BSP_W25Q128_H__
#define __BSP_W25Q128_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ── Flash 几何参数 ──────────────────────────────────────────── */
#define W25Q128_PAGE_SIZE     256u          /* 1 页 = 256 字节     */
#define W25Q128_SECTOR_SIZE   4096u         /* 1 扇区 = 4KB        */
#define W25Q128_BLOCK_SIZE    65536u        /* 1 块 = 64KB         */
#define W25Q128_TOTAL_SIZE    (16u * 1024u * 1024u)  /* 16MB 总容量 */

/* ── JEDEC ID (厂商 0xEF · 内存类型 0x40 · 容量 0x18=128Mb) ─── */
#define W25Q128_JEDEC_ID      0xEF4018u

/* ── 返回值 ─────────────────────────────────────────────────── */
#define W25Q128_OK            ( 0)
#define W25Q128_ERR_TIMEOUT   (-1)
#define W25Q128_ERR_PARAM     (-2)

/* ── 公共 API ─────────────────────────────────────────────────
 *
 *  W25Q128_Init          — 上电初始化，释放 Power-Down 状态
 *  W25Q128_ReadJEDECID   — 读 JEDEC ID，正常返回 0xEF4018
 *
 *  W25Q128_ReadData      — 读任意长度，地址自动递增，可跨页/跨扇区
 *
 *  W25Q128_PageProgram   — 写一页（最多 256 字节）
 *                          要求：addr 低 8 位 = 0（页对齐）
 *                          注意：写入前目标区域必须已擦除（数据为 0xFF）
 *
 *  W25Q128_Write         — 多页连续写（内部自动分页），同样要求目标已擦除
 *
 *  W25Q128_SectorErase   — 擦除 4KB 扇区（addr 取扇区内任意地址）
 *  W25Q128_BlockErase    — 擦除 64KB 块
 *  W25Q128_ChipErase     — 全片擦除（耗时约 200s，谨慎使用）
 * ─────────────────────────────────────────────────────────── */

void     W25Q128_Init(void);
uint32_t W25Q128_ReadJEDECID(void);

int8_t   W25Q128_ReadData(uint32_t addr, uint8_t *buf, uint32_t len);
int8_t   W25Q128_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len);
int8_t   W25Q128_Write(uint32_t addr, const uint8_t *buf, uint32_t len);
int8_t   W25Q128_SafeWrite(uint32_t addr, const uint8_t *buf, uint32_t len);

int8_t   W25Q128_SectorErase(uint32_t addr);
int8_t   W25Q128_BlockErase(uint32_t addr);
int8_t   W25Q128_ChipErase(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_W25Q128_H__ */
