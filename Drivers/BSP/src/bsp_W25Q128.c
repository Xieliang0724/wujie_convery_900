#include "bsp_W25Q128.h"
#include "alarm.h"
#include "params.h"
#include <string.h>

/* ── 软件 SPI 引脚映射 ─────────────────────────────────────────
 *   CS   — PD12  推挽输出  空闲 HIGH（高电平 = 未选中）
 *   SCLK — PD13  推挽输出  空闲 HIGH（Mode 3，CPOL=1）
 *   MISO — PD14  上拉输入
 *   MOSI — PD15  推挽输出
 * ─────────────────────────────────────────────────────────── */
#define CS_LOW()    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define SCLK_LOW()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define SCLK_HIGH() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define MOSI_LOW()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)
#define MOSI_HIGH() HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
#define MISO_READ() HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)

/* ── W25Q128 指令码 ─────────────────────────────────────────── */
#define CMD_WRITE_ENABLE        0x06u
#define CMD_READ_STATUS1        0x05u
#define CMD_READ_DATA           0x03u
#define CMD_PAGE_PROGRAM        0x02u
#define CMD_SECTOR_ERASE        0x20u
#define CMD_BLOCK_ERASE_64K     0xD8u
#define CMD_CHIP_ERASE          0xC7u
#define CMD_JEDEC_ID            0x9Fu
#define CMD_RELEASE_POWER_DOWN  0xABu

#define SR1_BUSY_BIT  0x01u

/* ── 等待超时 ───────────────────────────────────────────────── */
#define TIMEOUT_PROG_MS         50u
#define TIMEOUT_SECTOR_MS       500u
#define TIMEOUT_BLOCK_MS        3000u
#define TIMEOUT_CHIP_MS         300000u

/* ═══════════════════════════════════════════════════════════
 * 底层 SPI — Mode 3 (CPOL=1, CPHA=1)
 *   SCLK 空闲高；在下降沿更新 MOSI，上升沿采样 MISO
 * ═══════════════════════════════════════════════════════════ */
static uint8_t SPI_SwapByte(uint8_t tx)
{
    uint8_t rx = 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
        if (tx & 0x80u) MOSI_HIGH(); else MOSI_LOW();
        tx <<= 1u;
        SCLK_LOW();                                   /* 下降沿：MOSI 已稳定 */
        SCLK_HIGH();                                  /* 上升沿：采样 MISO   */
        rx <<= 1u;
        if (MISO_READ() == GPIO_PIN_SET) rx |= 0x01u;
    }
    return rx;
}

static inline void    SPI_WriteByte(uint8_t b) { SPI_SwapByte(b); }
static inline uint8_t SPI_ReadByte(void)       { return SPI_SwapByte(0xFFu); }

/* ═══════════════════════════════════════════════════════════
 * 内部辅助
 * ═══════════════════════════════════════════════════════════ */
static uint8_t W25Q128_ReadSR1(void)
{
    uint8_t sr;
    CS_LOW();
    SPI_WriteByte(CMD_READ_STATUS1);
    sr = SPI_ReadByte();
    CS_HIGH();
    return sr;
}

static int8_t W25Q128_WaitBusy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (W25Q128_ReadSR1() & SR1_BUSY_BIT) {
        if ((HAL_GetTick() - start) >= timeout_ms) return W25Q128_ERR_TIMEOUT;
    }
    return W25Q128_OK;
}

static void W25Q128_WriteEnable(void)
{
    CS_LOW();
    SPI_WriteByte(CMD_WRITE_ENABLE);
    CS_HIGH();
}

static void W25Q128_SendAddr(uint32_t addr)
{
    SPI_WriteByte((uint8_t)(addr >> 16u));
    SPI_WriteByte((uint8_t)(addr >>  8u));
    SPI_WriteByte((uint8_t)(addr));
}

/* 自检使用最后一个 4KB 扇区（16MB - 4KB = 0xFFF000），不与业务数据冲突 */
#define FLASH_TEST_ADDR  0xFFF000u

static void Flash_SelfTest(void)
{
    static const uint8_t test_data[] = "Geekplus";   /* 8 字节，不含 '\0' */
    uint8_t rx_buf[8]                = {0};
    const uint8_t test_len           = (uint8_t)(sizeof(test_data) - 1u);

    /* V3 板 Flash 不在 PD12-15(实为输入引脚):只做读校验,不做擦写,
     * 避免上电等待编程超时、以及向输入通道打脉冲 */
    W25Q128_ReadData(FLASH_TEST_ADDR, rx_buf, test_len);

    if (memcmp(rx_buf, test_data, test_len) != 0) {
        g_alarm.flags.flash_fault = 1u;
    }
}

/* ═══════════════════════════════════════════════════════════
 * 公共 API
 * ═══════════════════════════════════════════════════════════ */

/**
 * 上电初始化：确保 CS 释放，唤醒芯片
 */
void W25Q128_Init(void)
{
    CS_HIGH();
    SCLK_HIGH();
    CS_LOW();
    SPI_WriteByte(CMD_RELEASE_POWER_DOWN);
    CS_HIGH();
    HAL_Delay(1u);  /* tRES1 最大 3µs */

	//检查是否正常
    Flash_SelfTest();

    /* 读取掉电保持参数 */
    Params_Load();
}

/**
 * 读 JEDEC ID — W25Q128JVSIQ 正常返回 0xEF4018
 */
uint32_t W25Q128_ReadJEDECID(void)
{
    uint32_t id;
    CS_LOW();
    SPI_WriteByte(CMD_JEDEC_ID);
    id  = (uint32_t)SPI_ReadByte() << 16u;
    id |= (uint32_t)SPI_ReadByte() <<  8u;
    id |=            SPI_ReadByte();
    CS_HIGH();
    return id;
}

/**
 * 读数据（任意长度，可跨页跨扇区）
 */
int8_t W25Q128_ReadData(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u) return W25Q128_ERR_PARAM;

    int8_t ret = W25Q128_WaitBusy(TIMEOUT_PROG_MS);
    if (ret != W25Q128_OK) return ret;

    CS_LOW();
    SPI_WriteByte(CMD_READ_DATA);
    W25Q128_SendAddr(addr);
    for (uint32_t i = 0u; i < len; i++) buf[i] = SPI_ReadByte();
    CS_HIGH();
    return W25Q128_OK;
}

/**
 * 页编程（单页，最多 256 字节）
 *   - addr 必须页对齐（低 8 位 = 0）
 *   - 目标区域必须已擦除（0xFF）
 */
int8_t W25Q128_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || len > W25Q128_PAGE_SIZE) return W25Q128_ERR_PARAM;

    int8_t ret = W25Q128_WaitBusy(TIMEOUT_PROG_MS);
    if (ret != W25Q128_OK) return ret;

    W25Q128_WriteEnable();
    CS_LOW();
    SPI_WriteByte(CMD_PAGE_PROGRAM);
    W25Q128_SendAddr(addr);
    for (uint16_t i = 0u; i < len; i++) SPI_WriteByte(buf[i]);
    CS_HIGH();

    return W25Q128_WaitBusy(TIMEOUT_PROG_MS);
}

/**
 * 连续写（自动分页，可跨页，不可跨越未擦除区域）
 *   - 目标区域必须已擦除（0xFF）
 */
int8_t W25Q128_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u) return W25Q128_ERR_PARAM;

    uint32_t written = 0u;
    while (written < len) {
        /* 当前页剩余空间 */
        uint32_t page_offset = (addr + written) & (W25Q128_PAGE_SIZE - 1u);
        uint32_t page_remain = W25Q128_PAGE_SIZE - page_offset;
        uint32_t chunk = len - written;
        if (chunk > page_remain) chunk = page_remain;

        int8_t ret = W25Q128_PageProgram(addr + written, buf + written, (uint16_t)chunk);
        if (ret != W25Q128_OK) return ret;

        written += chunk;
    }
    return W25Q128_OK;
}

/**
 * 扇区擦除（4KB），addr 取扇区内任意地址即可
 * 典型耗时 45ms，最大 400ms
 */
int8_t W25Q128_SectorErase(uint32_t addr)
{
    int8_t ret = W25Q128_WaitBusy(TIMEOUT_SECTOR_MS);
    if (ret != W25Q128_OK) return ret;

    W25Q128_WriteEnable();
    CS_LOW();
    SPI_WriteByte(CMD_SECTOR_ERASE);
    W25Q128_SendAddr(addr);
    CS_HIGH();

    return W25Q128_WaitBusy(TIMEOUT_SECTOR_MS);
}

/**
 * 块擦除（64KB），addr 取块内任意地址即可
 * 典型耗时 500ms，最大 2000ms
 */
int8_t W25Q128_BlockErase(uint32_t addr)
{
    int8_t ret = W25Q128_WaitBusy(TIMEOUT_BLOCK_MS);
    if (ret != W25Q128_OK) return ret;

    W25Q128_WriteEnable();
    CS_LOW();
    SPI_WriteByte(CMD_BLOCK_ERASE_64K);
    W25Q128_SendAddr(addr);
    CS_HIGH();

    return W25Q128_WaitBusy(TIMEOUT_BLOCK_MS);
}

/**
 * 全片擦除 — 耗时约 200s，谨慎使用
 */
int8_t W25Q128_ChipErase(void)
{
    int8_t ret = W25Q128_WaitBusy(TIMEOUT_CHIP_MS);
    if (ret != W25Q128_OK) return ret;

    W25Q128_WriteEnable();
    CS_LOW();
    SPI_WriteByte(CMD_CHIP_ERASE);
    CS_HIGH();

    return W25Q128_WaitBusy(TIMEOUT_CHIP_MS);
}

/**
 * 安全写入（自动读-改-擦-写，无需手动擦除）
 *   - 自动处理跨扇区写入
 *   - 内部使用 4KB 静态缓冲区，单线程安全
 *   - addr/len 无需对齐
 */
int8_t W25Q128_SafeWrite(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0u) return W25Q128_ERR_PARAM;

    static uint8_t sector_buf[W25Q128_SECTOR_SIZE];

    uint32_t written  = 0u;
    uint32_t cur_addr = addr;

    while (written < len) {
        uint32_t sector_addr   = cur_addr & ~(W25Q128_SECTOR_SIZE - 1u); /* 对齐到扇区起始 */
        uint32_t offset        = cur_addr - sector_addr;                  /* 在扇区内的偏移 */
        uint32_t chunk         = W25Q128_SECTOR_SIZE - offset;            /* 本扇区可写字节数 */
        if (chunk > (len - written)) chunk = len - written;

        /* 1. 读出整个扇区 */
        int8_t ret = W25Q128_ReadData(sector_addr, sector_buf, W25Q128_SECTOR_SIZE);
        if (ret != W25Q128_OK) return ret;

        /* 2. 在缓冲区中覆盖新数据 */
        memcpy(sector_buf + offset, buf + written, chunk);

        /* 3. 擦除扇区 */
        ret = W25Q128_SectorErase(sector_addr);
        if (ret != W25Q128_OK) return ret;

        /* 4. 写回整个扇区 */
        ret = W25Q128_Write(sector_addr, sector_buf, W25Q128_SECTOR_SIZE);
        if (ret != W25Q128_OK) return ret;

        written  += chunk;
        cur_addr += chunk;
    }
    return W25Q128_OK;
}
