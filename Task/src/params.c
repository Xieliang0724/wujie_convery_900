#include "params.h"
#include "alarm.h"
#include "bsp_W25Q128.h"
#include <string.h>

Params_t g_params;

/* 上次写入 Flash 的镜像副本，用于变更检测 */
static Params_t params_shadow;

/* ── 默认值 ─────────────────────────────────────────────────────
 * Flash 中数据无效（首次上电或 magic 不匹配）时加载此默认值
 * 新增字段时在此处初始化对应的默认值
 * ─────────────────────────────────────────────────────────────── */
static void Params_SetDefaults(void)
{
    memset(&g_params, 0, sizeof(g_params));
    g_params.magic = PARAMS_MAGIC;
    /* 在此设置各字段的默认值 */
}

/* ── Params_Load ─────────────────────────────────────────────────
 * 从 Flash 读出数据；magic 校验失败则恢复默认值并同步镜像
 * 由 W25Q128_Init() 在上电完成后调用
 * ─────────────────────────────────────────────────────────────── */
void Params_Load(void)
{
    W25Q128_ReadData(PARAMS_FLASH_ADDR, (uint8_t *)&g_params, sizeof(Params_t));

    if (g_params.magic != PARAMS_MAGIC) {
        Params_SetDefaults();
    }

    params_shadow = g_params;  /* 初始化镜像，避免上电后立即触发写入 */
}

/* ── Params_Update ───────────────────────────────────────────────
 * 主循环中调用；将 g_params 与镜像对比，有变更则写入 Flash
 * flash_fault 报警激活时跳过写入，避免对故障芯片的无效操作
 * ─────────────────────────────────────────────────────────────── */
void Params_Update(void)
{
    if (g_alarm.flags.flash_fault) return;

    if (memcmp(&g_params, &params_shadow, sizeof(Params_t)) != 0) {
        g_params.magic = PARAMS_MAGIC;
        W25Q128_SafeWrite(PARAMS_FLASH_ADDR, (uint8_t *)&g_params, sizeof(Params_t));
        params_shadow = g_params;
    }
}
