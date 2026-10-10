#ifndef __PARAMS_H__
#define __PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ================================================================
 * Flash 存储地址（倒数第二个 4KB 扇区，与自检扇区 0xFFF000 不冲突）
 * ================================================================ */
#define PARAMS_FLASH_ADDR   0xFFE000u

/* 数据有效性魔数：首次写入或结构变更时需要更新版本号 */
#define PARAMS_MAGIC        0xA55A0001u

/* ================================================================
 * 掉电保持参数结构体
 *
 * 规则：
 *   - magic 字段不可删除，由驱动层维护，不要手动赋值
 *   - 新增字段直接在注释行下方添加
 *   - 删除或调整字段后需更新 PARAMS_MAGIC（版本号 +1），
 *     否则旧 Flash 数据会被当作有效数据读出
 * ================================================================ */
typedef struct {
    uint32_t magic;         /* 有效性校验，勿手动修改 */
	uint32_t test;
    /* ---- 在此处添加需要掉电保持的参数 ---- */

} Params_t;

extern Params_t g_params;  /* 全局参数实例，供外部读写 */

void Params_Load(void);    /* 上电时从 Flash 读取；无效数据则加载默认值 */
void Params_Update(void);  /* 主循环中调用；检测到变更时自动写入 Flash   */

#ifdef __cplusplus
}
#endif

#endif /* __PARAMS_H__ */
