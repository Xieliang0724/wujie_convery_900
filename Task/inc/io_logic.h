#ifndef __IO_LOGIC_H__
#define __IO_LOGIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

/* 输入结构体（1=有效/按下） */
typedef struct {
	uint8_t PNP_in[8];	/* PNP输入（保留） */
} IO_Input_t;

/* LED 结构体 */
typedef struct {
    uint8_t working;    /* working 灯 — PE5（高=亮，500ms 闪烁） */
    uint8_t network;    /* network 灯 — PE3（低=亮） */
    uint8_t system;     /* 系统/运行状态：PE2=镜像(高=亮)，PE4=反相(高=亮) */
} LED_t;

/* 输出结构体 */
typedef struct {
	uint8_t PNP_out[4];	/* PNP输出（保留） */
    LED_t led;
} IO_Output_t;

typedef struct {
    IO_Input_t  input;
    IO_Output_t output;
} IO_t;

extern IO_t g_io;

void IO_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* __IO_LOGIC_H__ */
