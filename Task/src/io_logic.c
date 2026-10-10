#include "io_logic.h"
#include "bsp_servo.h"
#include "bsp_modbus.h"
#include <stdint.h>

IO_t g_io;

/* 读取 PNP 输入（低有效，读取反相） */
static void IO_InputUpdate(void)
{
    g_io.input.PNP_in[0] = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[1] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[2] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[3] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[4] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[5] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[6] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) == GPIO_PIN_SET) ? 0u : 1u;
    g_io.input.PNP_in[7] = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET) ? 0u : 1u;
}

/* LED 引脚（与硬件一致） */
static void IO_OutputUpdate(void)
{
    /* PE2 = system 镜像(高=亮) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, g_io.output.led.system ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* PE3 = network(低=亮) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, g_io.output.led.network ? GPIO_PIN_RESET : GPIO_PIN_SET);
    /* PE4 = system 反相(高=亮) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, (g_io.output.led.system ^ 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* PE5 = working(高=亮) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, g_io.output.led.working ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void IO_Update(void)
{
	IO_InputUpdate();
	/* PNP 输入映射到 Modbus reg23~30(1=有效,每周期覆盖) */
	for (uint8_t i = 0; i < 8u; i++) {
		mb_holding_regs[SERVO_MB_REG_PNP_IN0 + i] = g_io.input.PNP_in[i];
	}
	IO_OutputUpdate();
}
