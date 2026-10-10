# SCH_NET-RELAY 硬件资源 / 引脚 / 串口 / Modbus 寄存器整理

> 主控 GD32F407VG（LQFP100），代码基于 STM32 HAL，寄存器/引脚与 STM32F407 兼容。
> 内容依据源码核对（`Core/Src/*`、`Task/*`、`Drivers/BSP/*`、`README.md`），随代码更新。

---

## 1. 系统资源总览

| 项 | 值 |
|---|---|
| MCU | GD32F407VG（实际）/ 工程按 STM32F407 处理 |
| 封装 | LQFP100 |
| 系统时钟 | SYSCLK 168 MHz（HSE 8MHz → PLL：M=4, N=168, P=2） |
| AHB / APB1 / APB2 | 168 MHz / 42 MHz / 84 MHz |
| TIM2 时钟 | APB1×2 = 84 MHz |
| 调试 | ST-LINK / J-Link（SWD：PA13/PA14） |
| 外部晶振 | HSE 8MHz（PH0/PH1）、LSE 32.768kHz（PC14/PC15，已配未用） |
| 外部存储 | W25Q128（16Mbit，软件 SPI） |

**用到的主要外设**

| 外设 | 实例/编号 | 用途 |
|---|---|---|
| USART | USART1 / USART3 / USART6 | USART1/3 = Modbus 从站；USART6 预留 |
| 定时器 | TIM2 | 1ms 帧间隔静默计时（Modbus） |
| DMA | DMA1 / DMA2 多个 Stream | 串口收/发 DMA |
| EXTI | EXTI3 / EXTI4 | 双路急停上升沿中断 |
| GPIO | GPIOA/B/C/D/E | 输入/输出/通信引脚 |
| SPI（软件） | PD12/13/14/15 | W25Q128 外部 Flash |

---

## 2. 引脚分配表

### 2.1 输入引脚

| 引脚 | 信号 | 极性 | 读取反相 | 备注 |
|---|---|---|---|---|
| PB3 | 急停1 `estop1` | 高=按下 | 否 | EXTI3 上升沿触发 |
| PB4 | 急停2 `estop2` | 高=按下 | 否 | EXTI4 上升沿触发 |
| PE1 | 复位键 `reset` | 低有效 | 是（读反相，1=按下） | 轮询 |
| PC12 | PNP_in[0] | 低=有效 | 是（读反相，1=有效） | 轮询 |
| PD0 | PNP_in[1] | 低=有效 | 是 | 轮询 |
| PD1 | PNP_in[2] | 低=有效 | 是 | 轮询 |
| PD2 | PNP_in[3] | 低=有效 | 是 | 轮询 |
| PD4 | PNP_in[4] | 低=有效 | 是 | 轮询 |
| PD5 | PNP_in[5] | 低=有效 | 是 | 轮询 |
| PD6 | PNP_in[6] | 低=有效 | 是 | 轮询 |
| PD7 | PNP_in[7] | 低=有效 | 是 | 轮询 |
| PD14 | W25Q128 MISO | 上拉输入 | — | 软件 SPI 读数据 |

> 代码位置：`Task/src/io_logic.c` 的 `IO_InputUpdate()`；GPIO 配置在 `Core/Src/gpio.c`。

### 2.2 输出引脚

| 引脚 | 信号 | 极性 | 备注 |
|---|---|---|---|
| PD9 | relay[0] | 高=吸合 | 继电器 0 |
| PE14 | relay[1] | 高=吸合 | 继电器 1 |
| PD8 | relay[2] | 高=吸合 | 继电器 2 |
| PE15 | relay[3] | 高=吸合 | 继电器 3 |
| PE13 | PNP_out[0] | 低=导通 | 当前由 LED 逻辑驱动 = system 灯 |
| PE12 | PNP_out[1] | 低=导通 | 当前 = PNP_out[0] 取反 |
| PE11 | PNP_out[2] | 低=导通 | 可由 Modbus reg 40 控制 |
| PE10 | PNP_out[3] | 低=导通 | 可由 Modbus reg 41 控制 |
| PE4 | LED working | 高=亮 | 正常时 500ms 闪烁 |
| PE3 | LED network | **低=亮**（反向） | 未使用，恒灭 |
| PE2 | LED system | 高=亮 | 正常时常亮 |

> 输出极性在 `Task/src/io_logic.c` 的 `IO_OutputUpdate()` 里统一实现；LED 逻辑在 `Task/src/led.c`。

### 2.3 通信引脚

| 引脚 | 功能 | 复用 | 备注 |
|---|---|---|---|
| PA9 / PA10 | USART1_TX / USART1_RX | GPIO_AF7_USART1 | Modbus 从站① |
| PC10 / PC11 | USART3_TX / USART3_RX | GPIO_AF7_USART3 | Modbus 从站② |
| PC6 / PC7 | USART6_TX / USART6_RX | GPIO_AF8_USART6 | 预留未用 |
| PA13 / PA14 | SWDIO / SWCLK | — | 调试口 |

### 2.4 外部 Flash（W25Q128）软件 SPI 引脚

| 引脚 | 功能 | 说明 |
|---|---|---|
| PD12 | CS（片选，低有效） | 推挽输出，空闲高 |
| PD13 | SCLK | 推挽输出，Mode 3（CPOL=1，空闲高） |
| PD15 | MOSI | 推挽输出 |
| PD14 | MISO | 上拉输入 |

> 软件 SPI 实现在 `Drivers/BSP/src/bsp_W25Q128.c`（`CS_LOW/HIGH`、`SCLK_*`、`MOSI_*`、`MISO_READ()`）。

---

## 3. 串口详细信息

| 项 | USART1 | USART3 | USART6 |
|---|---|---|---|
| 实例 | USART1 | USART3 | USART6 |
| 引脚 | PA9(TX) / PA10(RX) | PC10(TX) / PC11(RX) | PC6(TX) / PC7(RX) |
| 复用 | GPIO_AF7_USART1 | GPIO_AF7_USART3 | GPIO_AF8_USART6 |
| 波特率 | 115200 | 115200 | 115200 |
| 数据位 | 8 | 8 | 8 |
| 校验 | 无 | 无 | 无 |
| 停止位 | 1 | 1 | 1 |
| 流控 | 无 | 无 | 无 |
| 接收 DMA | DMA2_Stream2 / CH4 | DMA1_Stream1 / CH4 | DMA2_Stream1 / CH5 |
| 发送 DMA | DMA2_Stream7 / CH4 | DMA1_Stream3 / CH4 | DMA2_Stream6 / CH5 |
| 用途 | Modbus 从站① | Modbus 从站② | 预留（未接入逻辑） |
| 中断 | USART1_IRQn（IDLE+DMA） | USART3_IRQn（IDLE+DMA） | USART6_IRQn（仅 HAL 处理，未启用 IDLE 逻辑） |

**要点**
- **Modbus 实际使用 USART1 和 USART3**，两路均为 115200 8N1，同一从站地址 `1`。
- 接收用 **DMA + 空闲（IDLE）中断**，帧尾由 **TIM2 1ms 计时的 4ms 静默时间**判定。
- USART6 已初始化（引脚/DMA/中断都在），但**没有任何应用读写**，位于 `Core/Src/usart.c`，可作后续调试口/第三路通信。

---

## 4. Modbus 寄存器表（从站地址 1）

- 保持寄存器数量：**50**（`MODBUS_REG_NUM`，地址 `0~49`）
- 支持功能码：**03** 读保持寄存器 / **06** 写单寄存器 / **10** 写多寄存器
- 异常码：01 非法功能 / 02 非法地址 / 03 非法数据值
- 帧间隔静默：4ms（115200bps 下 3.5T≈0.3ms）
- 双口同时服务：USART1 + USART3，同一从站地址

| 寄存器 | 含义 | 方向 |
|---|---|---|
| 0 | 任务状态（1=正常） | 只读（映射覆盖） |
| 20 | 急停1 | 只读 |
| 21 | 急停2 | 只读 |
| 22 | 复位键 | 只读 |
| 23 | PNP_in[0] | 只读 |
| 24 | PNP_in[1] | 只读 |
| 25 | PNP_in[2] | 只读 |
| 26 | PNP_in[3] | 只读 |
| 27 | PNP_in[4] | 只读 |
| 28 | PNP_in[5] | 只读 |
| 29 | PNP_in[6] | 只读 |
| 30 | PNP_in[7] | 只读 |
| 31 | relay[0] | ✅ 上位机可写（钳位 0/1，当周期生效） |
| 32 | relay[1] | ✅ 上位机可写 |
| 33 | relay[2] | ✅ 上位机可写 |
| 34 | relay[3] | ✅ 上位机可写 |
| 35 | LED working | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 36 | LED network | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 37 | LED system | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 38 | PNP_out[0] | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 39 | PNP_out[1] | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 40 | PNP_out[2] | ✅ 上位机可写（钳位 0/1，当周期生效） |
| 41 | PNP_out[3] | ✅ 上位机可写 |
| 其余（1~19, 42~49） | 空闲 | 可读写（不参与映射，写后保留） |

> 正向（状态→寄存器）映射：`modbus_data_mapping()`；反向（寄存器→输出）映射：`modbus_write_mapping()`，均在 `Task/src/io_logic.c`。寄存器表定义在 `Drivers/BSP/src/bsp_modbus.c`。

---

## 5. 定时器 / 时间基

| 定时器 | 配置 | 周期 | 用途 |
|---|---|---|---|
| TIM2 | Prescaler=83, Period=999（84MHz → 1kHz） | 1 ms | `Modbus_TimerTick()` 递减各串口帧静默计时 |
| SysTick | HAL 默认 1ms | 1 ms | `HAL_GetTick()`，用于 LED 500ms 闪烁等 |

---

## 6. 中断资源

| 中断 | 触发源 | 优先级 | 处理 |
|---|---|---|---|
| EXTI3 | PB3 急停1 上升沿 | 0,0 | `Task_EStopISR()` → 状态机置 `ESTOP_PRESSED` |
| EXTI4 | PB4 急停2 上升沿 | 0,0 | `Task_EStopISR()` |
| TIM2 | TIM2 更新 | 0,0 | `Modbus_TimerTick()` |
| USART1 | IDLE / RX | 0,0 | `Modbus_RxHandler(&huart1,...)` |
| USART3 | IDLE / RX | 0,0 | `Modbus_RxHandler(&huart3,...)` |
| USART6 | — | 0,0 | 仅 HAL 处理 |
| DMA1_Stream1/3 / DMA2_Stream1/2/6/7 | 各串口 DMA | 0,0 | DMA 完成/错误处理 |

---

## 7. 主循环顺序（`Core/Src/main.c`）

```
内部初始化：MX_GPIO → MX_DMA → MX_TIM2 → MX_USART1/3/6
用户初始化：Alarm_Init → Modbus_Init → LED_Init → IO_Update → Task_Init → W25Q128_Init
主循环：
  IO_Update()    // 读输入 → 寄存器映射（正向+反向）→ 写输出引脚
  Task_Update()  // 急停/复位状态机
  Alarm_Update() // 急停双路不一致报警、reset 清报警
  LED_Update()   // LED 逻辑 + PNP_out[0..1]
  Params_Update()// 掉电参数
```

> 注意：`Task_Init()` 读取急停电平，需在第一次 `IO_Update()` 之后调用；当前 `main.c` 已先 `IO_Update()` 再 `Task_Init()`。

---

## 8. 安全相关提醒（根据现源码）

1. **急停未强制切断安全输出**：继电器（`relay[0..3]`）和 `PNP_out[2..3]` 直接由 Modbus 寄存器驱动，不受 `task_state` 门控；急停按下时输出不会被强制回到安全态，上位机仍可在急停期间写寄存器吸合继电器。
2. **复位键会无条件清掉所有报警**：`Alarm_Update()` 中按住复位即 `memset` 清空全部报警，即使故障（Flash 故障、急停不一致）仍然存在。
3. `working` LED 无状态判断，任何状态下都每 500ms 翻转；`network` LED 未驱动恒灭（已注明）。
