# wujie_convery_900 项目记录（输送带伺服控制）

> 本工程由 SafeHalt NET-RELAY 派生（GD32F407VG），基于 **RS-485 Modbus RTU** 驱动**四横低压伺服**，实现**单轴输送带运动控制**。
> 本文档记录项目当前状态，作为后续开发的需求整理基线。
> 最后更新：2026-10-10（备注：双传感器控制功能；传感器信号**低电平有效**）

---

## 0. 当前功能（传感器控制）

> **传感器信号极性**：默认**高电平**，检测到才输出**低电平**（低=检测到/有效）。

- **传感器1**（板载 PNP 输入1 / PD0 → reg24）：检测到来料（**低电平**，上升沿）→ **启动**输送带并锁存运行，物料离开后继续转
- **传感器2**（板载 PNP 输入2 / PD1 → reg25）：检测到物料到位（**低电平**，电平）→ **立即停机**，并置 reg4 停机标志
- **启动按钮**（PB3，常闭，低=按下）：上升沿启动，**同时武装自动模式**；**停止按钮**（PB4，常闭，低=按下）：停止
- **武装（armed）机制**：上电默认未武装，来料位有物料也**不自动启动**，需**人工按一次启动按钮**；此后保持武装，**来料位有物料即由传感器1自动启动**（含传感器2停后再次来料）；**急停后解除武装**，需重新按启动按钮
- 启动前提：传感器2未有效（物料已取走）且停止按钮未按下；**停止优先**
- 运行逻辑为**边沿触发·锁存**：传感器1/启动按钮触发后保持运行，直至传感器2触发或停止按钮按下
- 寄存器语义：`reg22`/`reg49` = 传感器1/2，**检测到=1**（内部按低电平判有效）

---

## 1. 项目概述

- **项目名**：wujie_convery_900（输送带/单轴伺服控制器）
- **实际主控**：GD32F407VG（LQFP100）
- **工程基准**：Keil 工程器件已配置为 GD32F407VG（GigaDevice.GD32F4xx_DFP 3.0.3），代码仍基于 STM32 HAL（GD32F407 与 STM32F407 引脚/寄存器兼容，代码直接跑在 GD32 上）
- **固件形态**：裸机超循环（无 RTOS），CubeMX 生成骨架 + 自研 BSP/应用层
- **当前已实现功能**：
  - 单轴伺服运动：**按钮启停 + 双传感器控制**（传感器1启动/传感器2停止，见上）
  - 8 路 PNP 输入（板载隔离）、LED 指示（working 灯三态指示伺服通信/故障）
  - Modbus RTU：主站（USART6/RS-485 驱动伺服）+ 从站（USART2/UART4，03/06/10 功能码）
  - W25Q128 外部 Flash（软件 SPI）+ 内部 Flash（伺服参数掉电保存）
  - 485 通信/伺服状态监控（reg0 状态字）、RS-485 通信计数诊断

---

## 2. 硬件平台

| 项 | 值 |
|---|---|
| MCU | GD32F407VG（实际）/ STM32F407VGTx（工程配置） |
| 封装 | LQFP100 |
| 系统时钟 | 168 MHz（HSE 8MHz → PLL：M=4, N=168, P=2）|
| 总线时钟 | AHB=168M，APB1=42M（TIM2 时钟 84M），APB2=84M |
| 调试 | ST-LINK（SWD：PA13/PA14） |
| 外部晶振 | HSE 8MHz（PH0/PH1）、LSE 32.768kHz（PC14/PC15，已配未用） |
| 外部存储 | W25Q128（16Mbit NOR Flash，软件 SPI） |
| 上位机通信 | Modbus RTU × 2 路（USART1、USART3，115200 8N1） |

---

## 3. 工程结构与工具链

### 3.1 工具链

- **IDE**：Keil MDK-ARM V5.33（工程文件版本 V5.32 兼容）
- **编译器**：ARMCC V5.06 update 7（build 960）
- **工程生成**：STM32CubeMX 6.15.0，固件包 STM32Cube FW_F4 V1.28.3
- **构建宏**：`USE_HAL_DRIVER, STM32F407xx`
- **链接脚本**：MDK-ARM/SCH_NET-RELAY/SCH_NET-RELAY.sct（默认分散加载）
- **最近一次构建**：0 Error, 0 Warning（产物：.hex/.axf 在 MDK-ARM/SCH_NET-RELAY/）
- **辅助**：VS Code + Keil Assistant 插件（MDK-ARM/.vscode）

### 3.2 目录结构

```
SCH_NET-RELAY/
├── Core/
│   ├── Inc/  main.h gpio.h dma.h tim.h usart.h stm32f4xx_it.h stm32f4xx_hal_conf.h
│   └── Src/  main.c gpio.c dma.c tim.c usart.c stm32f4xx_it.c stm32f4xx_hal_msp.c system_stm32f4xx.c
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/   HAL 库源码
│   ├── CMSIS/                  内核/设备支持
│   └── BSP/                    自研板级驱动
│       ├── inc/  bsp_modbus.h bsp_W25Q128.h
│       └── src/  bsp_modbus.c bsp_W25Q128.c
├── Task/                       应用层
│   ├── inc/  task.h io_logic.h led.h alarm.h params.h
│   └── src/  task.c io_logic.c led.c alarm.c params.c
├── Tools/
│   ├── inc/  crc.h
│   └── src/  crc.c            CRC16-Modbus
├── MDK-ARM/                   Keil 工程 + 构建产物（.o/.axf/.hex/.map 等）
├── SCH_NET-RELAY.ioc          CubeMX 工程（外设/引脚配置源）
└── .mxproject                 CubeMX 辅助文件
```

### 3.3 Keil 分组

| 分组 | 内容 |
|---|---|
| Application/MDK-ARM | startup_stm32f407xx.s |
| Application/User/Core | main.c gpio.c dma.c tim.c usart.c stm32f4xx_it.c stm32f4xx_hal_msp.c |
| Application/User/Task | task.c io_logic.c led.c alarm.c params.c |
| Drivers/STM32F4xx_HAL_Driver | HAL 各外设 |
| Drivers/CMSIS | system_stm32f4xx.c |
| Drivers/BSP | bsp_modbus.c bsp_W25Q128.c |
| Tools | crc.c |

### 3.4 macOS 开发（VS Code + Keil Studio + CMSIS-Toolbox）

> 不依赖 Windows/Keil MDK，整套工具链在 macOS 上原生编译、烧录、调试。

**工具链（本机已配置）**

- **IDE**：VS Code + Arm Keil Studio 扩展（`Arm.keil-studio-pack`，含 CMSIS Solution / CMSIS Debugger / Environment Manager / clangd）
- **工程**：CMSIS-Toolbox csolution（`csolution` / `cbuild` / `cpackget` 2.15.0）
- **编译器**：Arm GNU Toolchain 15.2.1（`arm-none-eabi-gcc`，带 newlib）
- **调试/烧录**：SEGGER J-Link 软件 V9.80（`JLinkExe` / `JLinkGDBServer`）+ `arm-none-eabi-gdb`
- **器件包**：`GigaDevice.GD32F4xx_DFP 3.0.3`、`Keil.STM32F4xx_DFP 3.1.1`

**工程文件（csolution）**

| 文件 | 说明 |
|---|---|
| `SCH_NET-RELAY.csolution.yml` | solution：目标 GD32F407VG，GCC，Debug 构建 |
| `SCH_NET-RELAY.cproject.yml` | source groups / 包含路径 / 宏 / 链接器 |
| `SCH_NET-RELAY.ld` | GNU 链接脚本（FLASH 1MB / RAM 128KB / CCM 64KB，含 `.RamFunc`） |
| `build.sh` | 编译（导出工具链环境 + `cbuild`） |
| `flash.sh` | J-Link 烧录 HEX |
| `debug.sh` | J-Link GDB Server + GDB 调试 |

**操作**

```bash
./build.sh      # 编译，产物在 out/SCH_NET-RELAY/Debug/Debug/
./flash.sh      # 烧录（需先 build），J-Link 按 STM32F407 算法写 GD32F407VG
./debug.sh      # GDB 调试（也可在 VS Code 用 Keil Studio CMSIS view 调试）
```

**说明**

- 目标器件为 GD32F407VG；烧录/调试按 STM32F407 处理（寄存器/Flash 兼容，实测 IDCODE 低 12 位 `0x413`）。
- 编译器用开源的 Arm GNU GCC，免许可证；若要换 Arm Compiler 6（AC6），需激活免费 MDK-Community 许可证。
- 启动文件复用仓库内 STM32F4xx 的 GCC 版 `startup_stm32f407xx.s`。

---

## 4. 软件架构

### 4.1 运行模型

裸机超循环，无 RTOS。TIM2 产生 **1ms** 中断节拍（预分频 83、周期 999）。

```c
main()
 ├─ HAL_Init / SystemClock_Config
 ├─ MX_GPIO_Init / MX_DMA_Init / MX_TIM2_Init
 ├─ MX_USART1/3/6_UART_Init
 ├─ Alarm_Init / Modbus_Init / LED_Init / Task_Init / W25Q128_Init
 └─ while(1)
     ├─ IO_Update()      // 读输入 → 写输出 → Modbus 寄存器映射（状态→寄存器）
     ├─ Task_Update()    // 急停状态机
     ├─ Alarm_Update()   // 报警检测
     ├─ LED_Update()     // LED 逻辑
     └─ Params_Update()  // 参数变更检测 → 写 Flash
```

**注意**：`Task_Init()` 依赖在 IO_Update() 之后调用（需读急停电平），当前 main.c 中顺序为 Task_Init 在 W25Q128_Init 之前，而上电第一次 IO_Update 在循环里才执行——Task_Init 读到的 g_io.input 是初值（全 0），建议后续开发时在 main 中显式先调一次 IO_Update() 再 Task_Init()。

### 4.2 模块职责

| 模块 | 文件 | 职责 |
|---|---|---|
| IO 逻辑 | io_logic.c | 输入采集、输出映射、Modbus 寄存器映射（唯一 I/O 出入口） |
| 任务状态机 | task.c | ESTOP_PRESSED → ESTOP_RELEASED → NORMAL 三态迁移 |
| 报警 | alarm.c | 急停双路不一致（10ms 超时）报警、Flash 故障标志；reset 清零 |
| LED | led.c | system 灯=正常常亮；working 灯=500ms 闪烁；network 灯未用 |
| 参数 | params.c | 掉电保持参数，magic 校验，变更即写（shadow 对比） |
| Modbus | bsp_modbus.c | 双串口 RTU 从站，03/06/10，异常响应，IDLE+DMA 收，DMA 发 |
| Flash | bsp_W25Q128.c | 软件 SPI（Mode 3），读/写/擦除/SafeWrite，上电自检 |
| CRC | crc.c | Modbus CRC16（poly 0xA001，init 0xFFFF，小端） |

### 4.3 中断资源清单

| 中断 | 触发 | 处理内容 |
|---|---|---|
| EXTI3（PB3） | 急停1 上升沿 | Task_EStopISR() → 状态机直接置 ESTOP_PRESSED |
| EXTI4（PB4） | 急停2 上升沿 | Task_EStopISR() |
| TIM2 | 1ms | Modbus_TimerTick()：递减各串口帧静默计时 |
| USART1 | IDLE | 计算 DMA 接收长度 → Modbus_RxHandler() |
| USART3 | IDLE | 同上 |
| USART6 | — | 仅 HAL_UART_IRQHandler，**未启用 IDLE 处理**（预留） |
| DMA1_Stream1/3 | USART3 RX/TX | HAL DMA 处理 |
| DMA2_Stream1/2/6/7 | USART6 RX / USART1 RX / USART6 TX / USART1 TX | HAL DMA 处理 |

所有业务中断优先级均配置为 0（抢占组 NVIC_PRIORITYGROUP_4），SysTick 为 15。

---

## 5. 引脚分配表

### 5.1 输入

| 引脚 | 信号 | 极性 | 备注 |
|---|---|---|---|
| PB3 | 急停1 (estop1) | 高=按下 | EXTI3 上升沿 |
| PB4 | 急停2 (estop2) | 高=按下 | EXTI4 上升沿 |
| PE1 | 复位键 (reset) | 低有效，读取反相：1=按下 | 轮询 |
| PC12 | PNP_in[0] | 低=有效，读取反相 | 轮询 |
| PD0~PD2 | PNP_in[1..3] | 同上 | 轮询 |
| PD4~PD7 | PNP_in[4..7] | 同上 | 轮询 |

### 5.2 输出

| 引脚 | 信号 | 极性 | 备注 |
|---|---|---|---|
| PD9 | relay[0] | 高=吸合 | |
| PE14 | relay[1] | 高=吸合 | |
| PD8 | relay[2] | 高=吸合 | |
| PE15 | relay[3] | 高=吸合 | |
| PE13 | PNP_out[0] | 低=导通（读取反相） | 目前由 LED 逻辑驱动 = system 灯 |
| PE12 | PNP_out[1] | 同上 | 目前 = PNP_out[0] 取反 |
| PE11 | PNP_out[2] | 同上 | 未使用 |
| PE10 | PNP_out[3] | 同上 | 未使用 |
| PE4 | LED working | 高=亮 | 正常时 500ms 闪烁 |
| PE3 | LED network | **低=亮**（反向） | 未使用，恒灭 |
| PE2 | LED system | 高=亮 | 正常时常亮 |

### 5.3 通信 / 存储

| 引脚 | 信号 |
|---|---|
| PA9/PA10 | USART1_TX/RX（Modbus 从站①） |
| PC10/PC11 | USART3_TX/RX（Modbus 从站②） |
| PC6/PC7 | USART6_TX/RX（预留，未用） |
| PA13/PA14 | SWDIO/SWCLK（调试） |
| PD12 | W25Q128 CS（软件 SPI） |
| PD13 | W25Q128 SCLK（软件 SPI，Mode 3，空闲高） |
| PD14 | W25Q128 MISO（上拉输入） |
| PD15 | W25Q128 MOSI（软件 SPI） |
| PH0/PH1 | HSE 8MHz |
| PC14/PC15 | LSE 32.768kHz（已配未用） |

---

## 6. Modbus 寄存器表（从站地址 1）

- 保持寄存器数量：50（MODBUS_REG_NUM）
- 支持功能码：**03** 读保持寄存器 / **06** 写单寄存器 / **10** 写多寄存器
- 异常码：01 非法功能 / 02 非法地址 / 03 非法数据值
- 帧间隔静默：4ms（TIM2 递减，115200bps 下 3.5T≈0.3ms）
- 双口同时服务：USART1 + USART3，同一从站地址

| 寄存器 | 含义 | 方向 |
|---|---|---|
| 0 | 任务状态（1=正常） | 只读（映射覆盖） |
| 20 | 急停1 | 只读 |
| 21 | 急停2 | 只读 |
| 22 | 复位键 | 只读 |
| 23~30 | PNP_in[0..7] | 只读 |
| 31~34 | relay[0..3] | ✅ 上位机可写（反向映射，钳位 0/1，当周期生效） |
| 35~37 | LED working/network/system | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 38~39 | PNP_out[0..1] | ⚠️ 映射覆盖（LED 逻辑主导，回读实际值） |
| 40~41 | PNP_out[2..3] | ✅ 上位机可写（反向映射，钳位 0/1，当周期生效） |
| 其余 (1~19, 42~49) | 空闲可写 | 读写（不参与映射，写后保留） |

---

## 7. Flash 存储规划（W25Q128，16MB）

| 地址 | 用途 |
|---|---|
| 0xFFF000（最后 4KB 扇区） | 上电自检区（写入 "Geekplus" 并回读比对） |
| 0xFFE000（倒数第二扇区） | 掉电保持参数区（Params_t，magic=0xA55A0001） |
| 其余空间 | 空闲（可用于日志/配方等） |

参数结构（params.h）：

```c
typedef struct {
    uint32_t magic;   /* 有效性校验 0xA55A0001，勿手动修改 */
    uint32_t test;    /* 示例字段 */
    /* 新增字段在此添加；增删字段后需将 magic 版本号 +1 */
} Params_t;
```

**要点**：Params_Update 每次变更做"读扇区→改→擦→写"整扇区操作，主循环内同步执行，耗时 45~400ms 阻塞。

---

## 8. 已知问题 / 待开发点（重要）

1. ~~**Modbus 写输出链路未打通**~~ ✅ **已解决（2026-08-15）**
   - `io_logic.c` 新增 `modbus_write_mapping()` 反向映射：reg 31~34 → relay[0..3]、reg 40~41 → PNP_out[2..3]，值钳位 0/1；
   - `IO_Update()` 内部顺序调整为 读输入 → 映射（正向+反向）→ 写引脚，上位机写入当周期生效；
   - LED（35~37）与 PNP_out[0..1]（38~39）仍由 LED 逻辑主导，上位机写会被覆盖（避免与 LED 状态机打架），如需放开需先修改 led.c。

2. ~~**Task_Init 时序**~~ ✅ **已解决（2026-08-15）**
   - `main.c` 中 `Task_Init()` 之前显式调用一次 `IO_Update()`，上电即可读到真实急停电平。

3. **USART6 已配置未用**：引脚 PC6/PC7 + DMA 已就绪，可作调试口/第三路通信/上位机协议。

4. **network LED（PE3）未使用**：低有效，当前恒灭；"NET"相关功能（如以太网状态）就绪后接入。

5. **Flash 写入阻塞主循环**：参数变更/日志存储频繁时需优化（脏扇区只擦、写次数限制、空闲时段写入等）。

6. **GD32F407 特性未利用/需验证**：
   - 当前按 STM32F407 配置跑 168MHz，GD32F407 最高支持 200MHz；
   - 外设基本兼容，但 RCC 等寄存器存在细微差异，上新外设前建议用 GD32F4xx 官方库或至少做差异验证；
   - 若上以太网（项目名含 NET），GD32F407 自带 10/100M MAC，需 RMII PHY + LWIP，当前无任何以太网配置。

7. **无版本控制**：目录无 .git，建议开发前初始化并清理 MDK-ARM 构建产物。

8. **参数写入策略**：当前每次变更立即写 Flash（无防抖/合并），上位机高频改参时磨损风险需评估。

---

## 9. 变更记录

| 日期 | 变更内容 | 备注 |
|---|---|---|
| 2026-09-30 | 新增 macOS 构建（csolution/CMSIS-Toolbox + ARM GNU GCC）；USART2 作为第三路 Modbus RTU（ESP32 接 PA2/PA3）；LED 映射改 PE2/PE3/PE4/PE5；新增 build/flash/debug 脚本与硬件资源文档 | 版本维护 |
| （首次盘点） | 通读全部源码，建立基线记录 | 尚未开始新需求开发 |
| 2026-08-15 | 打通 Modbus 写输出链路（反向映射 reg 31~34/40~41 → 输出，钳位 0/1，当周期生效） | 见第 6 节、第 8.1 条 |
| 2026-08-15 | 修复 Task_Init 时序（main 中先调 IO_Update 再 Task_Init） | 见第 8.2 条 |
| 2026-08-15 | Keil 工程器件从 STM32F407VGTx 切换为 GD32F407VG（GigaDevice.GD32F4xx_DFP 3.0.3，本机已装），修复 Error #550；烧录算法改 GD32F4xx_1MB.FLM，SVD 改 GD32F4xx.svd | VS Code 开发环境搭建 |
