# STM32H743 双信号识别与锁相输出

本工程把 `tongw60536/EDU_Work` 仓库中 `2023H` 的关键输入、频率识别、波形分类、Q32 NCO/PLL 和双 DAC 输出思路移植到 **STM32H743VIT6**。工程使用 STM32CubeIDE 1.19.0，时钟源仅使用芯片内部 HSI。

本次只修改了：

`C:\Users\48747\STM32CubeIDE\workspace_1.19.0\phase_locking_ported_codex`

旧的失败工程 `phase_locking_codex` 没有被用于承载代码，也没有被修改。

## 1. 实现的功能

数据链路如下：

1. TIM2 以 2.5 MHz 产生 TRGO，同时触发 ADC1 和 DAC1 两个通道。
2. ADC1 通过 PC0 采集混合信号，16 位、DMA 循环双半区。
3. 每个 DMA 半区包含 512 点；完成后先把最新安全半区的前 500 点复制到 CPU
   专用缓冲区，再执行分析，避免计算期间被下一轮 DMA 覆写。
4. 在 10 kHz～100 kHz 范围内，以 5 kHz 为间隔检查 19 个候选频点。
5. 对 4 帧幅值结果求平均，选出幅值最大的两个分量。
6. 根据三次/五次谐波比例区分正弦波和三角波。
7. 使用 Q32 NCO 和 PI 型数字 PLL 跟踪输入相位及频率误差；HSI 场景的软件
   捕获范围设为约 ±2%。
8. PA4 输出较低频率分量，PA5 输出较高频率分量。
9. 当高频是低频的整数倍时，两路 NCO 使用共同输出时间原点，PA5 相对 PA4
   默认设置 150° 初相位；可通过 API 修改为 0°～180°，不会再叠加输入 A/B
   原有的初相位差。

## 2. 引脚连接

| 功能 | H743 引脚 | 外部连接 |
|---|---|---|
| 混合输入 | PC0 / ADC1_INP10 | 函数发生器输出 |
| 分离输出 A | PA4 / DAC1_OUT1 | 示波器 CH1 |
| 分离输出 B | PA5 / DAC1_OUT2 | 示波器 CH2 |
| 串口发送 | PB14 / USART1_TX | USB 转串口 RX |
| 串口接收 | PB15 / USART1_RX | USB 转串口 TX，可不接 |
| 调试数据 | PA13 / SWDIO | ST-Link SWDIO |
| 调试时钟 | PA14 / SWCLK | ST-Link SWCLK |
| 公共参考 | GND | 函数发生器、示波器、串口和开发板必须共地 |

### 输入安全要求

PC0 前面目前没有运放缓冲、硬件偏置或自动限幅，所以函数发生器必须满足：

- 输出模式建议设为 **High-Z**，不要把 50 Ω 负载幅度误当成实际幅度。
- 波形最低点必须不低于 0 V，最高点必须不高于 3.3 V。
- 对双极性交流信号，建议设置 **1.65 V DC Offset**。
- 建议先从 0.5 Vpp～1.0 Vpp 开始，示波器确认后再增大。
- 函数发生器输出阻抗保持 50 Ω，连接线尽量短，不要让 PC0 悬空。
- 严禁把带负电压的未偏置正弦波直接接到 PC0。

DAC 已开启内部输出缓冲，示波器使用高阻输入。不要直接驱动低阻或大电容负载。

## 3. 当前 IOC 完整配置

打开：

`phase_locking_ported_codex.ioc`

### 3.1 CORTEX_M7 的 I-Cache 和 D-Cache

1. 点击上方 **Pinout & Configuration**。
2. 左侧展开 **System Core**。
3. 点击 **CORTEX_M7**。
4. 中间选择 **Parameter Settings**。
5. 将 **CPU ICache** 设置为 `Enabled`。
6. 将 **CPU DCache** 设置为 `Enabled`。

如果左侧项目较多，可在左上角搜索框输入 `CORTEX_M7`，再点击搜索结果。

### 3.2 时钟树

1. 点击上方 **Clock Configuration**。
2. 左侧 PLL1：
   - PLL Source Mux：`HSI`
   - DIVM1：`4`
   - DIVN1：`60`
   - DIVP1：`2`
3. System Clock Mux 选择 `PLLCLK`。
4. D1CPRE Prescaler：`/1`。
5. HPRE Prescaler：`/2`。
6. D1PPRE、D2PPRE1、D2PPRE2、D3PPRE 全部设为 `/2`。
7. 结果应为：
   - CPU/SYSCLK：480 MHz
   - HCLK：240 MHz
   - APB1/APB2/APB3/APB4：120 MHz
   - APB1/APB2 Timer Clock：240 MHz
8. PLL2 设置：
   - DIVM2：`4`
   - DIVN2：`49`
   - DIVP2：`16`
9. ADC Clock Mux 选择 `PLL2P`，ADC kernel clock 为 49 MHz。

页面右侧看不全时，使用底部横向滚动条；出现红色数字或红点就表示时钟仍有冲突。

### 3.3 TIM2

路径：**Timers → TIM2 → Parameter Settings**

- Clock Source：Internal Clock
- Prescaler：`0`
- Counter Period：`95`
- Counter Mode：Up
- Auto-reload preload：Enable
- Trigger Event Selection / TRGO：Update Event

计算：

`240 MHz / (95 + 1) = 2.5 MHz`

### 3.4 ADC1

路径：**Analog → ADC1**

- PC0：`ADC1_INP10`
- Resolution：16 Bits
- Scan Conversion Mode：Disable
- Continuous Conversion Mode：Disable
- External Trigger Conversion Source：Timer 2 Trigger Out event
- External Trigger Conversion Edge：Rising edge
- Conversion Data Management Mode：DMA Circular Mode
- Overrun behaviour：Overwritten
- ADC Clock Prescaler：Asynchronous clock mode divided by 1
- Rank 1 Channel：Channel 10
- Sampling Time：8.5 Cycles
- Single-ended

ADC DMA：

- DMA Request：ADC1
- DMA1 Stream0
- Direction：Peripheral to Memory
- Mode：Circular
- Peripheral Increment：Disable
- Memory Increment：Enable
- Peripheral/Memory Data Width：Half Word
- Priority：High
- NVIC 抢占优先级：5

### 3.5 DAC1

路径：**Analog → DAC1**

通道 1：

- PA4：DAC1_OUT1
- Trigger：Timer 2 Trigger Out
- Output Buffer：Enable
- DMA1 Stream1
- Mode：Circular
- Peripheral/Memory Data Width：Half Word
- Priority：Very High
- NVIC 抢占优先级：6

通道 2：

- PA5：DAC1_OUT2
- Trigger：Timer 2 Trigger Out
- Output Buffer：Enable
- DMA1 Stream2
- Mode：Circular
- Peripheral/Memory Data Width：Half Word
- Priority：Very High
- NVIC 抢占优先级：7

### 3.6 USART1

路径：**Connectivity → USART1**

- Mode：Asynchronous
- PB14：USART1_TX
- PB15：USART1_RX
- Baud Rate：115200
- Word Length：8 Bits
- Parity：None
- Stop Bits：1
- Hardware Flow Control：None
- NVIC Settings：勾选 `USART1 global interrupt`

CubeMX 生成时 USART1 默认优先级为 0。`uart_debug_init()` 会在运行时把它改为 8，使 ADC/DAC DMA 的优先级更高。

### 3.7 SWD

路径：**System Core → SYS → Debug**

- Debug：Serial Wire
- PA13：SYS_JTMS-SWDIO
- PA14：SYS_JTCK-SWCLK

### 3.8 保存和重新生成

1. 按 `Ctrl+S` 保存 `.ioc`。
2. 等待 **Device Configuration Tool Updating Code** 完成。
3. 所有用户代码均在 `Core\User`。
4. `main.c` 的用户初始化区只有 `system_init();`。
5. `while(1)` 的用户区只有 `system_process();`。
6. 重新生成后检查 `.cproject` 中仍有 `../Core/User` include 路径。
7. 检查 `STM32H743VITX_FLASH.ld` 中仍有 `.dma_buffer >RAM_D2` 段。

## 4. H723 到 H743 的移植差异

### 4.1 CORDIC

原 H723 工程使用硬件 CORDIC 生成正弦表并计算 I/Q 相位。H743 不提供相同的 CORDIC 外设，因此本工程：

- 启动时用 `sinf()`、`cosf()` 生成查找表；
- 实时相位测量用 `atan2f()`；
- 实时 DAC 波形仍使用整数查表和 Q32 相位累加，不会逐点调用三角函数。

### 4.2 DMA 和 D-Cache

H743 的 DMA1 不能访问 DTCM。链接脚本把 ADC/DAC 缓冲区放在 D2 SRAM：

- 起始地址：`0x30000000`
- 段名：`.dma_buffer`
- 当前大小：`0x2800` 字节
- 对齐：32 字节

ADC DMA 写完后，CPU 读取前调用 D-Cache invalidate，并先复制到 CPU 专用分析
缓冲区；CPU 写完 DAC 半区后，DMA 读取前调用 D-Cache clean。

### 4.3 DMA 半区长度

原算法每帧 500 点，正好对应 5 kHz 的频率分辨率。500 个 16 位样点为 1000 字节，不是 32 字节整数倍。

移植后每个 ADC DMA 半区改成 512 点，但只分析前 500 点：

- 保留 5 kHz 正交频点；
- 每半区占 1024 字节；
- 半区边界不会和相邻 Cache 行重叠。

DAC 每半区改为 1024 点，同样保证缓存维护边界安全。

## 5. 代码结构

```text
Core/User/
├─ system.h                       用户代码唯一统一头文件
├─ system.c                       初始化与主循环调度
├─ signal_separation_config.h     采样、识别、PLL 和输出参数
├─ signal_separation.h            对外 API 和状态结构
├─ signal_separation.c            采样、识别、锁相、DAC、DMA/Cache
├─ uart_debug.h
└─ uart_debug.c                   启动信息和一次性非阻塞锁定结果
```

所有 ADC/DAC HAL DMA 回调只递增一个对应的 32 位单调事件标志；耗时处理全部
在主循环完成。主循环按事件代次重建绝对样点时间，积压时只处理当前仍安全的
最新 ADC 半区，不会再把布尔标志折叠后的旧半区当成有效帧。

## 6. 编译

工程的 Debug 配置保留 `-g3`，优化等级设为 `-O2`，用于满足 2.5 MSPS 下的实时运算需求。

在 CubeIDE 中：

1. 右键工程 `phase_locking_ported_codex`。
2. 选择 **Build Configurations → Set Active → Debug**。
3. 选择 **Project → Clean...**。
4. 点击锤子图标 Build。
5. 成功后应生成：

`Debug\phase_locking_ported_codex.elf`

本次本机完整构建结果：

- 编译优化：`-O2`
- `0 errors, 0 warnings`
- ELF 已生成
- `.dma_buffer = 0x30000000，大小 0x2800`

## 7. 串口输出

上电启动时：

```text
H743 phase locking port
ADC PC0, DAC PA4/PA5, Fs=2500000Hz
state=search
```

首次识别完成时：

```text
locked A=25000Hz/sin B=60000Hz/tri adc_drop=0 dac_drop=0
```

锁定结果使用 USART1 中断发送，不阻塞实时采样。

## 8. 第一次实板验证顺序

1. 先不接 PC0，完成下载并确认程序没有进入 `Error_Handler()`。
2. 打开串口，确认看到启动文本。
3. 函数发生器设置：
   - 正弦波；
   - 20 kHz；
   - 0.5 Vpp；
   - DC Offset 1.65 V；
   - High-Z 显示模式。
4. 函数发生器、开发板、示波器共地后，再连接 PC0。
5. 再叠加第二个 5 kHz 整数倍频率，例如 55 kHz。
6. 观察 PA4/PA5，确认分别输出较低和较高频率。
7. 观察串口 `adc_drop` 和 `dac_drop` 是否保持 0。
8. 用示波器测输入与输出相位差，确认 PLL 是否收敛且无持续漂移。
9. 相位功能使用整数倍频组合（例如 20 kHz 和 60 kHz），依次设置
   0°、90°、150°、180°，测量 PA5 相对 PA4 的初相位差。
10. 再测试三角波分类、不同幅度组合和 10～100 kHz 边界。

## 9. 当前限制

- 已完成 IOC、源码和本机编译验证，尚未在真实 H743 板上验证模拟波形。
- 输入没有硬件保护，接线和电压范围必须由操作者保证。
- 候选输入频率按原题思路限定在 5 kHz 栅格；偏离过大时会识别到最近频点。
- 使用内部 HSI，绝对频率精度和温漂不如外部晶振。软件 PLL 捕获范围已从参考
  工程的约 ±0.05% 扩大到约 ±2%，但捕获、稳态相位误差和环路参数仍须实板确认。
- 当前采用同源主从锁相模式，通道 0 是主 PLL。
- 整数倍频相位控制已改为共同输出时间原点；非整数倍频只要求两路稳定同频显示，
  不承诺固定的 A′/B′ 初相位定义。
- PA5 默认额外偏移 150°；如不需要，修改配置宏或调用 `signal_separation_set_phase_offset_deg(0)`。
- 所有“锁定稳定、相位误差、幅度精度”结论均为待硬件验证。
