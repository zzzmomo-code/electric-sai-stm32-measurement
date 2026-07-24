# STM32H743 单/双信号识别、重建与锁相输出

本工程把 `tongw60536/EDU_Work` 仓库中 `2023H` 的关键输入、频率识别、波形分类、Q32 NCO/PLL 和双 DAC 输出思路移植到 **STM32H743VIT6**，并增加单信号重建模式。工程使用 STM32CubeIDE 1.19.0，时钟源仅使用芯片内部 HSI。

本次只修改了：

`C:\Users\48747\STM32CubeIDE\workspace_1.19.0\phase_locking_ported_codex`

旧的失败工程 `phase_locking_codex` 没有被用于承载代码，也没有被修改。

## 锁相思路快速说明

本工程把“首次判频”和“持续锁相”分成两个阶段：

1. 默认先连续采集 32768 点，经过直流去除、Hann 窗、FFT 峰值插值和前后
   半段相位斜率，得到非 5 kHz 栅格的毫赫兹格式频率初值。
2. 初值直接转换成 Q32 NCO 相位步进，不重新取整成整数 Hz。
3. 运行时每 512 个 ADC 样点做一次带直流项的最小二乘测相；双信号在同一帧
   联合求解四个正余弦系数，避免任意双音相互串扰。
4. PI 型数字 PLL 根据相位误差微调 NCO 步进；积分器带泄漏和限幅，最终频率
   修正限制为名义步进的约 ±2%。
5. DAC 按绝对样点编号预测未来半缓冲区的相位，因此软件处理延迟不会让输入、
   输出的相对相位持续漂移。

锁相公式、具体函数、单/双通道差异和 Debug 变量见：

- [docs/PHASE_LOCKING.md](docs/PHASE_LOCKING.md)

## 1. 实现的功能

### 1.1 工作模式超参数

工作模式在 `Core/User/signal_separation_config.h` 中选择：

```c
#define SIGSEP_MODE_DUAL_MIXED  1U
#define SIGSEP_MODE_SINGLE      2U

#define SIGSEP_OPERATION_MODE   SIGSEP_MODE_DUAL_MIXED
```

修改 `SIGSEP_OPERATION_MODE` 后必须重新编译并下载。当前默认值为
`SIGSEP_MODE_DUAL_MIXED`，先验证混合输入分离、PA4/PA5 双路重建和锁相；
若双信号测试异常，再切换到单信号模式逐级排查。

| 模式 | PC0 输入 | PA4 | PA5 |
|---|---|---|---|
| `SIGSEP_MODE_SINGLE` | 一路干净的正弦波、三角波或方波 | 重建并锁相输出 | 保持 DAC 中点，约 1.65 V |
| `SIGSEP_MODE_DUAL_MIXED` | 两路信号经过模拟加法器后的混合波形 | 较低频分量 | 较高频分量 |

#### 在 STM32CubeIDE 中切换模式

模式不在 `.ioc` 中选择，也不能通过按复位键切换。它是一个编译期宏，每次修改后
都必须重新编译并下载。具体操作如下：

1. 在左侧 **Project Explorer** 展开工程
   `phase_locking_ported_codex`。
2. 依次展开 **Core → User**，双击打开
   `signal_separation_config.h`。
3. 在文件顶部找到 `SIGSEP_OPERATION_MODE`。只修改这一行，不要修改
   `SIGSEP_MODE_DUAL_MIXED` 和 `SIGSEP_MODE_SINGLE` 的数值。
4. 测试两路混合输入、PA4/PA5 双路输出时写成：

   ```c
   #define SIGSEP_OPERATION_MODE SIGSEP_MODE_DUAL_MIXED
   ```

5. 测试一路干净输入、PA4 单路输出时写成：

   ```c
   #define SIGSEP_OPERATION_MODE SIGSEP_MODE_SINGLE
   ```

6. 按 **Ctrl+S** 保存。
7. 在菜单栏依次选择 **Project → Clean...**，勾选
   `phase_locking_ported_codex` 后点击 **Clean**。
8. 选中左侧的 `phase_locking_ported_codex`，点击工具栏锤子图标，或选择
   **Project → Build Project**。在底部 **Console** 确认出现
   `Build Finished`，并且为 `0 errors`。
9. 点击绿色运行按钮下载，或点击小甲虫进入 Debug；下载完成后再让程序继续运行。
10. 打开串口观察启动信息：
    - `mode=dual_mixed, low=PA4, high=PA5` 表示双信号模式；
    - `mode=single, signal=PA4, PA5=midscale` 表示单信号模式。

当前源码默认使用 `SIGSEP_MODE_DUAL_MIXED`，且
`SIGSEP_COMMON_SOURCE_LOCK=0U`，因此 PA4、PA5 分别闭环锁定低频和高频输入。
开发板实际运行的模式以最后一次编译下载时的宏为准；只按复位键不会切换模式。

### 1.2 频率识别模式

频率识别模式也在 `Core/User/signal_separation_config.h` 中选择：

```c
#define SIGSEP_FREQ_MODE_GRID_5KHZ   1U
#define SIGSEP_FREQ_MODE_CONTINUOUS  2U
#define SIGSEP_FREQ_MODE_PRECISE_FFT 3U

#define SIGSEP_FREQUENCY_MODE SIGSEP_FREQ_MODE_PRECISE_FFT
```

| 频率模式 | 用途 | 识别结果 |
|---|---|---|
| `SIGSEP_FREQ_MODE_GRID_5KHZ` | 完整保留原题和已实板验证方案 | 只识别 10～100 kHz 范围内的 5 kHz 整数倍 |
| `SIGSEP_FREQ_MODE_CONTINUOUS` | 低内存、快速任意频率方案 | 5 kHz 粗搜索、4096 点连续记录和 250 Hz 细搜索 |
| `SIGSEP_FREQ_MODE_PRECISE_FFT` | 默认高精度首次判频方案 | 32768 点连续记录、Hann 窗 FFT、三点对数谱峰插值和前后半段相位斜率细化，结果保留到 0.001 Hz |

当前源码默认使用 `SIGSEP_FREQ_MODE_PRECISE_FFT`。它连续采样约 13.1072 ms，
FFT 原始频点间隔约 76.2939 Hz，再用峰顶插值消除“只能落在整数频点”的限制。
默认搜索范围为 1～250 kHz，范围宏为
`SIGSEP_PRECISE_FREQ_MIN_HZ` 和 `SIGSEP_PRECISE_FREQ_MAX_HZ`。
如果高精度模式实板测试出现问题，
只需把 `SIGSEP_FREQUENCY_MODE` 改回 `SIGSEP_FREQ_MODE_GRID_5KHZ`，Clean/Build
并重新下载，即可恢复之前已经锁住的算法，不需要回退代码。

串口启动信息会明确显示：

- `frequency=grid_5khz`：原 5 kHz 栅格方案；
- `frequency=continuous, coarse=5000Hz, fine=250Hz+PLL`：快速任意频率方案；
- `frequency=precise_fft, N=32768, Hann+peak+phase`：默认高精度方案。

### 1.3 公共数据链路

1. TIM2 以 2.5 MHz 产生 TRGO，同时触发 ADC1 和 DAC1 两个通道。
2. ADC1 通过 PC0 采集单信号或混合信号，16 位、DMA 循环双半区。
3. 每个 DMA 半区包含 512 点；完成后先把最新安全半区复制到 CPU
   专用缓冲区，再执行分析，避免计算期间被下一轮 DMA 覆写。
4. 原模式在 10 kHz～100 kHz 范围内，以 5 kHz 为间隔检查 19 个候选频点，
   并对 4 帧幅值求平均。
5. 快速连续频率模式先用相同的 19 点粗搜索定位主峰，同时收集 8 个完整 DMA 半区；
   然后在粗峰附近按 250 Hz 搜索并做三点抛物线插值。双信号模式会屏蔽第一个
   主峰附近的谱泄漏，再寻找第二分量。
6. 默认高精度模式收集 32768 个连续样点，去直流后加 Hann 窗，执行基 2 FFT，
   在设定频段内寻找局部主峰并用相邻三点的对数功率做抛物线插值，再比较记录
   前后两半的基波相位斜率细化频率。双信号模式还会用 4 kHz 保护区避免重复
   选择同一主峰。
7. 双信号题目模式与参考工程一样根据三次/五次谐波区分正弦波和三角波；单信号
   扩展模式额外支持方波分类与重建。
8. 插值得到的毫赫兹初值直接换算为 Q32 NCO 步进；之后仍使用原 Q32 NCO 和 PI
   型数字 PLL 跟踪输入相位及频率误差，锁相环参数没有因判频升级而改变。HSI 场景的软件
   捕获范围设为约 ±2%。
9. 单信号用 2×2 中心化最小二乘测幅/测相；双信号用 4×4 联合最小二乘同时
   求两路幅相，解决低频不足一个周期和任意双音非正交造成的测相偏差。测相后
   仍按原流程执行 PI 型 PLL、Q32 NCO 状态更新和幅值平滑；单信号只更新通道 0
   并输出 PA4，PA5 始终回填中点值。
10. 双信号默认使用两个独立 PLL：PA4 持续测量并锁定低频输入，PA5 持续测量并
   锁定高频输入。只有把 `SIGSEP_COMMON_SOURCE_LOCK` 改为 `1U` 时，才切换为
   GitHub 参考工程的同源主从锁相。PA5 还可额外叠加默认 150° 的输出相位。

## 2. 引脚连接

| 功能 | H743 引脚 | 外部连接 |
|---|---|---|
| 单信号/混合输入 | PC0 / ADC1_INP10 | 函数发生器或模拟加法器输出 |
| 主输出/低频输出 | PA4 / DAC1_OUT1 | 示波器 CH1 |
| 中点/高频输出 | PA5 / DAC1_OUT2 | 示波器 CH2 |
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
   - DIVP2：`10`
9. ADC Clock Mux 选择 `PLL2P`，时钟树显示 78.4 MHz。

本板芯片 Revision ID 为 **Rev.V**。H743 HAL 会在 Rev.V 的 ADC 路径内部再除以 2，
所以 ADC 实际工作时钟为 39.2 MHz。若把 DIVP2 恢复成 16，时钟树虽然显示
49 MHz，但 ADC 实际只有 24.5 MHz，无法接收全部 2.5 MHz 外部触发，会固定漏掉
20% 采样；算法仍按 2.5 MSPS 换算时就会出现频率偏高。

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
- Sampling Time：1.5 Cycles
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
├─ frequency_estimator.h          高精度首次判频接口
├─ frequency_estimator.c          32768 点 FFT、插值和相位斜率细化
├─ uart_debug.h
└─ uart_debug.c                   启动信息和一次性非阻塞锁定结果

docs/
└─ PHASE_LOCKING.md               锁相公式、代码入口和 Debug 指南

STUDY_NOTES.md                     移植差异、关键架构和学习总结
```

所有 ADC/DAC HAL DMA 回调只递增一个对应的 32 位单调事件标志；耗时处理全部
在主循环完成。主循环按事件代次重建绝对样点时间，积压时只处理当前仍安全的
最新 ADC 半区，不会再把布尔标志折叠后的旧半区当成有效帧。

## 6. 编译

正常连续运行时建议使用 `-O2`，用于满足 2.5 MSPS 下的实时运算需求。若为了单步调试临时改成
`-Og`，断点停住期间 ADC/DAC DMA 仍可能继续运行，继续执行后出现 `adc_drop` 或 `dac_drop`
不代表正常全速运行也会丢帧。

在 CubeIDE 中：

1. 右键工程 `phase_locking_ported_codex`。
2. 选择 **Build Configurations → Set Active → Debug**。
3. 选择 **Project → Clean...**。
4. 点击锤子图标 Build。
5. 成功后应生成：

`Debug\phase_locking_ported_codex.elf`

本次本机完整构建结果见本次交付说明；每次修改工作模式宏后，都应重新 Clean/Build 并确认：

- `0 errors, 0 warnings`
- ELF 已生成
- `.dma_buffer = 0x30000000，大小 0x2800`

本次任意频率扩展还完成了以下软件侧检查：

- 实际 `frequency_estimator.c` 主机测试覆盖 1 kHz、1234.567 Hz、
  10.321789 kHz、99.999730 kHz、249.876400 kHz，以及多组双音和恰好
  相差 4 kHz 的组合；带量化和随机噪声的这些用例误差不超过 0.002 Hz。
- 500 点双音联合最小二乘测试覆盖 1 kHz + 5 kHz、10 kHz + 14 kHz、
  20.3214 kHz + 30.7896 kHz 和 1 kHz + 249 kHz；测试相位误差不超过
  0.086°。
- 原栅格/快速连续/高精度三种判频模式、单/双通道模式和同源主从可选路径均
  通过 `-Wall -Wextra -Werror` 语法编译。

这些是确定性合成数据和交叉编译检查，不替代 H743 + 模拟前端 + 示波器验证。

## 7. 串口输出

上电启动时：

```text
H743 phase locking port
ADC PC0, DAC PA4/PA5, Fs=2500000Hz
mode=dual_mixed, low=PA4, high=PA5
frequency=precise_fft, N=32768, Hann+peak+phase
command: r=restart identify
state=search
```

单信号模式首次识别完成时：

```text
locked A=20000.000Hz/sin adc_drop=0 dac_drop=0
```

双信号模式启动行和首次识别结果示例：

```text
mode=dual_mixed, low=PA4, high=PA5
locked A=20000.000Hz/tri B=30000.000Hz/tri adc_drop=0 dac_drop=0
```

锁定结果使用 USART1 中断发送，不阻塞实时采样。输入频率或接线改变后，可通过
USART1 发送字符 `r` 或 `R` 重新识别；这对应参考工程串口屏上的“分离”按键，
不必重新下载固件。

## 8. 第一次实板验证顺序

### 8.1 先验证默认双混合信号模式

1. 先不接 PC0，完成下载并确认程序没有进入 `Error_Handler()`。
2. 确认串口启动行显示 `mode=dual_mixed`；若不是，检查
   `SIGSEP_OPERATION_MODE` 是否为 `SIGSEP_MODE_DUAL_MIXED`，然后重新编译、下载。
3. 两路发生器信号必须先经过外部模拟加法器，再把**加法器的单路输出**接到 PC0；
   不要把两个推挽信号源输出端直接短接。
4. 先使用两个相差较大的 5 kHz 栅格频率，例如 20 kHz 和 55 kHz，并确保叠加后
   PC0 始终位于 0～3.3 V。
5. 观察 PA4/PA5，确认分别输出较低和较高频率。
6. 观察串口 `adc_drop` 和 `dac_drop` 是否保持 0。
7. 用示波器分别测输入分量与对应输出的相位差，确认 PLL 收敛且无持续漂移。
8. 相位功能使用同一时基生成的整数倍频组合（例如 20 kHz 和 60 kHz），依次设置
   0°、90°、150°、180°，测量 PA5 相对 PA4 的初相位差。
9. 再测试正弦波、三角波、方波分类、不同幅度组合和 10～100 kHz 边界。

### 8.2 双信号异常时退回单信号模式

1. 把 `SIGSEP_OPERATION_MODE` 改为 `SIGSEP_MODE_SINGLE`，Clean/Build 后重新下载。
2. 函数发生器只启用一路，建议先设置正弦波、20 kHz、0.5 Vpp、DC Offset
   1.65 V，并选择 High-Z 显示模式。
3. 函数发生器、开发板、示波器共地后，先用示波器探头直接测 **PC0 引脚处**，
   确认频率正确，且最低电压不低于 0 V、最高电压不高于 3.3 V。
4. 观察 PA4，应输出识别到的同类波形；PA5 应保持约 1.65 V 直流中点。
5. 串口应报告类似 `locked A=20000.000Hz/sin`，并观察 `adc_drop` 和 `dac_drop`
   在正常全速运行时是否保持 0。
6. 示波器同时观察 PC0 和 PA4，连续观察 30 秒以上；相位差可以是固定常量，
   但不应持续单向漂移。
7. 高精度模式再依次测试三角波、方波，以及 1.3 kHz、10.3 kHz、17.8 kHz、23.4 kHz、
   51.7 kHz、99.6 kHz 等非 5 kHz 整数倍频率；原栅格模式仍测试
   10 kHz、15 kHz、25 kHz、50 kHz 和 100 kHz。

## 9. 当前限制

- 已完成 IOC、源码、完整编译、下载校验和真实 H743 Rev.V 板上寄存器/DMA/锁相
  状态验证；单信号模式的 PC0 输入与 PA4 重建输出已由用户用示波器确认锁相且无
  持续相位漂移。双信号模式下 PA4/PA5 的最终模拟幅值、波形质量和相位仍需继续
  用示波器完整确认。
- 输入没有硬件保护，接线和电压范围必须由操作者保证。
- 高精度任意频率扩展仍需在真实 H743 上逐点验证；
  原 `SIGSEP_FREQ_MODE_GRID_5KHZ` 方案保持不变，可随时切回。
- 高精度模式默认识别范围为 1～250 kHz；2.5 MSPS 的理论奈奎斯特上限为
  1.25 MHz，因此不存在不受采样率和模拟带宽限制的“任意实数频率”。可以修改
  范围宏，但必须保持 `0 < MIN < MAX < 1.25 MHz`；为保证方波/三角波重建仍有
  约 10 点/周期，默认没有把上限推到奈奎斯特边缘。
- 双信号分量建议至少相差 4 kHz；更近时需要更长记录或专门的双音高分辨算法。
- 强三角波/方波的谐波若高于弱信号基波，第二主峰可能选到谐波。若一条谱线
  同时可能是强信号谐波和第二路真实基波，仅靠 PC0 单路混合采样无法无条件
  区分来源；实测应控制幅度比例，或增加额外输入/题目先验。
- 32768 点 FFT 的静态 CPU 缓冲区约占 320 KiB，且识别计算期间 DAC 暂时保持
  中点；FFT 只在启动或收到 `r` 重新识别时运行，不进入锁定后的实时路径。
- 单信号模式只使用 PA4 输出；PA5 保持中点不是故障。
- 输入幅度低于有效阈值后，PLL 会冻结相位和积分状态，避免把噪声相位继续写入
  NCO；程序不会自动重新搜索。改变频率档位后，发送串口字符 `r`/`R`、复位，
  或调用 `signal_separation_restart_identify()`。
- 方波/三角波判别使用三次、五次谐波比例，阈值会受函数发生器带宽、前端失真和
  ADC 噪声影响，仍需实板标定。
- 使用内部 HSI，绝对频率精度和温漂不如外部晶振。软件 PLL 捕获范围已从参考
  工程的约 ±0.05% 扩大到约 ±2%，但捕获、稳态相位误差和环路参数仍须实板确认。
- 默认 `SIGSEP_COMMON_SOURCE_LOCK=0U`，两路 PLL 分别持续测量各自输入相位，
  适用于独立或同源的两路输入。只有明确确认两路来自同一相干时钟、并希望完全
  复现参考工程的主从时间误差传播时，才改成 `1U`。主从模式按主通道幅度门控：
  主通道失效时两路冻结；主通道有效时从通道按频率比继续跟随。
- 双信号模式的 PA5 默认额外偏移 150°；如不需要，修改配置宏或调用
  `signal_separation_set_phase_offset_deg(0)`。
- 20 kHz + 30 kHz 当前实板输入已确认识别正确，ADC:DAC 半缓冲事件比为
  2.000:1，ADC 无 overrun、DAC 无 underrun；同源主从和独立双 PLL 均已确认
  进入闭环且修正量未饱和。模拟幅度精度和示波器相位漂移仍待探头确认。
