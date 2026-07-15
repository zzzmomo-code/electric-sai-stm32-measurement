# 片上双 ADC 的 CubeMX 配置步骤

只在新工程 `h7_onchip_adc.ioc` 中操作。原 `h7_pre` 不打开、不生成代码。

## 1. 导入新工程

1. STM32CubeIDE 选择 `File > Import...`。
2. 选择 `General > Existing Projects into Workspace`。
3. Root directory 选择 `C:\Users\48747\STM32CubeIDE\workspace_1.19.0\h7_onchip_adc`。
4. 不勾选 `Copy projects into workspace`，点击 `Finish`。
5. 双击项目根目录的 `h7_onchip_adc.ioc`。

## 2. 移除 ADS8688 外设

1. `Pinout & Configuration > Connectivity > SPI2`，Mode 选择 `Disable`。
2. 打开 `System Core > DMA`，确认 SPI2_RX 和 SPI2_TX 请求已经消失；如仍存在则删除。
3. 打开 `System Core > NVIC`，确认 SPI2 global interrupt 已关闭。
4. PB12、PB13、PB14、PB15 应恢复为空闲；PD8、PD9 的旧 ADS8688 GPIO Output 也改为未使用。

不要手改 `spi.c`、`dma.c` 或 `MX_SPI2_Init()`，由 Generate Code 自动更新。

## 3. 配置 ADC 引脚

1. 在 Pinout 图点击 PC4，选择 `ADC1_INP4`。
2. 点击 PB1，选择 `ADC2_INP5`。
3. `Analog > ADC1` 和 `Analog > ADC2` 应显示绿色可配置状态。

## 4. 配置 ADC1 和 ADC2

两个 ADC 的基础参数均设置为：

- Resolution：`16 Bits`。
- Data Alignment：`Right alignment`。
- Conversion Data Management：按后续 Multimode DMA 配置。
- Scan Conversion Mode：Disable。
- Continuous Conversion Mode：Disable。
- Discontinuous Conversion Mode：Disable。
- Overrun：`Data Overwritten`。
- Oversampling：Disable。
- 输入模式：Single-ended。
- Number Of Conversion：1。
- Rank 1 Sampling Time：`8.5 Cycles`。

ADC1 Rank 1 选择 Channel 4；ADC2 Rank 1 选择 Channel 5。

## 5. 配置双 ADC 同步模式

1. 在 ADC1 的 Multimode 设置中选择 `Dual Regular Simultaneous`。
2. ADC1 为 Master，ADC2 为 Slave。
3. DMA Access Mode/Data Format 选择 32-bit packed 数据，要求 ADC1 位于低 16 位、ADC2 位于高 16 位。
4. Two Sampling Delay 保持满足当前采样时序的最小合法值；生成后由代码检查实际宏值。
5. 再次确认两个 ADC 的 Overrun 都是 `Data Overwritten`，即 `OVRMOD=1`。

H750 勘误要求单 DMA 读取 Dual Regular Simultaneous 时启用覆盖模式，避免 ADC2 数据错位。

## 6. 配置 TIM2 为 80 kHz 触发源

1. `Timers > TIM2`，Clock Source 选择 `Internal Clock`。
2. Prescaler 设置 `0`。
3. Counter Period 设置 `2999`。
4. `Master/Slave Mode` 中 Trigger Output (TRGO) 选择 `Update Event`。
5. 当前时钟树 APB1=120 MHz，定时器时钟为 240 MHz，因此触发率为：

```text
240 MHz / (0 + 1) / (2999 + 1) = 80 kHz
```

若 Clock Configuration 页面中的 TIM2 clock 不是 240 MHz，先停止生成并按 `ARR = TIM2时钟 / 80000 - 1` 重新计算，不要照抄2999。

80 kSPS 配合8192点FFT时，本征频点间隔为 `80000 / 8192 = 9.765625 Hz`，奈奎斯特频率为40 kHz。该模式满足最高20 kHz基波和10 Hz内频点间隔，但不保证20 kHz输入的高次谐波与THD测量。

## 7. 配置触发与 ADC 时钟

1. ADC1 External Trigger Conversion Source 选择 `TIM2 TRGO`。
2. ADC1 External Trigger Edge 选择 `Rising Edge`。
3. ADC2 不配置独立外部触发，由 Dual Regular Simultaneous 模式跟随 ADC1。
4. 在 `Clock Configuration` 中确认 ADC Kernel Clock 来源为 `PER_CK`，当前 `PER_CK` 为 64 MHz。
5. ADC1 和 ADC2 的 Clock Prescaler 均选择 `Asynchronous clock mode divided by 4`，实际 ADC 内核时钟为 16 MHz。

## 8. 配置 ADC1 DMA

ADC1 的 `DMA Settings` 添加请求：

- Instance：`DMA1 Stream0`。
- Direction：Peripheral to Memory。
- Mode：Circular。
- Peripheral Increment：Disable。
- Memory Increment：Enable。
- Peripheral Data Width：Word。
- Memory Data Width：Word。
- Priority：Very High。

ADC2 不添加独立 DMA。

在 `NVIC Settings` 中同时启用：

- `DMA1 Stream0 global interrupt`，Preemption Priority 设为 `5`。
- `ADC1 and ADC2 global interrupt`，Preemption Priority 设为 `5`，Sub Priority 设为 `0`。

双 ADC DMA 启动时 HAL 会启用 ADC overrun 错误中断，因此必须保留 ADC1/ADC2 全局中断用于错误处理。

## 9. 保持 USART1 不变

- PA9：USART1_TX。
- PA10：USART1_RX。
- 9600、8 Bits、None、1 Stop Bit、No Flow Control。
- USART1 DMA Settings 为空。
- USART1 global interrupt 不启用。

## 10. 生成和交接检查

1. 先保存 `.ioc`。
2. 选择 `Project > Generate Code`。
3. 若弹出保留用户代码选项，确认保留 `USER CODE BEGIN/END` 区域。
4. 生成后不要手改 `MX_ADC1_Init()`、`MX_ADC2_Init()`、`MX_TIM2_Init()` 或 DMA 初始化。
5. 把 CubeMX 的 ADC1、ADC2、TIM2、DMA 和 Clock Configuration 截图发给我检查。
6. 我将检查 `.ioc`、`adc.c/.h`、`tim.c/.h`、`dma.c`、MSP、IRQ、`main.c` 初始化顺序和最终编译。
