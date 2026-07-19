# ADC1 迁移至 PA6/INP3 设计

## 目标

将 STM32H743VIT6 的 ADC1 规则通道从 PC4/ADC1_INP4 迁移到 PA6/ADC1_INP3，同时保持现有 ADC1/ADC2 双重规则同步采集、TIM2 TRGO 触发、DMA 搬运和 FFT 数据通路不变。

## 可行性结论

STM32H743VIT6 的 PA6 支持 ADC12_INP3，工程中 PA6 在迁移前未被其他外设占用。用户已确认 PA6 已接入目标模拟信号、没有板载资源冲突，且信号满足模拟输入电气条件。ADC12_INP3 与原 ADC12_INP4 均为快速 ADC 通道，因此此次迁移不降低现有 600 kS/s 采集目标。

## CubeMX 配置

- 在 Pinout & Configuration 中释放 PC4 的 ADC1_INP4，并将 PA6 设置为 ADC1_INP3、IN3 Single-ended。
- ADC1 Regular Conversion Rank 1 使用 Channel 3，采样时间保持 8.5 cycles。
- ADC1 保持 16-bit、Scan Disabled、一次规则转换、TIM2 TRGO Rising Edge、DMA Circular 和 Overrun Data Overwritten。
- ADC1/ADC2 保持 Dual Regular Simultaneous；ADC2 继续使用 PB1/ADC2_INP5。
- ADC1 DMA 保持 DMA1 Stream0、Peripheral-to-Memory、Word/Word、Memory Increment、Circular、Very High Priority。
- ADC 和 DMA 中断设置不因迁移而改变。

## 代码与文档修改

CubeMX 重新生成 `h743_pre1.ioc` 和 `Core/Src/adc.c`，不手工维护生成区。用户代码仅更新模块头部和相关中文注释中的 ADC1 引脚映射，不改变 DMA 缓冲格式、回调标志、数据拆包、FFT 输入顺序或主循环调用关系。README 更新硬件连接、CubeMX 操作、采集链路、验证方法和电气限制。

## 数据流与错误处理

TIM2 TRGO 同时触发 ADC1 PA6/INP3 与 ADC2 PB1/INP5。ADC1 仍作为双模式主 ADC，通过 DMA1 Stream0 读取公共数据寄存器；每个 32 位字低 16 位为 ADC1/CH1，高 16 位为 ADC2/CH2。现有 DMA 半满、满和 ADC 错误回调只设置各自标志，错误统计与恢复继续在主循环处理。

## 验证

- 静态检查 `.ioc` 中 PA6/ADC1_INP3、Channel 3、单端模式及 PC4/INP4 移除情况。
- 静态检查 `adc.c` 中 ADC_CHANNEL_3、GPIOA Pin 6 的初始化和反初始化。
- 检查 ADC2、TIM2、DMA、NVIC 和双模式参数未发生非预期变化。
- 检查 `main.c` 初始化区和主循环仍分别只调用 `system_init()` 与 `system_process()`。
- 运行现有离线测试并新增迁移契约检查；条件允许时执行 STM32CubeIDE Debug 编译。
- 实板向 PA6 输入符合 VDDA 范围的信号，观察 CH1 原始码、DMA 统计与 FFT 结果；ADC2/CH2 同时验证以确认双通道同步链路未受影响。

## 不在范围内

不调整模拟前端、ADC 时钟、采样率、采样时间、DMA 缓冲长度、FFT 参数、ADC2 引脚或中断业务逻辑。
