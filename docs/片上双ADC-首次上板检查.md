# 片上双 ADC 首次上板检查

## 1. 测试边界

- 工程：`h7_onchip_adc`，分支 `momo/onchip-adc`。
- CH1：PC4 / ADC1_INP4。
- CH2：PB1 / ADC2_INP5。
- 两路 ADC 引脚只允许输入 `0 V` 到 `VREF+`；未接前级时禁止输入负电压或高于 3.3 V 的电压。
- 第一次验证使用核心板 GND、3.3 V 和稳定的 1.65 V 附近直流源，不接 ±2.5 V 被测信号。
- 串口屏继续使用 USART1 PA9 TX、9600 8N1、轮询发送。

## 2. 烧录与启动

1. 在 CubeIDE 中选择 `h7_onchip_adc` 项目。
2. 执行 `Project > Clean...`，然后 `Project > Build Project`。
3. 确认控制台为 `0 errors, 0 warnings`。
4. 使用 ST-Link 启动 Debug；停在 `main()` 后按 F8 继续运行。
5. 不要在 ADC DMA 回调或 `HAL_UART_Transmit()` 上保留断点。

## 3. 调试器观察项

在 `Window > Show View > Expressions` 中逐项添加：

```text
adc_dual_stats.state
adc_dual_stats.cubemx_ready
adc_dual_stats.timer_running
adc_dual_stats.last_hal_status
adc_dual_stats.dma_half_count
adc_dual_stats.dma_full_count
adc_dual_stats.error_count
adc_dual_stats.overflow_count
adc_dual_stats.sample_pair_count
adc_dual_stats.ch1_min_code
adc_dual_stats.ch1_max_code
adc_dual_stats.ch2_min_code
adc_dual_stats.ch2_max_code
```

`adc_dual_stats` 是 `adc_dual.c` 内的文件静态变量。若 Expressions 暂时无法解析，先在
`adc_dual_process()` 内暂停一次，再刷新表达式。

正常运行时应满足：

- `cubemx_ready == 1`。
- `last_hal_status == 0`，即 `HAL_OK`。
- `state` 在采集时为 `ADC_DUAL_STATE_RUNNING`，FFT 处理期间可短暂为 `STOPPED`。
- `dma_half_count`、`dma_full_count` 和 `sample_pair_count` 持续增加。
- `error_count == 0`，`overflow_count == 0`。

## 4. 直流测试顺序

每次改线前先断电，并确保信号源与核心板共地。

### 4.1 两路接地

PC4、PB1 都接 GND。运行后两路原始码应靠近 0，允许存在少量零点噪声。若码值接近
满量程或大幅跳动，停止测试并检查引脚、地线和 ADC 通道映射。

### 4.2 两路接同一个中间电压

PC4、PB1 同接稳定的约 1.65 V。理论 16 位码约为：

```text
65535 * 1.65 / VREF+
```

当 `VREF+ = 3.3 V` 时约为 32768。两路最小值和最大值应处于相近范围，且不应出现
固定半帧错位。

### 4.3 两路接 3.3 V

确认实测电压不高于 VREF+ 后，PC4、PB1 同接 3.3 V。两路码值应接近 65535，但不要
利用该点判断绝对精度；核心板 3.3 V 与 VREF+ 的差异会直接影响结果。

## 5. 同步与 FFT 测试

直流测试通过后，将同一个经过 `0～3.3 V` 偏置和限幅的正弦信号同时送到 PC4、PB1：

1. 先测 1 kHz，再测 10 kHz，最后测 20 kHz。
2. 信号必须始终处于 ADC 允许范围内，不得出现负半周。
3. 两路同源时相位差应接近 0°；当前结果只用于验证同步性，不作为最终 1°精度结论。
4. 频率显示应跟随输入；80 kSPS、8192 点 FFT 的频点间隔为 9.765625 Hz，插值结果可优于该间隔。
5. 未写入前级校准参数前，幅度显示 `--` 属于正确状态。

## 6. 通过标准

- 两路 DMA 半满和满帧计数持续增加。
- ADC 和 DMA 错误计数保持为 0。
- 两路接相同直流时原始码趋势一致。
- 两路接同一正弦时频率正确、相位差接近 0°。
- 串口屏保持刷新且不影响 DMA 错误计数。
- 所有电压精度、幅度精度和相位精度结论均标记为“待前级与校准完成后硬件验证”。

