# 片上双 ADC DMA 半区顺序处理设计

## 问题与证据

片上 ADC 的 `ch1_recent_mean_code` 能正常更新，说明 ADC1/ADC2、TIM2、DMA 和
原始样本解包链路有效。但实板观察到 `adc_dual_stats.backlog_count` 与
`measurement_fft_diagnostics.capture_resync_count` 持续增加，FFT 窗口无法完成。

当前 `adc_dual_process()` 在前后半区标志同时待处理时，直接增加
`dropped_pair_count`、调用 `measurement_fft_resynchronize()` 并返回。DMA 缓冲区
只有 1024 对样本，在 600 kSPS 下每 0.853 ms 产生一个半区事件，因此正常主循环
延迟就可能触发该分支，并反复清空 65536 点窗口。

## 选定方案

保持 600 kSPS、1024 对 DMA 缓冲区、65536 点 FFT、现有公共接口和中断回调不变。
在 `adc_dual.c` 内增加 `adc_dual_expected_half`：

- `0` 表示下一个可信半区应为前半区；
- `1` 表示下一个可信半区应为后半区；
- 初始化、启动和停止时重置为 `0`；
- 成功处理一个半区后切换到另一个半区。

`adc_dual_process()` 根据标志快照与期望半区处理：

- 两个标志同时存在：按 `adc_dual_expected_half` 指定的顺序连续处理两个半区；
- 只有期望半区标志：处理该半区并推进期望状态；
- 只有非期望半区标志：记录一个半区的丢失样本，调用
  `measurement_fft_resynchronize()`，丢弃该错误半区并保持当前期望状态，等待
  DMA 环形缓冲区的下一个正确半区；
- ADC/DMA 错误回调及现有错误停机行为保持不变。

## 数据流

```text
ADC DMA 半满/全满回调
        |
        v
各自设置唯一标志
        |
        v
adc_dual_process() 原子领取标志快照
        |
        +-- 两个标志 --> 按 expected_half 连续处理两块
        |
        +-- 期望单标志 --> 处理并推进 expected_half
        |
        +-- 非期望单标志 --> 记录丢样并重同步 FFT
        |
        v
measurement_fft_ingest_pair()
        |
        v
约 111 ms 收满稳定期与 65536 对样本并进入 FFT
```

## 诊断语义

- `dma_half_count`、`dma_full_count`：每成功处理一个对应半区增加一次；
- `backlog_count`：一次主循环同时领取到两个标志时增加一次，但不再代表丢样；
- `sample_pair_count`：FFT 接受的同步样本对数量；
- `dropped_pair_count`：仅统计 FFT 暂停、输入状态拒绝或真正单半区乱序造成的丢弃；
- `capture_resync_count`：正常双标志积压时不增加，单半区乱序时增加。

## 测试与验收

先修改契约测试，使当前“双标志直接重同步”代码产生预期失败。测试要求：

1. 存在 `adc_dual_expected_half` 并在初始化、启动和停止时清零；
2. 双标志分支不增加整缓冲区丢样，不调用 `measurement_fft_resynchronize()`，
   且能够处理两个半区；
3. 处理每个半区后推进期望状态；
4. 单半区乱序仍保留半缓冲区丢样计数与 FFT 重同步；
5. ADC 中断回调仍只给各自唯一的 `xxx_flag` 赋值。

软件验证包括片上 ADC/FFT 契约测试、全量测试和 STM32CubeIDE Debug 编译。实板
烧录后应看到 `measurement_fft_capture_index` 在约 111 ms 内达到 65536，
`window_count` 与 `fft_count` 增加；正常运行时 `capture_resync_count` 不再随着
`backlog_count` 同步增加。
