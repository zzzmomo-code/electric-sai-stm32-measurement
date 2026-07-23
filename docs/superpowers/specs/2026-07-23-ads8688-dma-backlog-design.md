# ADS8688 DMA 双标志积压处理设计

## 问题与证据

ADS8688 SPI3、DMA 和最新值存储已经正常，但 FFT 窗口始终无法完成。实板诊断显示：

- `measurement_fft_diagnostics.capture_resync_count = 66`
- `measurement_fft_diagnostics.window_count = 0`
- `ads8688_diagnostics.lost_samples = 68608`
- `ads8688_diagnostics.recoveries = 66`
- `measurement_fft_settle_count` 反复停在 512

当前 `ads8688_process()` 在 DMA 半满和全满标志同时待处理时，无条件丢弃
1024 个帧、调用 `measurement_fft_resynchronize()` 并恢复 ADS8688。双标志积压本身
仍然包含两个可按地址顺序处理的完整半区，因此该分支会在正常主循环延迟下错误地
反复清空 FFT 窗口。

## 选定方案

保留现有布尔 DMA 标志和 `ads8688_expected_half` 状态，不改变 DMA 缓冲区大小、
SPI3 时钟、ADS8688 通道模式或 FFT 公共接口。

当两个标志同时待处理时：

- `ads8688_expected_half == 0`：先处理前半区，再处理后半区；
- `ads8688_expected_half == 1`：先处理后半区，再处理前半区；
- 每处理一个半区都通过现有原子领取函数清除对应标志并推进
  `ads8688_expected_half`；
- 不增加 `lost_samples`，不调用 `measurement_fft_resynchronize()`，也不触发器件恢复。

当只有一个标志且它不是当前期望半区时，继续沿用现有保护行为：清除错误标志、
记录 512 个丢失帧、恢复 ADS8688。SPI/DMA 错误回调的恢复流程保持不变。

## 数据流

```text
DMA 半满/全满回调
        |
        v
设置各自唯一标志
        |
        v
ads8688_process() 获取标志快照
        |
        +-- 两个标志 --> 按 expected_half 连续处理两个半区
        |
        +-- 期望的单标志 --> 处理该半区
        |
        +-- 非期望单标志 --> 记录丢样并恢复
        |
        v
measurement_fft_ingest_sample()
        |
        v
连续收满 65536 对样本后进入 FFT
```

## 测试与验收

先增加静态回归测试，证明当前代码因“双标志直接恢复”而失败。测试要求：

1. `pending_flags == 3` 不得直接增加 `ADS8688_DMA_WORD_COUNT` 丢样；
2. `pending_flags == 3` 不得直接调用 `measurement_fft_resynchronize()` 或
   `ads8688_recover()`；
3. 前后半区仍由 `ads8688_expected_half` 决定处理顺序；
4. 非期望的单半区标志仍保留原恢复路径。

完成代码后运行 ADS8688 专项测试、全量契约测试和 STM32CubeIDE Debug 编译。
实板验收时应看到 `measurement_fft_capture_index` 连续增长到 65536、
`window_count` 与 `fft_count` 增加，且正常运行期间 `capture_resync_count` 不再
按 DMA 周期持续增加。

