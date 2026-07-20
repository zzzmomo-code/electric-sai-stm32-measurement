# DDS 基于 ADC 中频的五次闭环补偿设计

## 目标

串口屏“立即测量”按键发送 `M` 后，先使用最新 TIM5 粗测外界频率设置一次 DDS
初值，再使用 CH1 ADC/FFT 测得的中频连续修正 DDS 五次。第五次成功修正后退出
补偿状态并保持最终 DDS 频率，直到下一次收到 `M`。

## 频率关系

硬件采用低侧本振：

```text
fADC = fEXTERNAL - fDDS
```

目标中频固定为 100 kHz。每次闭环修正使用：

```text
error = fADC - 100000
fDDS_new = fDDS_current + error
```

ADC 中频偏高时提高 DDS，ADC 中频偏低时降低 DDS。误差按最近整数 Hz 四舍五入后
参与无符号 DDS 频率计算，计算过程先使用有符号宽类型，防止下溢或溢出。

## 触发与状态机

补偿只由 HMI 的 `M` 命令触发。普通 TIM3 周期测频不再自动改变处于保持状态的
DDS 输出。

`M` 命令在主循环上下文中同时调用：

- `frequency_measure_request_now()`：请求本轮 TIM5 粗测；
- `dds_control_request_compensation()`：请求重新开始 DDS 闭环。

`system_process()` 当前先处理 HMI 输入，再处理 TIM5 测频，随后处理 DDS，因此同一
主循环内 DDS 可以使用本次立即测量得到的 `frequency_measure_hz`。

DDS 控制状态扩展为：

- `waiting`：尚无合法粗测结果；
- `test`：保留现有固定测试模式；
- `coarse`：收到请求并等待或应用 TIM5 粗调；
- `compensating`：等待新的有效 FFT 帧并执行五次修正；
- `holding`：第五次修正成功，保持最终 DDS 输出；
- `error`：DDS 目标越界或 AD9834 写入失败，本轮终止。

再次收到 `M` 时，无论当前处于 `compensating`、`holding` 或 `error`，都清零本轮
计数并重新执行 TIM5 粗调。

## 数据流与帧同步

粗调成功后：

1. 补偿计数清零；
2. 记录当前 `measurement_result.sequence`；
3. 调用 `measurement_fft_resynchronize()` 放弃正在采集的旧 DDS 窗口；
4. 进入 `compensating`。

在 `compensating` 状态中，`dds_control_process()` 每次只接受一个新的
`measurement_result.sequence`。结果必须满足：

- `measurement_result_get_snapshot()` 返回有效快照；
- `result.valid != 0`；
- `result.valid_mask` 包含 `MEASUREMENT_VALID_FREQUENCY`；
- `result.frequency_hz` 有限且大于 0。

无效或重复帧不计入五次调整。每次 DDS 成功写入后计数加一；前四次写入后再次调用
`measurement_fft_resynchronize()`，确保下一帧全部对应新的 DDS 输出。第五次写入后
直接进入 `holding`，不再启动下一轮补偿采集。

不设置频率死区。即使四舍五入后的误差为 0，也重写当前 DDS 频率并计作一次成功
调整，从而保证本轮最多消费五个有效 FFT 结果后退出。

## 范围与错误处理

- TIM5 粗测输入仍限制为 1～30 MHz。
- DDS 初值继续使用 `fTIM5 - 100 kHz`。
- 每次闭环目标必须位于 1～`AD9834_MAX_OUTPUT_HZ` Hz。
- 目标越界或 `ad9834_set_frequency_hz()` 失败时，增加错误计数并进入 `error`；失败
  写入不增加补偿次数。
- 无效 ADC/FFT 结果只等待下一帧，不进入错误状态。

所有控制操作均在主循环执行；UART、TIM3、ADC 和 DMA 中断回调继续只设置各自的
单一标志，不增加计算或外设写入。

## 接口与诊断

`dds_control.h` 新增：

```c
void dds_control_request_compensation(void);
```

`dds_control_diagnostics_t` 增加：

- `compensation_count`：本轮已成功完成的 ADC 闭环修正次数，范围 0～5；
- `compensation_request_count`：收到的立即补偿请求总数；
- `adc_frequency_hz`：最近一次用于补偿的 CH1 校准后 ADC 中频；
- `frequency_error_hz`：最近一次 `fADC - 100000` 的有符号误差；
- `last_result_sequence`：最近一次已经消费的 FFT 结果序号。

现有 `update_count` 继续统计包括初始化、粗调和闭环修正在内的全部成功 DDS 更新。

## 文件影响

- `Core/User/dds_control.c/.h`：实现状态机、补偿公式、请求接口和诊断。
- `Core/User/hmi_tjc.c`：`M` 命令增加补偿请求调用。
- `tests/test_dds_contract.py`：增加公式、五次状态机、有效序号和 HMI 触发契约。
- `README.md`：记录立即测量的粗调与五次 ADC 闭环流程。

不修改 CubeMX、`.ioc`、GPIO、时钟、DMA 或 NVIC 配置。

## 验证标准

- 数学测试证明 ADC 中频高/低于 100 kHz 时 DDS 调整方向正确。
- 静态契约证明 `M` 同时触发 TIM5 立即测量与 DDS 补偿请求。
- 静态契约证明只消费新的有效 FFT 序号、成功写入才累计，计满 5 次进入保持状态。
- 现有全部 Python 契约测试通过。
- STM32 Debug 工程构建成功。
