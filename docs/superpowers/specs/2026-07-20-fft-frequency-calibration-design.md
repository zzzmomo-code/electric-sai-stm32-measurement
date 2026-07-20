# FFT 双通道频率校准设计

## 1. 目标

在 `Core/User/measurement_fft.c` 中增加独立频率校准函数，对双通道 FFT 插值得到的原始频率应用指定分段线性公式。诊断结构同时保留原始 FFT 频率和校准后频率；发布到 `measurement_result`、HMI 和其他下游模块的频率使用校准后结果。

## 2. 校准公式

设 FFT 插值得到的原始频率为 `f_raw`，校准频率为 `f_cal`：

```text
f_raw <= 40000 Hz：f_cal = 0.9999807 * f_raw - 0.2414
f_raw >  40000 Hz：f_cal = 0.99995854 * f_raw - 0.3226
```

边界值 40000 Hz 必须进入第一段。为避免无效测量产生负频率或传播 NaN/Infinity，输入非有限值、零或负值时返回 0 Hz；公式结果为负值时也钳位到 0 Hz。

## 3. 配置宏和公开接口

在 `Core/User/measurement_fft.h` 中增加以下宏，便于后续重新标定：

```c
#define MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ       40000.0f
#define MEASUREMENT_FFT_LOW_FREQUENCY_GAIN       0.9999807f
#define MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ  (-0.2414f)
#define MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN      0.99995854f
#define MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ (-0.3226f)
```

新增公开纯函数：

```c
float measurement_fft_calibrate_frequency(float raw_frequency_hz);
```

函数不访问外设、不修改全局状态，可以由合同测试、调试代码或其他用户模块直接调用。

## 4. 诊断字段命名

现有 `measurement_fft_diagnostics_t` 中的频率字段扩展为：

```c
float raw_peak_frequency_hz;
float peak_frequency_hz;
float secondary_raw_peak_frequency_hz;
float secondary_peak_frequency_hz;
```

含义如下：

- `raw_peak_frequency_hz`：CH1 FFT 峰值插值得到的原始频率；
- `peak_frequency_hz`：CH1 经分段公式校准后的频率；
- `secondary_raw_peak_frequency_hz`：CH2 FFT 峰值插值得到的原始频率；
- `secondary_peak_frequency_hz`：CH2 经分段公式校准后的频率。

“raw”仅表示未经本次频率公式校准；它仍包含现有三点抛物线插值结果，不退回整数 FFT 频点。

## 5. 双通道数据流

每帧双通道 FFT 完成后：

1. 使用 `peak_position * bin_width_hz` 分别计算 CH1、CH2 原始插值频率；
2. 将原始值写入两个 `raw` 诊断字段；
3. 分别调用 `measurement_fft_calibrate_frequency()`；
4. 将结果写入两个校准后诊断字段；
5. `measurement_fft_publish()` 继续把 `peak_frequency_hz` 和 `secondary_peak_frequency_hz` 写入 `measurement_result.frequency_hz` 与 `secondary_frequency_hz`；
6. HMI 现有接口不改名，自动显示校准后频率。

两个通道独立校准，不用 CH1 结果覆盖 CH2，也不取双通道平均值。

当某通道被判定为直流输入时，该通道的 raw 和校准频率都清零。其他无效状态沿用现有有效位和故障处理逻辑，不改变波形、THD、相位或电压结果。

## 6. 兼容性和范围

- `measurement_result_t.frequency_hz` 与 `secondary_frequency_hz` 保持原名，避免修改 HMI 和结果消费者接口；
- 只对 FFT 模块的双通道结果应用公式，不修改独立的 TIM5 `frequency_measure_hz`；
- 不修改 FFT 长度、采样率、插值算法、频谱、相位、THD 或电压校准；
- 不修改 CubeMX、ADC、DMA、GPIO 或时钟配置；
- 不修改或提交当前工作区中无关的 `main.c` 与 Debug 构建产物变化。

## 7. 验证

自动测试需要覆盖：

- 公式五个配置宏及公开函数声明存在；
- `f_raw <= 40000` 使用第一段，`f_raw > 40000` 使用第二段；
- 0 Hz 和负值返回 0 Hz；
- 40000 Hz 严格使用第一段；
- 两个 raw 诊断字段与两个校准诊断字段同时存在；
- 双通道先保存 raw，再分别调用校准函数；
- 发布到 `measurement_result` 的仍是两个校准字段；
- 直流通道同时清零 raw 和校准频率；
- 现有 FFT、频率测量、VGA 和 DDS 合同不回归。

完成合同测试后，对 `measurement_fft.c` 执行 ARM GCC 语法检查。由于当前工作区存在用户自己的 `main.c` 临时调用，全量测试若受该无关改动影响，需要单独报告，不覆盖或回滚该改动。
