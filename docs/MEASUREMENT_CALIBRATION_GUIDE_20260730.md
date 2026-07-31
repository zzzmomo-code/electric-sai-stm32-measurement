# G题测量结果物理量与前端增益校准指南

## 1. 2026-07-31协议单位

FPGA新协议已经明确区分“标量物理量”和“数组压缩码”：

| 数据 | 协议单位 |
|---|---|
| `vpp_uv` | µV，峰峰值 |
| `vrms_uv` | µV，真有效值 |
| `dc_uv` | µV，有符号直流偏置 |
| `fundamental_mhz` | mHz |
| `component[].frequency_mhz` | mHz |
| `component[].amplitude_peak_uv` | µV，正弦峰值 |
| `time_samples[]` | `int16_t`二补码，乘本帧`time_uv_per_lsb` |
| `spectrum[]` | `uint16_t`，乘本帧`spectrum_uv_per_lsb`得到µV_peak |

当前量化系数为：

```text
time_uv_per_lsb     = 250 µV/LSB
spectrum_uv_per_lsb = 125 µV_peak/LSB
```

标量电压字段不能再执行旧的`/6400`或`/12800`原始码换算。

## 2. STM32校准层

工程使用：

```text
Core/User/measurement_calibration.c
Core/User/measurement_calibration.h
```

旧打表时阻抗匹配错误，用户确认测得电压约为端口真实值的两倍。因此已校准模式
暂时保留uV到uV的前端逆增益：

```text
input_vpp_uv  = fpga_vpp_uv  / 2.0
input_vrms_uv = fpga_vrms_uv / 2.0
input_peak_uv = fpga_peak_uv / 2.0
```

对应常量：

```c
#define MEASUREMENT_FRONTEND_VPP_GAIN        2.0
#define MEASUREMENT_FRONTEND_VRMS_GAIN       2.0
#define MEASUREMENT_FRONTEND_COMPONENT_GAIN  2.0
```

这里的2.0必须表示“信号源输入参考面到FPGA测量参考面”的实际模拟增益。如果FPGA
已经补偿同一个2倍误差，应把三个值统一改为1.0，禁止FPGA和STM32重复补偿。

## 3. 串口屏校准切换

淘晶驰页面的`t_nihe`显示当前模式，按钮事件保持：

```text
printh A5 20 5A
```

- 已校准：显示应用前端逆增益后的输入端电压；
- 未校准：直接显示FPGA发送的物理uV/mHz；
- 切换只重新生成数字文本；
- 不清空、不重画波形和频谱；
- 不向FPGA发送命令，不影响SPI持续收帧。

## 4. 时域和频谱换算

时域必须先以32位有符号整数换算：

```c
int32_t time_uv =
    (int32_t)time_sample * (int32_t)header.time_uv_per_lsb;
```

频谱必须先以32位无符号整数换算：

```c
uint32_t spectrum_peak_uv =
    (uint32_t)spectrum_code
    * (uint32_t)header.spectrum_uv_per_lsb;
```

当前最大乘积约8.192 V，32位整数足够。时域不能当`uint16_t`，频谱不能当
`int16_t`；频谱幅值是峰值，不是峰峰值。

## 5. 调试观察量

```text
fpga_link_diagnostics.last_protocol_result
fpga_link_diagnostics.frame_valid_count
fpga_link_diagnostics.published_snapshot_count
measurement_conversion_diagnostics.last_time_uv_per_lsb
measurement_conversion_diagnostics.last_spectrum_uv_per_lsb
measurement_conversion_diagnostics.last_spectrum_max
measurement_conversion_diagnostics.last_spectrum_max_uv
measurement_calibration_enabled
measurement_calibration_diagnostics.apply_count
```

新FPGA正常接入时应满足：

```text
last_protocol_result == FPGA_PROTOCOL_OK
last_time_uv_per_lsb == 250
last_spectrum_uv_per_lsb == 125
```

软件测试和编译通过不等于实板参考面已经确认。最终还需要FPGA同学确认标量电压
是否已经做过模拟前端增益补偿，并用已知幅度单音复测Vpp、Vrms和分量峰值。
