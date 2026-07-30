# G题测量结果打表拟合与串口屏切换指南

## 1. 当前冻结换算

工程使用独立校准模块：

```text
Core/User/measurement_calibration.c
Core/User/measurement_calibration.h
```

2026-07-30 打表稳健拟合得到：

```text
K_UPP  = 6405.50 raw/mV
K_URMS = 6404.76 raw/mV
K_SPEC = 6399.19 raw/mV
```

三者非常接近，因此比赛固件统一使用 6400 raw/mV：

```c
Upp_mV  = raw_upp  / 6400.0f;
Urms_mV = raw_rms  / 6400.0f;
Ui_mV   = raw_spec / 6400.0f;  /* 正弦峰值幅度，不是峰峰值 */
f_Hz    = raw_freq / 1000.0f;  /* FPGA频率字段单位为mHz */
```

模块对外仍使用 uV/mHz 整数接口，内部等效电压公式为：

```text
calibrated_uV = raw_value × 1000 / 6400
```

剔除明显录入或漏检异常点后，统一 `/6400` 的最大绝对误差为：

| 测量量 | 最大绝对误差 |
|---|---:|
| 峰峰值 Upp | 1.226 mV |
| 真有效值 Urms | 2.281 mV |
| 频谱分量峰值 Ui | 3.636 mV |

上述结果满足题目 5 mV 绝对误差要求，但仍需在完整量程和实板温漂条件下复验。

## 2. 串口屏校准切换

淘晶驰页面的 `t_nihe` 控件显示当前模式。按钮按下事件：

```text
printh A5 20 5A
```

上电默认“已校准”：

- 已校准：电压原始码按 `/6400` 换算为实际电压，频率按协议单位显示；
- 未校准：电压字段直接显示 FPGA 原始码值，方便继续打表；
- 切换只重新生成数字文本；
- 不清空或重画曲线；
- 不向 FPGA 发送命令，不影响 SPI 持续收帧。

## 3. 后续重新打表只改三个参数

打开：

```text
Core/User/measurement_calibration.c
```

修改：

```c
#define MEASUREMENT_CALIBRATION_VPP_RAW_PER_MV       6400.0
#define MEASUREMENT_CALIBRATION_VRMS_RAW_PER_MV      6400.0
#define MEASUREMENT_CALIBRATION_COMPONENT_RAW_PER_MV 6400.0
```

参数含义都是“每 1 mV 对应多少 FPGA 原始码”。频谱分量的幅度是正弦峰值，
不能按峰峰值填写。若 FPGA 后续改为直接发送真实 uV，必须同步取消这里的 `/6400`
换算，否则会重复校准。

## 4. 调试观察量

CubeIDE Expressions 建议加入：

```text
measurement_calibration_enabled
measurement_calibration_diagnostics.apply_count
measurement_calibration_diagnostics.toggle_count
measurement_calibration_diagnostics.enabled
hmi_task2_diagnostics.calibration_toggle_count
hmi_task2_diagnostics.calibration_enabled
```

正常现象：

```text
上电：enabled = 1，t_nihe 显示“已校准”
按一次：enabled = 0，t_nihe 显示“未校准”
再按一次：enabled = 1，t_nihe 显示“已校准”
```

若 `toggle_count` 不增加，检查屏幕事件是否为 `A5 20 5A`、USART2 RX 接线以及
屏幕和 STM32 的 512000 baud 配置。

## 5. 当前边界

- 当前函数依据本次打表结论假定 FPGA 电压字段是原始码，而不是真实 uV；
- 小谱线漏检不是线性校准问题，应由 FPGA 的谱线检测或定点谐波搜索处理；
- 时域和频谱曲线只做形状归一化，不经过 `/6400`；
- 软件测试和工程编译通过不等于最终实板精度验收，比赛前仍需完整量程复测。
