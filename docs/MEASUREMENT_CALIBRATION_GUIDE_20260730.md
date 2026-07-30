# G题测量结果打表拟合与串口屏切换指南

## 1. 当前实现结论

工程已经加入独立校准模块：

```text
Core/User/measurement_calibration.c
Core/User/measurement_calibration.h
```

上电默认使用“已校准”模式。当前尚未取得正式打表数据，因此所有曲线都暂设为
`y=x`；此时已校准和未校准显示相同，这是正常现象。

淘晶驰页面的 `t_nihe` 控件显示当前模式：

```text
已校准：使用拟合函数
未校准：直接使用 FPGA 原始测量结果
```

按钮按下事件必须是：

```text
printh A5 20 5A
```

每按一次就在两种模式间切换。切换只重算并刷新数字文本，不清空波形、不重画频谱、
不向 FPGA 发命令，也不重新启动测量。

## 2. 哪些量参与拟合

当前拟合接口覆盖：

| 测量量 | 输入单位 | 输出单位 | 串口屏对象 |
|---|---:|---:|---|
| 峰峰值 UPP | V | V | `t_vpp` |
| 真有效值 URMS | V | V | `t_vrms` |
| 基频 | Hz | Hz | `t_freq` |
| 各有效分量频率 | Hz | Hz | `t_comp1`～`t_comp3` |
| 各有效分量峰值幅度 | V | V | `t_comp1`～`t_comp3` |
| 直流偏置 | V | V | 已预留接口，当前页面未显示 |

时域波形和频谱曲线是归一化后的形状显示，不通过标量拟合函数改变。这样切换
已校准/未校准时不会改变曲线形状，也不会引入闪烁。

## 3. 打表后只改这里

打开：

```text
Core/User/measurement_calibration.c
```

文件顶部有五组系数：

```c
calibration_vpp_curve
calibration_vrms_curve
calibration_frequency_curve
calibration_component_amplitude_curve
calibration_dc_offset_curve
```

统一公式为：

```text
y = c0 + c1*x + c2*x² + c3*x³
```

其中：

```text
x = FPGA/STM32 当前未校准读数
y = 标准仪器参考值
```

现在每组都是：

```c
{0.0, 1.0, 0.0, 0.0}
```

也就是 `y=x`。

打表拟合后，把对应组改成拟合软件得到的 `c0、c1、c2、c3` 即可。电压系数使用
V 作为输入和输出单位，频率系数使用 Hz 作为输入和输出单位，不要直接把 µV 或
mHz 的系数填进来。

若最终只需要一次线性校准 `y=a*x+b`，则填写：

```c
{b, a, 0.0, 0.0}
```

## 4. 推荐打表步骤

1. 先按拟合切换键，将 `t_nihe` 切到“未校准”。
2. 选择覆盖题目量程的若干标准输入点，建议低、中、高量程均有数据。
3. 每个点稳定后记录串口屏未校准读数 `x` 和标准仪器参考值 `y`。
4. 分别对 UPP、URMS、频率和分量幅度拟合。
5. 优先尝试一次线性；只有残差明显弯曲时再使用二次或三次项。
6. 把系数写入 `measurement_calibration.c`，重新编译、烧录。
7. 上电默认显示“已校准”，重新走完整量程验证。
8. 现场按键切换已校准/未校准，确认原始值可追溯、拟合值误差满足题目要求。

## 5. CubeIDE调试观察量

可在 Expressions 中加入：

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

`toggle_count` 每按一次增加。若计数不变，先检查屏幕事件是否确实发送
`A5 20 5A`、PA10 RX 接线以及屏幕和 STM32 的 512000 baud 配置。

## 6. 当前验证边界

当前已经完成代码接口和主机契约测试。正式拟合系数、量程外行为、温漂以及最终误差
仍需要用标准仪器完成打表和实板验收；在打表完成前不要把 `y=x` 当作已完成校准。
