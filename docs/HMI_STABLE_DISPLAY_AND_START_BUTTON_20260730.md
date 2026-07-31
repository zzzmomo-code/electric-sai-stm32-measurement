# 串口屏稳定显示与开始键配置

## 目标行为

STM32仍持续读取并ACK FPGA测量帧，但屏幕曲线不再随每一帧清空重画：

1. 上电收到第一份有效快照后，立即显示参数、频谱和默认的一周期波形。
2. 后续FPGA数据仍持续接收，但屏幕保持冻结。
3. 按“开始”后立即锁存最新完整快照并更新全部显示。
4. 启动帧计作第1帧，后续小变化标量加入平均，最多累计5帧。
5. 若后续数据变化很大或800 ms到期，立即停止微调并保留此前结果。
6. 按“一周期”或“三周期”时，只切换两个已经装载的重叠波形控件。

## HMI手工修改

最新页面 `fpga1_codex_ui_v1 (1).HMI` 已经包含启动按钮 `b_t5`。不要覆盖原来的
`fpga1.HMI`，在最新页面副本中把按钮按下事件设置为：

| 属性 | 建议值 |
|---|---|
| objname | `b_t5` |
| txt | `启动` |
| 按下事件 | `printh A5 04 5A` |

三个命令的最终约定如下：

```text
printh A5 04 5A  // 开始：锁存最新完整结果并开启最多5帧小幅平均
printh A5 01 5A  // 切换到严格的一周期
printh A5 02 5A  // 切换到严格的三周期
printh A5 03 5A  // 可选：手动重画一次频谱
```

页面上的 `b_t4`“切换模式”当前发送 `printh A5 10 5A`。STM32会识别该保留命令，
但不执行任何动作，也不会累计为非法命令；该按钮留给后续模式扩展。

页面“后初始化事件”保留：

```text
vis s_t1,0
vis s_t3,0
vis s_spec,0
t_status.txt="WAIT FPGA"
```

首份有效数据到达后，STM32会自动显示 `s_spec` 和默认的 `s_t1`；`s_t3` 保持隐藏。
开始命令负责锁存后台最新完整结果，周期命令只改变 `s_t1`、`s_t3` 的前后可见关系。

## 严格周期算法

不能再用 `time_count / 3` 当作一个周期，因为FPGA缓存可能含有多于3个周期。
STM32现在按以下公式计算真实周期：

```text
period_samples = time_sample_rate_hz × 1000 / fundamental_mHz
```

然后在时域缓存中寻找均值附近的上升穿越点，取得一个完整基波周期作为共同模板：

```text
一周期：模板重复1次
三周期：同一模板连续重复3次
```

两个输出都使用Q16.16周期线性插值为350点，所以无论10 kHz还是400～500 kHz，
横轴都应正好铺满1个或3个周期。周期延拓还避免了FPGA缓存不包含最后一个闭合端点时
高频合法帧被拒绝的问题。

## 实板验收

1. 上电后确认第一份有效快照到达时参数、频谱和一周期波形立即出现。
2. 保持输入不变10秒，频谱不得反复清空、闪烁。
3. 改变输入但不按开始，屏幕应保持原结果；按开始后应立即显示最新完整结果。
4. 按三周期，横轴应准确出现3个周期；再按一周期，应准确出现1个周期。
5. 不按键继续等待10秒，波形不得自行重画或闪烁。
6. 启动后观察标量只在小范围内微调，最多累计5帧；大变化应停止微调并保持此前结果。

调试时可观察：

```text
hmi_task2_diagnostics.stable_candidate_count  // 0~5
hmi_task2_diagnostics.stable_accept_count
hmi_task2_diagnostics.stable_reject_count
hmi_task2_diagnostics.command_count
hmi_task2_diagnostics.last_command            // 开始键应为4
hmi_task2_diagnostics.tx_error_count           // 应保持0
measurement_conversion_diagnostics.last_one_cycle_samples
```

本次代码已通过主机合同测试和工程编译；新增显示策略及严格周期数仍需按上述步骤实板验证。
