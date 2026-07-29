# G26 串口屏实板联调交接备注（2026-07-29）

## 本次交付范围

- 分支：`codex/fpga-hmi-bode`
- MCU：STM32H743VIT6
- FPGA—STM32：SPI3 主机，Mode 0，目标 20 MHz，协议 V1
- STM32—淘晶驰：USART1，512000 baud，8N1，DMA 收发
- 页面工程：仓库根目录 `fpga1.HMI`
- 当前状态：FPGA SPI 从机尚未完成，固件保持串口屏自检模式

## 已完成并经过实板观察的内容

1. 串口屏能够接收 STM32 参数文本和曲线指令。
2. `s_t1` 与 `s_t3` 位于左侧同一位置，分别用于一周期和三周期。
3. `s_spec` 独立位于右侧，切换一周期/三周期时始终保持显示。
4. 三个 Waveform 控件宽度统一为 350 点。
5. 串口屏按键能够通过 `A5 01 5A`、`A5 02 5A` 控制时域显示模式。
6. 曲线发送成功，USART1 DMA 发送完成且按键命令能够被固件解析。
7. 曲线纵坐标改为 8～201，适配高度 210 的控件，已消除节点超过高度造成的削顶。
8. 页面参数区已包含峰峰值、真有效值、基频、状态和最多三个频率分量。

以上结论只证明 STM32—串口屏链路和自检显示正常，不代表 FPGA 正式测量精度已经通过。

## 当前必须保留的自检开关

FPGA SPI 从机未完成前，`Core/User/system.c` 中保持：

```c
#define HMI_CHART_SELF_TEST_ENABLE 1u
```

它会生成固定参数、时域曲线和频谱，用来独立检查串口屏。接入 FPGA 正式帧前必须改为：

```c
#define HMI_CHART_SELF_TEST_ENABLE 0u
```

否则屏幕仍会优先显示自检快照，不能用于判断 FPGA 数据是否正确。

## FPGA 完成后的联调顺序

1. 断电检查并连接公共地、SPI3 SCK/MOSI/MISO、软件 CS 和 DATA_READY。
2. 将 `HMI_CHART_SELF_TEST_ENABLE` 改为 `0`，重新编译并烧录 STM32。
3. 用逻辑分析仪确认 SPI Mode 0、MSB first、同一 CS 内先发 `A5 CMD` 再立即返回数据。
4. 在 CubeIDE Expressions 观察：
   - `fpga_link_diagnostics.status_valid_count`
   - `fpga_link_diagnostics.frame_valid_count`
   - `fpga_link_diagnostics.frame_crc_error_count`
   - `fpga_link_diagnostics.frame_dma_timeout_count`
   - `fpga_link_diagnostics.ack_count`
   - `hmi_task2_diagnostics.preload_complete_count`
5. 正常现象：
   - `status_valid_count`、`frame_valid_count`、`ack_count` 随新帧增加；
   - CRC、格式和 DMA 超时计数保持 0；
   - 屏幕参数和三组曲线使用同一帧序号更新。
6. 若收不到帧，按 DATA_READY → CS → SCK/MOSI → MISO → 状态 CRC → 完整帧 CRC 的顺序排查。

## G 题正式验收清单

- `ua`：覆盖 100～250 mVpp、10～200 kHz。
- `ub`：覆盖 50～250 mVpp、10～500 kHz。
- 测试基波加 1 个、2 个谐波，确认谱线数量、相对位置和相对高度。
- 记录峰峰值、真有效值和各分量幅值误差，目标绝对误差不超过 5 mV。
- 记录基频误差，目标绝对误差不超过 1 kHz。
- 叠加 200 mVpp、频率不低于 1 MHz 的干扰，验证滤波后的全部测量和显示功能。
- 从上电开始计时，分别验证一周期、三周期、参数和频谱项目在 2 秒内显示。
- 确认 `t_comp1`～`t_comp3` 能完整显示频率与幅值，例如：

```text
H1 12.345 kHz 50.000 mV
```

- 整机检查单 5 V 供电、BNC 输入和 50 Ω 匹配。

## 尚未验证的边界

- FPGA SPI 从机实际电气时序和连续帧稳定性；
- STM32 SPI DMA 与 D-Cache 在实板连续运行下的正确性；
- FPGA 输出字段的实际端序、单位、点数和 CRC；
- 真实信号下一周期/三周期截取是否准确；
- Vpp、真有效值、基频和分量幅值的最终测量精度；
- 干扰抑制效果和整机 2 秒指标。

## 本次提交验证记录

- Debug 构建：使用 CubeIDE 随附 GNU Tools for STM32 13.3，全量编译和 ELF 链接成功；
  资源统计为 `text=64004`、`data=472`、`bss=47520`。
- 合同测试：执行 50 项，44 项通过。G 题 SPI 协议、CRC、DMA/D-Cache、350 点转换
  等测试通过。
- 其余 6 项中，5 项属于历史 ADC/DAC/DDS/TIM 链路断言；另 1 项要求正常模式，
  当前为等待 FPGA 的串口屏联调阶段，因此有意保留
  `HMI_CHART_SELF_TEST_ENABLE=1`。

提交或合并时不得把上述待验证项描述成“已经实板通过”。
