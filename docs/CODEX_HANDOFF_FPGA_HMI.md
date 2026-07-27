# FPGA -> STM32 -> TJC HMI 交接文档

> 交接日期：2026-07-27，由 Claude Code 交给 Codex 继续

## 1. 项目位置与平台

```
C:\Users\48747\STM32CubeIDE\workspace_1.19.0\h743_task2_20260727
```

| 项 | 值 |
|---|---|
| MCU | STM32H743VIT6 (H743, 不是 H750) |
| IDE | STM32CubeIDE 1.19.0 |
| Git 分支 | `codex/fpga-hmi-bode` |
| .ioc 文件 | `h743_task2_20260727.ioc` |

## 2. 任务目标

**FPGA 通过 1M 波特率串口发幅频/相频数据给 STM32 -> STM32 接收后串口屏显示功率 + 幅频相频曲线**

数据流：
```
FPGA -(USART2 1M baud)-> STM32 接收 1024 点 -> 降采样 64 点 -> USART1 9600 发 cle+add -> TJC 串口屏
```

## 3. 硬件引脚

| 引脚 | 功能 | 备注 |
|---|---|---|
| PA2 | USART2_TX -> FPGA RX | **新增** |
| PA3 | USART2_RX -> FPGA TX | **新增** |
| PA9 | USART1_TX -> TJC HMI RX | 已有 |
| PA10 | USART1_RX -> TJC HMI TX | 已有 |
| SPI2 PB12-15 | AD9834 | 已有 |

## 4. CubeMX .ioc 已配好的

| 外设 | 参数 | 状态 |
|---|---|---|
| USART1 | 9600 8N1, NVIC 中断 7:0 | 已有 |
| USART2 | **1000000 8N1** (1M baud), NVIC 中断 7:0 | **新增 ✓** |
| DMA1_Stream1 | USART2_RX, Peripheral-to-Memory, Normal, Byte/Byte | **新增 ✓** |
| DMA1_Stream0 | ADC1 (没动) | 已有 |

- 生成代码确认：`main.c` 自动调用 `MX_DMA_Init()` + `MX_USART2_UART_Init()` ✓
- `USART2_IRQHandler` 调用 `HAL_UART_IRQHandler(&huart2)` ✓
- `DMA1_Stream1_IRQHandler` 调用 `HAL_DMA_IRQHandler(&hdma_usart2_rx)` ✓

## 5. 新增模块

### fpga_link.h / fpga_link.c（FPGA 串口接收）

**Codex 已改进版本**，用 `HAL_UARTEx_ReceiveToIdle_DMA()` 替代原始方案。

- 绑定 USART2，启动 Receive-to-IDLE DMA 接收
- D-Cache 正确处理（`SCB_CleanInvalidateDCache` / `SCB_InvalidateDCache`）
- 中断回调只设一个 `fpga_link_rx_event_size`（符合 AGENTS.md 规则 4）
- 主循环 `fpga_link_process()` 领取事件、解析 AA55、发布快照
- 诊断量：`fpga_link_diagnostics`（rx_event_count / frame_found / valid / error 等）

**FPGA 协议**：
```
AA 55 [N高 N低] [数据×N×4字节] 0D 0A
每点 4 字节: mag2_hi(2字节大端 uint16) + phase(2字节大端 int16)
1024 点 = 4102 字节, 1M baud 下 41ms 一帧
```

**数据含义**：
- mag2_hi: 幅度响应，∝ 信号幅度²，0~65535
- phase: 相位响应，÷32768×π = 实际弧度
- 频率轴: freq[i] = (f_start + i × f_step) × 762.94 Hz

### hmi_chart.h / hmi_chart.c（Bode 图构帧）

**Codex 已改进版本**，使用 `s0.id`/`s1.id` 表达式（非数字 ID），无状态设计。

- **降采样**：任意点数 N -> 64 点（按比例分桶，每桶取 mag2_hi 最大值 + 对应 phase）
- **幅度映射**：整数平方根 sqrt(mag2_hi) -> 线性映射到 0-255
- **相位映射**：(phase + 32768) / 65535 * 255 -> 0-255
- **构帧**：`cle s0.id,0` + 64 条 `add s0.id,0,val` + `cle s1.id,0` + 64 条 `add s1.id,0,val`
- 每帧最坏约 2332 字节，缓冲区预留 2560 字节；9600 baud 下发约 2.4 秒

### hmi_task2.c（已升级）

**新增内容**（对比 codex/dds-ad9834-test 原始版）：
- t_power 文本控件（从 `measurement_result.power_w` 读，显示格式 `"%.3f W"`）
- Bode 图约每 2000 ms 选取最新 FPGA 帧并构帧
- Bode 帧与 1024 点快照放在静态区，不占用 1 KB 主栈
- 每轮主循环只发送一条完整 `cle/add` 命令，避免连续阻塞约 2 秒
- `HAL_UARTEx_RxEventCallback` 只写 `fpga_link_rx_event_size`
- `HAL_UART_ErrorCallback` 把非 USART1 错误分流给 fpga_link

### measurement_result.h/c（已升级）

- 加 `power_w` 字段 (float, 单位瓦特)
- 加 `MEASUREMENT_VALID_POWER (1u << 8)` 有效位
- 提供 `measurement_result_set_power_w()` 和 `measurement_result_clear_power()`
- 数据由用户/FPGA 算法填入，本任务不计算功率

### system.h / system.c（已升级）

- system.h 加 `#include "hmi_chart.h"` + `#include "fpga_link.h"`
- system_init() 调用：`fpga_link_init()` / `fpga_link_bind_uart(&huart2)` / `fpga_link_start()`
- hmi_chart 为无状态构帧模块，不需要初始化

## 6. ✅ 已修复问题

### system_process 已加 fpga_link_process()

`system_process()` 已在 `hmi_task2_process()` 前调用 `fpga_link_process()`。软件调用链已接通；实际接收效果仍待 FPGA 与开发板联调确认。

## 7. HMI 软件侧待完成

### 已建控件

| 控件名 | 类型 | id | 属性 |
|---|---|---|---|
| s0 (幅频) | Waveform | **6** | ch=1, h=267, gdc=1024, dir=从左往右 |
| t_power | Text | ? | 只此一个文本控件 |

### 待建控件

| 控件名 | 类型 | 用途 |
|---|---|---|
| s1 (相频) | Waveform | 建好后保持对象名 `s1`，代码无需填写数字 ID |

### 确认事项

- user 说 HMI 页面目前只有 t_power 一个文本控件，s0（s1 待建）。
- s1 建好后代码不用改（hmi_chart 用 `s1.id` 表达式，自动找控件）
- 横纵轴 + 标签：用 Text/线条控件在 HMI 软件里静态画好，不刷新

## 8. 通信方案确认

| 决定 | 结论 |
|---|---|
| cle + add 模式 | 采用（第一版） |
| addt 透传 | **不做**（用户明确：透传就不考虑了） |
| 降采样 | 1024 -> 64 点 |
| 波特率 | HMI USART1 9600 保持，FPGA USART2 1M (1000000) |
| HMI 控件 | 功率用 Text t_power，幅频相频用 Waveform s0/s1 |

## 9. 禁止 / 约束

参考 AGENTS.md 和 CLAUDE.md：
- 代码放 `Core/User/`，main.c 只调 system_init/system_process
- ISR 只置标志，不写计算和协议处理
- 不直接改 `MX_*_Init()`
- 不改 .ioc 直接从代码配外设
- **不 push** 未经用户许可
- DMA 优先级问题：DMA1_Stream1_IRQn 优先级 0:0（跟 HardFault 同级），建议 CubeMX NVIC 改为 5:0

## 10. 状态总览

| 项 | 状态 |
|---|---|
| .ioc CubeMX 配置 | ✅ 完成 |
| CubeMX 代码生成 | ✅ 完成 |
| fpga_link 模块 | ✅ Codex 已改进 |
| hmi_chart 模块 | ✅ Codex 已改进 |
| hmi_task2 升级 | ✅ 完成 |
| measurement_result 升级 | ✅ 完成 |
| system.h/c 升级 | ✅ system_process 已加 fpga_link_process() |
| HMI s0 控件 | ✅ 已建 (id=6) |
| HMI s1 控件 | ❌ 待建 |
| HMI t_power 控件 | ✅ 已建 |
| Python 契约测试 | ✅ 42 项全部通过 |
| 变更源码 GCC 检查 | ✅ CubeIDE GCC，`-Wall -Wextra -Werror` 通过 |
| 完整工程 `.elf` | ⚠️ 未取得；现有 `.cproject` 生成 `-I../../Core/Inc`，HAL 编译前即找不到头文件 |
| 实板验证 | ❌ 全部待硬件验证 |
