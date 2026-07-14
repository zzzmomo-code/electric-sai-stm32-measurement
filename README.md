# STM32H750 + ADS8688 双通道信号分析器

本工程是大学生电子设计竞赛测量组训练题 1 的固件基线。目标 MCU 为
`STM32H750VBT6`，开发环境为 `STM32CubeIDE 1.19.0`。当前软件链路为：

`ADS8688 SPI2 DMA -> AIN0/AIN1 样本存储 -> 8192 点 Q15 FFT -> 测量结果快照 -> TJC4827T143_011R_I_P20 串口屏`

当前实际工程目录：

`C:\Users\48747\STM32CubeIDE\workspace_1.19.0\h7_pre`

## 已实现功能

- ADS8688 AIN0/AIN1 自动循环采集，SPI2 RX/TX 使用 DMA1 Stream0/Stream1。
- STM32H7 DMA 缓冲区 32 字节对齐，并在 CPU 读取前执行 D-Cache 失效维护。
- 最新值和 4096 条历史样本环形缓存。
- 8192 点 CMSIS-DSP Q15 RFFT、Hann 窗和三点对数抛物线频率插值。
- 20 Hz 至 20 kHz 自适应抽取：低频 8 倍、中频 2 倍、高频 1 倍。
- AIN0 的 DC、交流 RMS、Vpp、频率、THD、波形类型和 64 点压缩频谱。
- AIN0/AIN1 同频信号的相位差，并补偿顺序采样的通道时间差。
- 每通道软件增益与直流零点校准接口。
- 算法只通过 `measurement_result_publish()` 发布结果，HMI 不读取 ADS8688 DMA 缓冲区。
- TJC 串口屏基础五控件显示，USART1 采用 9600 8N1 轮询发送，不使用 UART DMA 或中断。
- 扩展 `t_dc`、`t_rms`、`t_thd` 文本帧和 64 点频谱曲线构帧接口。

题目精度、输入阻抗、单电源前端和实物频谱显示仍需按
`docs/训练题1-软件完成度与上板验收.md` 完成硬件验收，不能仅凭软件测试宣称达标。

## 硬件连接

ADS8688 使用内部 4.096 V 基准，`DAISY` 保持低电平，工作在单器件模式。

| 功能 | STM32H750 引脚 | 说明 |
| --- | --- | --- |
| ADS8688 CS | PB12 | SPI2_NSS |
| ADS8688 SCLK | PB13 | SPI2_SCK |
| ADS8688 SDO | PB14 | SPI2_MISO |
| ADS8688 SDI | PB15 | SPI2_MOSI |
| ADS8688 RST/PD | PD8 | GPIO Output |
| ADS8688 DAISY | PD9 | GPIO Output，保持低电平 |
| TJC RX | PA9 | USART1_TX -> 屏幕 RX |
| TJC TX | PA10 | USART1_RX，当前未使用 |

串口屏与核心板必须共地。ADS8688 模块的模拟电源、数字电源和输入保护必须以实际模块原理图及数据手册为准，不从 MCU 引脚配置推断。

## CubeMX 配置

### SPI2 与 DMA

- SPI2：Master，32-bit，CPOL Low，CPHA 2 Edge，硬件 NSS 输出，NSS 低有效。
- SPI2 内核时钟 129 MHz，Prescaler 8，SCLK 16.125 MHz。
- Master Inter Data Idleness：09 Cycle；CRC Disabled；FIFO Threshold 01 Data。
- SPI2_RX：DMA1 Stream0，Peripheral To Memory，Circular，Word/Word，Memory Increment。
- SPI2_TX：DMA1 Stream1，Memory To Peripheral，Circular，Word/Word，Memory Increment Disabled。
- NVIC：启用 DMA1 Stream0、DMA1 Stream1 和 SPI2 global interrupt。

### USART1 串口屏

- PA9 `USART1_TX`，PA10 `USART1_RX`。
- Asynchronous，9600，8 Bits，None，1 Stop Bit，No Flow Control。
- USART1 DMA Settings 为空，不启用 USART1 global interrupt。

不要手改 CubeMX 生成的 `MX_*_Init()`。需要调整引脚、DMA、时钟或外设参数时，在
`h7_pre.ioc` 中修改并重新生成。

## 目录结构

- `Core/User/ads8688.*`：寄存器配置、SPI2 DMA 采集、量程和诊断。
- `Core/User/ads8688_storage.*`：最新值和历史样本存储。
- `Core/User/measurement_fft.*`：DC/RMS/Vpp/频率/THD/相位/波形/频谱算法。
- `Core/User/measurement_result.*`：算法到显示的唯一结果快照边界。
- `Core/User/hmi_tjc.*`：TJC 文本和频谱命令构建、USART1 轮询刷新。
- `Core/User/system.*`：用户模块统一初始化入口。
- `Core/Src/main.c`：CubeMX 主程序，只调用用户模块入口。
- `hmi/ads8688_measure.HMI`：已烧录验证的基础串口屏源工程。
- `tests/`：源码契约和离线算法模型测试。
- `docs/训练题1-软件完成度与上板验收.md`：题目指标矩阵与实板步骤。

## 初始化与主循环

CubeMX 初始化 GPIO、DMA、SPI2、USART1 和 NVIC 后，用户初始化区只调用：

```c
system_init();
```

主循环顺序固定为：

```c
ads8688_process();
measurement_fft_process();
if (measurement_fft_hmi_refresh_allowed() != 0u)
{
    hmi_tjc_process();
}
```

默认 ADS8688 自动扫描掩码为 `0x03`，即 AIN0 和 AIN1。AIN0 是主测量通道，AIN1 用于相位差。
按当前 16.125 MHz SPI 时钟和 41 个时钟周期一帧估算，每通道原始采样率约 196.646 kSPS；实测值以调试诊断和逻辑分析仪为准。

## 测量与校准接口

读取最近一次测量和频谱：

```c
measurement_result_t result;
measurement_fft_spectrum_t spectrum;

(void)measurement_result_snapshot(&result);
(void)measurement_fft_get_spectrum(&spectrum);
```

两点校准的推荐计算为：

```text
gain = (reference_2 - reference_1) / (measured_2 - measured_1)
offset_v = reference_1 - gain * measured_1
```

写入 AIN0 校准：

```c
measurement_fft_calibration_t calibration = {
    .gain = 1.0f,
    .offset_v = 0.0f,
};

(void)measurement_fft_set_calibration(0u, &calibration);
```

未经实测不得在源码中写入猜测的校准系数。

## 编译与软件测试

在 STM32CubeIDE 中选择 `h7_pre` 的 Debug 配置，执行 `Project -> Build Project`。

离线测试：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

2026-07-14 的离线验收结果：

- 44 项契约/模型测试全部通过。
- H750 Debug 全量交叉编译通过，未出现 C 编译警告。
- `text=92400`、`data=472`、`bss=187552`。
- Flash 使用 `92872 / 131072 bytes`，约 70.9%，剩余约 38.2 KB。
- 用户函数最大静态栈 168 bytes；CMSIS FFT 内部最大静态栈 1040 bytes。

## 已完成硬件验证

- TJC 页面已烧录，PA9 -> 屏幕 RX、共地、9600、`FF FF FF` 命令链路工作正常。
- ADS8688 AIN0 和 AIN1 的直流输入已观察到正确趋势和数值。
- 串口屏固定自检模式已验证后关闭，当前使用真实测量结果。

## 待硬件验证

- 20 Hz 至 20 kHz 全频段的频率误差、幅度误差和相位误差。
- DC 误差不大于 1 mV、AC 幅度误差不大于 2 mV。
- THD 相对误差不大于 10%，三类波形在 20 Hz 至 2 kHz 的识别边界。
- 输入电阻不小于 47 kΩ、输入范围 -2.5 V 至 +2.5 V、单电源模拟前端。
- HMI 新增 DC/RMS/THD 控件后的实屏显示。
- HMI 频谱曲线控件 ID、9600 波特率下的低频刷新节奏和采集连续性。

本分支未经用户明确许可不得推送到远程仓库。
