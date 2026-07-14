# STM32H750 片上双 ADC 信号分析器

本工程是训练题固件从 ADS8688 迁移到 STM32H750VBT6 片上 ADC1/ADC2 的独立副本，开发环境为 `STM32CubeIDE 1.19.0`。原工程 `h7_pre` 保留 ADS8688 版本，不在本分支删除或覆盖。

目标链路：

`ADC1 + ADC2 同步采样 -> 8192 点 Q15 FFT -> measurement_result -> TJC 串口屏`

当前目录：

`C:\Users\48747\STM32CubeIDE\workspace_1.19.0\h7_onchip_adc`

当前分支：`momo/onchip-adc`。未经用户明确许可不得 push。

## 当前状态

已经完成的软件部分：

- 新增 `adc_dual` 采集边界，接收 ADC1 低 16 位、ADC2 高 16 位的 32 位 DMA packed word。
- 从本机 STM32Cube FW H7 V1.12.1 补入与工程固件包一致的 HAL ADC/LL ADC 驱动文件。
- 1024 个 `uint32_t` DMA 缓冲区，32 字节对齐；主循环读取半缓冲区前执行 D-Cache invalidate。
- DMA 半满、满帧、错误回调只置位各自标志，拆包和统计均在主循环执行。
- FFT 改为 `measurement_fft_ingest_pair(ch1, ch2)` 同步样本对接口，不再做 ADS8688 通道轮询延迟补偿。
- 保留 8192 点 CMSIS-DSP Q15 RFFT、Hann 窗、频率插值、相位、THD、波形识别和 64 点频谱接口。
- 前级尚未确定时，频率、相位和波形仍可分析；Vpp、DC、RMS 只有校准参数有效后才发布。
- 算法仍只通过 `measurement_result_publish()` 发布快照，HMI 不读取 ADC DMA 缓冲区。
- 保留 TJC4827T143_011R_I_P20 串口屏：USART1、PA9/PA10、9600 8N1、阻塞轮询发送，不使用 USART DMA 或 USART1 中断。
- `main.c` 初始化区只调用 `system_init()`，主循环只调用 `system_process()`。

尚未完成的硬件生成部分：

- `h7_onchip_adc.ioc` 目前仍保存复制时的 SPI2/ADS8688 配置。
- 必须先按 `docs/片上双ADC-CubeMX配置步骤.md` 完成 CubeMX 配置并生成代码，才可编译和上板测试片上 ADC。
- 前级放大、偏置、保护、抗混叠滤波和电压校准参数等待参考电路后确定。

## 计划引脚

| 功能 | STM32H750 引脚 | 说明 |
| --- | --- | --- |
| CH1 模拟输入 | PC4 | ADC1_INP4，待 CubeMX 配置 |
| CH2 模拟输入 | PB1 | ADC2_INP5，待 CubeMX 配置 |
| TJC RX | PA9 | USART1_TX -> 屏幕 RX |
| TJC TX | PA10 | USART1_RX，第一版暂不连接 |
| SWDIO/SWCLK | 原配置 | 保持不变 |

释放原 ADS8688 使用的 PB12、PB13、PB14、PB15、PD8、PD9。最终接线以生成后的 `h7_onchip_adc.ioc` 和实际前级电路为准。

## CubeMX 摘要

- 禁用 SPI2，并删除 SPI2 RX/TX DMA 和 SPI2 中断。
- PC4 配置为 ADC1_INP4，PB1 配置为 ADC2_INP5。
- ADC1/ADC2：16 bit、Single-ended、每个 ADC 一个 Rank、8.5 Cycles、关闭连续/扫描/过采样。
- ADC1 为 Master、ADC2 为 Slave，Dual Regular Simultaneous，DMA 数据格式 32 bits，`OVRMOD=1`。
- TIM2：APB1 定时器时钟 240 MHz，PSC=0、ARR=479、TRGO=Update，触发率 500 kHz。
- ADC1 外部触发为 TIM2 TRGO Rising Edge；ADC2 不配置独立触发。
- ADC1 DMA：DMA1 Stream0、Peripheral to Memory、Circular、Word/Word、Memory Increment、Very High、IRQ 优先级 5。
- ADC 异步时钟目标约 32.25 MHz，分频 `/4`。
- USART1 保持 9600 8N1、无 DMA、无 USART1 global interrupt。

详细点击路径见 `docs/片上双ADC-CubeMX配置步骤.md`。不要手改 CubeMX 生成的 `MX_*_Init()`。

## 目录结构

- `Core/User/adc_dual.*`：片上双 ADC DMA、D-Cache、拆包和采集统计。
- `Core/User/measurement_fft.*`：DC/RMS/Vpp/频率/THD/相位/波形/频谱算法。
- `Core/User/measurement_result.*`：算法到显示的唯一结果快照边界。
- `Core/User/hmi_tjc.*`：TJC 命令构帧和 USART1 轮询刷新。
- `Core/User/system.*`：用户模块统一初始化和主循环入口。
- `Core/User/ads8688*`：迁移期保留的历史实现，不属于新活动采集链路。
- `hmi/ads8688_measure.HMI`：已验证的串口屏源工程，文件名暂不改以保留历史。
- `tests/`：源码契约和离线算法模型测试。

## 初始化与主循环

CubeMX 生成的 HAL 初始化完成后：

```c
system_init();
```

主循环只调用：

```c
system_process();
```

`system_process()` 内部依次处理 ADC DMA、FFT 状态和允许时的 HMI 刷新。FFT 收齐一帧后暂停 TIM2，完成分析和 9600 波特率屏幕发送后再开始下一帧。

## 电压校准

前级传递函数确认前，默认校准 `valid=0`，屏幕幅度显示 `--`，不得把原始码宣称为真实输入电压。

确认前级后，每个通道写入线性参数：

```c
measurement_fft_calibration_t calibration = {
    .volts_per_code = 5.0f / 65536.0f,
    .offset_v = -2.5f,
    .valid = 1u,
};

(void)measurement_fft_set_calibration(0u, &calibration);
```

以上数值仅示范接口，不是最终硬件参数。最终参数必须根据前级比例和两点或多点实测标定计算。

## 测试与构建

离线测试：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

完成 CubeMX 配置并生成代码后，在 STM32CubeIDE 选择 `h7_onchip_adc`，执行 `Project > Clean...` 和 `Project > Build Project`。验收要求：0 errors、0 用户源码 warnings，Flash 不超过 128 KiB。

2026-07-14 的迁移前置验证：

- 49 项源码契约和 NumPy 合成信号模型测试全部通过。
- 旧 `.ioc` 配置下的占位构建为 0 errors、0 warnings，Flash 为 82,096 bytes。
- 强制启用 ADC/TIM HAL 后，`adc_dual.c` 和补入的 H7 V1.12.1 ADC HAL/LL 源码均通过 `-Wall -Wextra -Werror` 编译。
- 以上不替代 CubeMX 生成后的最终构建；实际 ADC 固件大小和链接结果仍待重新验收。

## 待硬件验证

- ADC1/ADC2 packed word 顺序与 PC4/PB1 实际通道一致。
- 两路中点直流码、满量程裕量、噪声和削顶统计。
- 同一正弦信号接两路时的相位零点。
- 20 Hz 至 20 kHz 的频率、幅度、相位、THD 和波形识别误差。
- 前级输入阻抗、保护、偏置、抗混叠滤波和校准。
- 片上双 ADC 与 9600 波特率 HMI 分帧运行的长期稳定性。
