# h7_pre ADS8688 采集工程

本工程用于 STM32 测量组实验，目标 MCU 为 `STM32H750VBT6`，开发环境为 `STM32CubeIDE 1.19.0`。当前实现 ADS8688 启动配置、自动循环扫描、单通道采集、DMA 连续读取、最新值缓存和 4096 条历史记录缓存。

## 硬件连接

ADS8688 使用内部 4.096 V 基准，`DAISY` 接地，工作在非 daisy-chain 单器件模式。

| ADS8688 信号 | STM32 引脚 | CubeIDE 功能 |
| --- | --- | --- |
| CS | PB12 | SPI2_NSS |
| SCLK | PB13 | SPI2_SCK |
| SDO | PB14 | SPI2_MISO |
| SDI | PB15 | SPI2_MOSI |
| RST/PD | PD8 | GPIO Output |
| DAISY | PD9 | GPIO Output，软件保持低电平 |

## CubeIDE/CubeMX 配置要点

- SPI2：Master，32-bit Data Size，CPOL Low，CPHA 2 Edge，硬件 NSS 输出，NSS Polarity Low。
- SPI2 时钟源：SPI123 clock source 选 PLL2，当前 SPI2 内核时钟 129 MHz，Prescaler 8，SCLK 为 16.125 MHz。
- SPI2 Advanced Parameters：Master Inter Data Idleness 为 09 Cycle，CRC Disabled，FIFO Threshold 01 Data。
- DMA：SPI2_RX 使用 DMA1 Stream0，Peripheral To Memory，Circular，Memory Increment Enable，Word/Word，FIFO Full。
- DMA：SPI2_TX 使用 DMA1 Stream1，Memory To Peripheral，Circular，Memory Increment Disable，Word/Word，FIFO Full。
- NVIC：启用 DMA1 Stream0、DMA1 Stream1、SPI2 global interrupt。
- GPIO：PD8、PD9 配置为推挽输出；PB12、PB13、PB14、PB15 由 SPI2 复用功能接管。

## TJC 串口屏显示

串口屏型号为 `TJC4827T143_011R_I_P20`，第一版显示幅度 Vpp、频率、相位、波类型和状态，不读取 ADS8688 DMA 缓冲区。测量算法只通过 `measurement_result_publish()` 发布结果；`hmi_tjc_process()` 在主循环末尾按约 4 Hz 刷新屏幕。

- 选定 `USART1`：PA9 为 TX，接串口屏 RX；PA10 为 RX，预留给后续触摸控制。它们不与 ADS8688 的 PB12-PB15、PD8、PD9 冲突。
- CubeMX 配置为 Asynchronous、9600、8N1、无校验、无流控；USART1 不使用 DMA，也不启用 USART1 global interrupt。
- 每条文本命令均以原始 `FF FF FF` 结束。固件动态文本只发送 ASCII，避免运行时中文编码问题。
- 页面控件和 USART HMI 制作步骤见 `hmi/README.md`。HMI 工程源文件完成后保存为 `hmi/ads8688_measure.HMI` 并提交。

屏幕 5 V 供电能力、PA9 到屏幕 RX 的连线、实际波特率和约 4 Hz 刷新均为待硬件验证。轮询发送会短暂占用主循环，必须与 SPI2 DMA 连续采集一同验证。

## 工程目录结构

- `Core/User/system.h`：用户代码唯一头文件入口。
- `Core/User/system.c`：用户模块初始化入口 `system_init()`。
- `Core/User/ads8688.h`、`Core/User/ads8688.c`：ADS8688 寄存器配置、DMA 采集、模式切换和诊断接口。
- `Core/User/ads8688_storage.h`、`Core/User/ads8688_storage.c`：最新值和历史记录存储。
- `Core/User/measurement_result.h`、`Core/User/measurement_result.c`：测量算法向显示模块发布幅度、频率、相位和波类型的唯一边界。
- `Core/User/hmi_tjc.h`、`Core/User/hmi_tjc.c`：淘晶驰文本指令格式化和 USART1 轮询刷新；未生成 USART1 时保持空闲。
- `Core/Src/main.c`：只在 USER CODE 区调用 `system_init()`、`ads8688_process()` 和 `hmi_tjc_process()`。
- `hmi/README.md`：USART HMI 页面控件、串口协议、CubeMX 配置与接线记录。
- `tests/test_project_contract.py`：结构、接口和文档契约测试。

## 初始化和运行

CubeMX 生成的 `MX_GPIO_Init()`、`MX_DMA_Init()`、`MX_SPI2_Init()`、`MX_USART1_UART_Init()` 和 `MX_NVIC_Init()` 完成后，`main.c` 在用户初始化区调用：

```c
system_init();
```

主循环用户区持续调用：

```c
ads8688_process();
hmi_tjc_process();
```

默认初始化后启用 8 通道自动循环扫描，全部通道默认量程为 `ADS8688_RANGE_BIPOLAR_10V24`。

## API 使用

自动循环扫描：

```c
(void)ads8688_set_auto_mode(0xffu);
```

单通道采集：

```c
(void)ads8688_set_manual_mode(0u);
```

设置量程并读取最新值：

```c
ads8688_latest_t latest;
(void)ads8688_set_channel_range(0u, ADS8688_RANGE_BIPOLAR_5V12);
if (ads8688_get_latest(0u, &latest) == ADS8688_STATUS_OK && latest.valid != 0u)
{
    /* latest.raw_code 和 latest.voltage 为通道 0 最新采样值。 */
}
```

读取历史记录：

```c
ads8688_sample_t samples[128];
uint32_t count = ads8688_read_history(samples, 128u);
```

## 编译、烧录和运行

编译：

1. 使用 STM32CubeIDE 1.19.0 打开 `C:\Users\48747\STM32CubeIDE\workspace_1.19.0\1`。
2. 选择 Debug 配置，执行 Project -> Build Project。
3. 或使用 headless build 验证 Debug 目标。

烧录：

1. 连接 ST-LINK 和目标板。
2. 确认 ADS8688 供电、地和 SPI2 连接正确。
3. 在 STM32CubeIDE 中执行 Run/Debug，将 Debug 产物烧录到目标 MCU。

运行：

1. 上电后 `system_init()` 复位 ADS8688，写入自动扫描、内部基准默认配置和量程寄存器。
2. SPI2 + DMA 连续发送 32 位 NO_OP 帧并接收转换结果。
3. `ads8688_process()` 在主循环中处理 DMA 半满/全满标志，写入最新值和历史记录。
4. `hmi_tjc_process()` 在 ADS8688 处理后读取测量结果快照；USART1 已由 CubeMX 生成时才通过轮询方式向串口屏发送。

## 性能目标

当前 SPI2 SCLK 为 16.125 MHz，单帧 32 个 SCLK，并保留 9 周期帧间空闲时间，理论总吞吐约 393 kSPS。若实板验证发现 NSS 高电平时间、DMA 中断负载或信号完整性不足，建议先回退到 200 kSPS 级别验证稳定性，再提高时钟。

历史记录环形缓冲区容量为 4096 条；缓冲区满后覆盖最旧记录，并在诊断计数中记录覆盖次数。

## 验证方法

软件验证：

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
```

实板验证建议：

- 用逻辑分析仪确认 SCLK 为 16.125 MHz。
- 确认每个 NSS 低电平窗口包含 32 个 SCLK 边沿。
- 确认 NSS 高电平和总帧率接近约 393 kSPS。
- 在双极性 ±10.24 V 量程下，将输入接地，原始码应接近 `0x8000`。
- 检查自动循环扫描通道顺序、单通道采集标签和电压换算精度。
- 确认 USART1 的 PA9 接屏幕 RX、屏幕与 MCU 共地，9600 下每条命令均以 `FF FF FF` 结束。
- 算法尚未发布结果时确认 `t_status` 为 `WAIT`；发布有效结果后确认约 4 Hz 刷新为 `LIVE`。

## 已知限制

- 尚未完成实板验证，不能宣称当前代码已经达到最终硬件采样精度和抗干扰指标。
- DMA 半缓冲每次处理 512 个样本，主循环若长期被其他任务阻塞，可能产生采样丢失并触发恢复。
- 当前默认使用内部 4.096 V 基准和全部通道 ±10.24 V 量程；外部基准或其他默认量程需要另行确认硬件后再修改。
- 本分支未执行远程 push。
- USART1 尚未通过当前 `.ioc` 生成；在 CubeMX 完成 PA9/PA10 且保存生成代码前，HMI 模块不会访问 UART。USART1 不使用 DMA 或 UART 中断；该配置及实屏刷新仍为待硬件验证。
