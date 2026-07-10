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

## 工程目录结构

- `Core/User/system.h`：用户代码唯一头文件入口。
- `Core/User/system.c`：用户模块初始化入口 `system_init()`。
- `Core/User/ads8688.h`、`Core/User/ads8688.c`：ADS8688 寄存器配置、DMA 采集、模式切换和诊断接口。
- `Core/User/ads8688_storage.h`、`Core/User/ads8688_storage.c`：最新值和历史记录存储。
- `Core/Src/main.c`：只在 USER CODE 区调用 `system_init()` 和 `ads8688_process()`。
- `tests/test_project_contract.py`：结构、接口和文档契约测试。

## 初始化和运行

CubeMX 生成的 `MX_GPIO_Init()`、`MX_DMA_Init()`、`MX_SPI2_Init()` 和 `MX_NVIC_Init()` 完成后，`main.c` 在用户初始化区调用：

```c
system_init();
```

主循环用户区持续调用：

```c
ads8688_process();
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

1. 使用 STM32CubeIDE 1.19.0 打开 `D:\CubeIDE\h7_pre`。
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

## 已知限制

- 尚未完成实板验证，不能宣称当前代码已经达到最终硬件采样精度和抗干扰指标。
- DMA 半缓冲每次处理 512 个样本，主循环若长期被其他任务阻塞，可能产生采样丢失并触发恢复。
- 当前默认使用内部 4.096 V 基准和全部通道 ±10.24 V 量程；外部基准或其他默认量程需要另行确认硬件后再修改。
- 本分支未执行远程 push。
