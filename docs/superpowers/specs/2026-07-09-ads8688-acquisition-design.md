# ADS8688 高速采集模块设计

## 1. 目标与范围

本设计在 STM32H750VBT6 与 STM32CubeIDE 1.19.0 工程中实现 ADS8688 的初始化、启动转换、转换结果读取和存储。

系统使用 ADS8688 内部 4.096 V 基准，支持以下两种可运行时切换的采集模式：

- 自动扫描：按通道使能掩码依次扫描，默认扫描全部 8 个通道；
- 单通道连续采集：持续采集指定通道。

两种模式均更新各通道最新值，并将样本写入统一的 4096 点历史环形缓冲区。目标总采样率约为 393 kSPS，设计档位记为稳定 400 kSPS；板级验证不通过时降级到 200 kSPS。

## 2. 硬件连接

| STM32H750VBT6 | 功能 | ADS8688 |
| --- | --- | --- |
| PB12 / SPI2_NSS | 硬件片选输出 | CS |
| PB13 / SPI2_SCK | 串行时钟输出 | SCLK |
| PB14 / SPI2_MISO | 串行数据输入 | SDO |
| PB15 / SPI2_MOSI | 串行数据输出 | SDI |
| PD8 / GPIO 输出 | 硬件复位和掉电控制 | RST/PD |
| PD9 / GPIO 输出，始终为低 | 单器件菊花链输入 | DAISY |

ADS8688 的 REFSEL 接低电平，选择内部基准。DAISY 通过 PD9 始终驱动为低，等效于单器件应用中直接接数字地。

## 3. CubeIDE 和 CubeMX 配置

### 3.1 SPI2

- 工作模式：Full-Duplex Master；
- NSS：Output Hardware；
- Motorola 帧格式；
- 数据宽度：32 Bits；
- MSB First；
- CPOL：Low；
- CPHA：2 Edge；
- NSSP Mode：Enabled；
- NSS Polarity：Low；
- Master SS Idleness：00 Cycle；
- Master Inter-Data Idleness：09 Cycle；
- Master Keep IO State：Enable；
- FIFO Threshold：01 Data；
- CRC、TI Mode、Receiver Auto Suspension 和 IO Swap：Disable。

SPI123 时钟源选择 PLL2P，频率为 129 MHz。SPI2 分频设为 8，SCLK 为 16.125 MHz，低于 ADS8688 的 17 MHz 上限。32 个有效时钟加 9 个帧间空闲周期时，理论总帧率约为 393.3 kSPS。实际 NSS 高电平宽度、帧间间隔和采样率必须用逻辑分析仪确认。

### 3.2 DMA

SPI2_RX 使用 DMA1 Stream0，SPI2_TX 使用 DMA1 Stream1：

- RX 和 TX 均为 Circular；
- 优先级均为 Very High；
- 外设与存储器数据宽度均为 Word；
- 外设地址均不递增；
- RX 存储器地址递增；
- TX 存储器地址不递增，以重复发送 32 位 NO_OP 零字；
- FIFO Enabled，Threshold Full；
- Peripheral Burst 和 Memory Burst 均为 Single；
- 不启用 DMAMUX 同步或事件。

启用 DMA1 Stream0、DMA1 Stream1 和 SPI2 全局中断，抢占优先级均为 5。生成中断处理函数并调用对应 HAL 处理函数。

### 3.3 GPIO

- PD8：推挽输出、默认高电平，用于 RST/PD；
- PD9：推挽输出、默认低电平，并在运行期间保持低电平；
- PB12 至 PB15：SPI2 复用功能、无上下拉、高速或极高速输出速度。

## 4. 软件架构

所有用户 `.c` 和 `.h` 文件放在 `Core/User/`：

- `system.c/.h`：统一用户初始化入口、统一头文件入口及跨模块声明；
- `ads8688.c/.h`：命令和寄存器访问、模式切换、量程配置、DMA 控制、数据解析及电压换算；
- `ads8688_storage.c/.h`：最新通道数据和 4096 点历史环形缓冲区。

`main.c` 只在用户包含区加入 `#include "system.h"`，用户初始化区只调用 `system_init()`，主循环用户区只调用 `ads8688_process()`。

## 5. 初始化流程

1. `system_init()` 确认 PD9 为低电平。
2. 将 RST/PD 拉低超过 400 ns，使 ADS8688 进入硬件掉电并复位程序寄存器。
3. 将 RST/PD 拉高，等待至少 15 ms，使内部基准和模拟电路稳定。
4. 配置自动扫描掩码，默认值为 `0xff`。
5. 配置所有通道量程，默认均为 ±10.24 V。
6. 配置 16 位转换结果输出格式。
7. 回读并验证全部已写程序寄存器。
8. 发送 `AUTO_RST` 命令，将通道序列同步到最低启用通道。
9. 启动 SPI2 TX/RX 循环 DMA。

初始化任一步骤失败时最多重试 3 次。连续失败后进入错误状态，不启动采集。

## 6. 采集与数据流

SPI2 每个 32 位帧的前 16 位发送命令，持续采集时发送 NO_OP；ADS8688 在后 16 位输出转换结果。PB12 由 SPI2 硬件自动生成每帧 NSS 脉冲。

RX DMA 暂存区包含 1024 个 32 位字，分为两个 512 字半区：

1. DMA 半满回调只给 `ads8688_dma_half_flag` 赋值；
2. DMA 完成回调只给 `ads8688_dma_full_flag` 赋值；
3. SPI 或 DMA 错误回调只给 `ads8688_error_flag` 赋值；
4. `ads8688_process()` 在主循环中处理相应半区，提取 16 位原始码、确定通道号、换算电压并存储结果。

自动扫描模式按启用掩码在软件中跟踪通道号。单通道模式为所有样本附加固定的选定通道号。发生传输错误或分区顺序异常时，停止 DMA、记录错误、重新发送 AUTO_RST 或手动通道命令，再恢复 DMA，以避免结果与通道错配。

## 7. 数据结构与存储

历史记录固定为 8 字节：

```c
typedef struct
{
    uint32_t sample_index;
    uint16_t raw_code;
    uint8_t channel;
    uint8_t reserved;
} ads8688_sample_t;
```

4096 条历史记录约占 32 KB RAM。缓冲区写满后覆盖最旧记录，并增加覆盖计数，不停止实时采集。

每个通道还保存：

- 最新 16 位原始码；
- 按当前量程换算的浮点电压；
- 对应采样序号；
- 数据有效标志。

双极性量程使用直二进制换算，默认 ±10.24 V 量程下 `0x8000` 约对应 0 V。单极性量程从 0 V 开始换算。

## 8. 公共接口

模块至少提供以下接口：

```c
ads8688_status_t ads8688_set_auto_mode(uint8_t channel_mask);
ads8688_status_t ads8688_set_manual_mode(uint8_t channel);
ads8688_status_t ads8688_set_channel_range(uint8_t channel,
                                            ads8688_range_t range);
void ads8688_process(void);
ads8688_status_t ads8688_get_latest(uint8_t channel,
                                    ads8688_latest_t *latest);
uint32_t ads8688_read_history(ads8688_sample_t *samples,
                              uint32_t max_count);
void ads8688_clear_history(void);
```

模式或量程切换过程先停止 DMA，完成寄存器写入和回读验证，重建通道跟踪状态，再恢复 DMA。切换模式不清空已有历史数据。

## 9. 错误处理与诊断

模块提供状态值及以下诊断计数：

- 初始化失败次数；
- SPI/DMA 错误次数；
- DMA 分区顺序异常和丢样次数；
- 历史缓冲区覆盖次数；
- 自动恢复次数。

非法通道、非法量程、空指针和未初始化调用均返回明确错误码。运行时传输错误由主循环处理，不在中断中复位器件、打印或执行协议逻辑。

若未来启用 Cortex-M7 D-Cache，DMA 缓冲区必须放在非缓存区，或在 DMA 与 CPU 交接时增加正确的缓存维护；当前生成工程未启用 D-Cache。

## 10. 验证方案

### 10.1 软件验证

- 编译 Debug 配置并检查全部警告；
- 测试命令字和程序寄存器访问字生成；
- 测试五种量程的边界码和中点电压换算；
- 测试自动扫描掩码和单通道通道标记；
- 测试 4096 点环形缓冲区写入、读取、回绕和覆盖；
- 测试模式切换及错误恢复状态机。

### 10.2 板级验证

- 用逻辑分析仪确认 SCLK 约为 16.125 MHz；
- 确认每个 NSS 低电平帧包含 32 个 SCLK；
- 确认 NSS 高电平时间满足 ADS8688 最小要求；
- 测量实际总采样率，目标约为 393 kSPS；
- 默认双极性量程下将输入短接至地，原始码应在 `0x8000` 附近；
- 对各通道施加已知电压，检查原始码、换算电压和通道顺序；
- 分别验证自动扫描、单通道连续采集、模式切换和缓冲区覆盖。

无法在无目标板、示波器或逻辑分析仪的当前环境中完成板级时序和模拟精度验证，交付时必须明确列为未验证项。

## 11. 资料依据

- Texas Instruments, *ADS8684, ADS8688 16-Bit, 4- and 8-Channel, 500-kSPS Analog-to-Digital Converters*，SBAS582C；
- STMicroelectronics, *STM32H750VB/STM32H750ZB/STM32H750IB Datasheet*；
- STM32Cube FW_H7 V1.12.1 HAL 驱动和 STM32CubeIDE 1.19.0 生成配置。
