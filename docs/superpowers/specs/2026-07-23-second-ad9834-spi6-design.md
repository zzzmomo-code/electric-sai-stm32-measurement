# SPI6 第二块 AD9834 独立驱动设计

## 目标

在不改变第一块 AD9834 及现有 `dds_control` 行为的前提下，为第二块 AD9834
增加一套独立底层驱动。第二块器件使用 SPI6 和专用控制引脚，上电后由
`system_init()` 初始化为 900 kHz，并提供双频率、双相位寄存器读写选择能力。

保留用户在第一块驱动中尚未提交的 `DDS_SetFrequency()`。第二块驱动增加行为对应的
`DDS2_SetFrequency()`，用于交替写入非活动频率寄存器后切换输出。

## 硬件与 CubeIDE 配置依据

第二块 AD9834 的板载 MCLK 为 75 MHz，引脚映射如下：

- PB3：SPI6_SCK；
- PB5：SPI6_MOSI；
- PD5：DDS2_FSYNC，空闲高电平；
- PD6：DDS2_FS，对应 FSELECT，默认低电平；
- PD7：DDS2_PS，对应 PSELECT，默认低电平；
- PB4：DDS2_RST，对应硬件 RESET，正常运行时为低电平。

已生成的 SPI6 配置为主机单向发送、16 位、MSB 优先、CPOL 高、第一边沿采样、
软件 NSS、120 MHz 外设时钟四分频，即 30 Mbit/s。FSYNC 由 PD5 手动控制，不使用
硬件 NSS。SPI6 在 `main.c` 中先于 `system_init()` 初始化。

本功能不再修改 `.ioc`、Pinout、Clock Configuration、GPIO、DMA、NVIC 或 CubeMX
生成的初始化代码。

## 文件与模块边界

新建 `Core/User/ad9834_2.c` 和 `Core/User/ad9834_2.h`。第二块驱动具有独立的
状态、SPI6 写入函数和 GPIO 控制，不调用或修改第一块 `ad9834.c/.h` 的内部实现。

`Core/User/system.h` 统一包含 `ad9834_2.h`。`Core/User/system.c` 的
`system_init()` 调用 `ad9834_2_init(900000u)`，不在 `main.c` 增加用户初始化逻辑。
第二块驱动不加入 `system_process()`，因为所有接口均为调用即完成的阻塞式底层操作。

## 公共接口

第二块驱动提供以下独立接口：

```c
ad9834_2_status_t ad9834_2_init(uint32_t initial_frequency_hz);
ad9834_2_status_t ad9834_2_set_frequency_hz(uint32_t frequency_hz);
ad9834_2_status_t ad9834_2_set_frequency_register_hz(
    ad9834_2_frequency_register_t frequency_register,
    uint32_t frequency_hz);
ad9834_2_status_t ad9834_2_set_phase_register_degrees(
    ad9834_2_phase_register_t phase_register,
    uint16_t phase_degrees);
void ad9834_2_select_frequency_register(
    ad9834_2_frequency_register_t frequency_register);
void ad9834_2_select_phase_register(
    ad9834_2_phase_register_t phase_register);
uint32_t ad9834_2_calculate_tuning_word(uint32_t frequency_hz);
ad9834_2_status_t DDS2_SetFrequency(uint32_t freq);
```

除用户明确指定的兼容接口 `DDS2_SetFrequency()` 外，所有新增函数和变量均使用
小写下划线命名。`DDS2_SetFrequency()` 与第一块的 `DDS_SetFrequency()` 保持直观
对应，不替换任何小写底层接口。

`ad9834_2_set_frequency_hz()` 固定写入 FREQ0，用于与第一块基础 API 对应。
通用频率和相位接口分别支持 FREQ0/FREQ1 与 PHASE0/PHASE1。写入寄存器不会自动切换
FSELECT 或 PSELECT。

## 数据格式与 SPI 时序

第二块器件继续使用 75 MHz MCLK，28 位频率字按下式四舍五入：

```text
FTW = round(frequency_hz * 2^28 / 75000000)
```

每次写入一个 16 位控制字或数据字时：

1. 将 PD5/DDS2_FSYNC 拉低；
2. 调用 `HAL_SPI_Transmit(&hspi6, ..., 1u, 10u)` 发送一个 16 位字；
3. 无论 HAL 返回成功或失败，都将 PD5/DDS2_FSYNC 恢复为高电平；
4. 成功时增加写入计数，失败时增加错误计数并返回 SPI 错误状态。

FREQ0、FREQ1 数据字地址分别为 `0x4000`、`0x8000`，每个 28 位频率字依次发送低
14 位和高 14 位。PHASE0、PHASE1 地址分别为 `0xC000`、`0xE000`，相位接口接受
0 至 359 度整数并换算为 12 位相位字。

## 初始化与 RESET

`ad9834_2_init(900000u)` 执行以下顺序：

1. 清零第二块驱动诊断信息；
2. 将 DDS2_FSYNC 置高，DDS2_FS 和 DDS2_PS 置低；
3. 将 PB4/DDS2_RST 置高，使输出进入硬件复位状态；
4. 写入带软件 RESET 的 AD9834 控制字；
5. 将 FREQ0 和 FREQ1 都写为 900 kHz；
6. 将 PHASE0 和 PHASE1 都写为 0 度；
7. 写入运行控制字；
8. 将 PB4/DDS2_RST 拉低；
9. 再次选择 FREQ0 和 PHASE0，并标记初始化成功。

若任一 SPI 写入失败，初始化立即返回错误；退出前保证 DDS2_FSYNC 为高电平，
DDS2_RST 保持高电平，使器件不会带着不完整配置输出。下一次调用
`ad9834_2_init()` 可重新执行完整初始化。

## 交替频率更新

`DDS2_SetFrequency(freq)` 保存第二块当前活动频率寄存器。每次调用时先把新频率写入
非活动寄存器；只有两个 16 位频率数据字均发送成功后，才切换 PD6/DDS2_FS 并更新
活动寄存器状态。SPI 写入失败时保持当前输出和活动寄存器不变。

该接口不改变相位寄存器选择。非法频率沿用底层参数检查，不发送 SPI 数据，也不切换
DDS2_FS。

## 状态、诊断与错误处理

第二块使用独立的 `ad9834_2_diagnostics`，字段与第一块诊断能力对应，包括：

- 最近成功设置的频率和频率字；
- SPI 成功写入数、错误数、最近数据字和最近 HAL 状态；
- 初始化状态及当前频率、相位寄存器选择；
- 两组频率、频率字、相位角度和相位字；
- 最近成功写入的频率、相位寄存器编号。

频率为 0 或超过 30 MHz、相位超过 359 度、寄存器枚举非法时返回明确错误状态。
参数错误不得改变 GPIO、不得发起 SPI、不得修改成功写入诊断。所有操作均在主循环或
初始化上下文调用，不在中断中执行。

## 测试与文档

扩展 `tests/test_dds_contract.py`，使用独立主机桩验证：

- `.ioc` 和生成代码中的 SPI6 参数、引脚及初始化顺序；
- 75 MHz 频率字计算和四个寄存器地址；
- 初始化的 RESET、FSYNC、FSELECT、PSELECT 顺序和最终状态；
- SPI6 每个 FSYNC 低电平窗口只发送一个 16 位字；
- 非法频率、相位和寄存器不产生 SPI 或 GPIO 副作用；
- SPI 失败后 FSYNC 恢复高电平，初始化失败时 RESET 保持高电平；
- `DDS2_SetFrequency()` 只在完整写入成功后切换非活动寄存器；
- 第一块现有 `DDS_SetFrequency()` 内容仍被保留。

更新 `README.md`，记录第二块 AD9834 的用途、引脚、SPI6 配置、900 kHz 初始化、
独立 API、`DDS2_SetFrequency()` 示例、限制与示波器/逻辑分析仪验证方法。

## 完成标准

- 所有新增 `.c/.h` 文件位于 `Core/User/`，并具有完整中文模块说明和函数注释；
- `system.h` 包含第二块驱动头文件，`system_init()` 完成唯一初始化调用；
- `main.c` 用户初始化区和主循环结构保持不变；
- 第一块 AD9834、`dds_control` 及用户的 `DDS_SetFrequency()` 行为不变；
- 主机契约测试全部通过；
- STM32CubeIDE Debug 工程构建成功；
- README 与实际代码、引脚及配置一致；
- 只提交本功能涉及的文件，不夹带现有无关工作区改动，不执行远程 push。
