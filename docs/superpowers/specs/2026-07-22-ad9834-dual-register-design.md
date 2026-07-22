# AD9834 双频率与双相位寄存器设计

## 目标

扩展现有 AD9834 底层驱动，使应用能够分别写入 `FREQ0`、`FREQ1`、`PHASE0`
和 `PHASE1`，并继续通过现有 FSELECT、PSELECT 引脚选择当前输出组合。所有现有
函数、枚举值及默认输出行为保持兼容。

本功能只扩展 `Core/User/ad9834.c/.h` 及相关文档，不修改 `.ioc`、CubeMX 生成代码、
GPIO、SPI、DMA、NVIC 或时钟配置。

## 硬件依据

根据 Analog Devices AD9834 Rev. D 数据手册：

- `FREQ0` 和 `FREQ1` 均为 28 位频率寄存器；
- `PHASE0` 和 `PHASE1` 均为 12 位相位寄存器；
- 当前控制字设置 `PIN/SW=1`，因此 PB14/FSELECT 和 PD8/PSELECT 决定输出使用的
  频率、相位寄存器；
- FSELECT 低电平选择 `FREQ0`，高电平选择 `FREQ1`；
- PSELECT 低电平选择 `PHASE0`，高电平选择 `PHASE1`。

现有引脚映射保持不变：PB12/FSYNC、PB13/SPI2_SCK、PB15/SPI2_MOSI、
PB14/FSELECT、PD8/PSELECT。

## 公共接口

新增两个通用写入接口：

```c
ad9834_status_t ad9834_set_frequency_register_hz(
    ad9834_frequency_register_t frequency_register,
    uint32_t frequency_hz);

ad9834_status_t ad9834_set_phase_register_degrees(
    ad9834_phase_register_t phase_register,
    uint16_t phase_degrees);
```

频率接口根据枚举选择 `FREQ0` 或 `FREQ1` 地址，并按照现有 B28 模式依次发送低
14 位和高 14 位。频率字继续使用：

```text
FTW = round(frequency_hz * 2^28 / AD9834_MCLK_HZ)
```

相位接口接受 `0~359` 度整数，并换算为 12 位相位字：

```text
PHASE = round(phase_degrees * 4096 / 360)
```

换算使用整数运算。由于输入最大为 359 度，结果始终位于 `0~4095`。

现有接口全部保留：

- `ad9834_init()`；
- `ad9834_set_frequency_hz()`；
- `ad9834_select_frequency_register()`；
- `ad9834_select_phase_register()`；
- `ad9834_calculate_tuning_word()`。

`ad9834_set_frequency_hz()` 继续表示写入 `FREQ0`，内部调用新的通用频率接口，
从而维持现有 `dds_control` 调用方的行为。

## 初始化与选择行为

`ad9834_init(initial_frequency_hz)` 的初始化顺序调整为：

1. 清零诊断信息并将 FSYNC 恢复为高电平；
2. 选择 `FREQ0 + PHASE0`；
3. 写入 RESET 控制字；
4. 将 `initial_frequency_hz` 分别写入 `FREQ0` 和 `FREQ1`；
5. 将 `0` 度分别写入 `PHASE0` 和 `PHASE1`；
6. 写入 RUN 控制字；
7. 再次确认选择 `FREQ0 + PHASE0` 并标记初始化完成。

这样旧代码的默认输出不变，同时在应用尚未主动设置备用寄存器时，切换到备用寄存器
也不会输出未初始化内容。

写寄存器与选择输出相互独立。写入 `FREQ1` 不会自动切换 FSELECT，写入 `PHASE1`
也不会自动切换 PSELECT。应用需要显式调用现有选择函数完成无额外 SPI 传输的切换。

## 状态与诊断

在现有 `ad9834_diagnostics_t` 末尾追加以下信息，不删除或重命名原字段：

- 两个频率寄存器最近成功写入的频率值；
- 两个频率寄存器最近成功写入的 28 位频率字；
- 两个相位寄存器最近成功写入的整数角度；
- 两个相位寄存器最近成功写入的 12 位相位字；
- 最近成功写入的频率寄存器编号；
- 最近成功写入的相位寄存器编号。

原 `output_frequency_hz` 和 `tuning_word` 字段继续记录最近一次成功频率写入，以免
破坏现有调试观察习惯。原选择诊断字段继续反映 FSELECT、PSELECT 的实际输出电平。

## 错误处理

在现有状态枚举末尾追加非法寄存器和非法相位状态，保留现有枚举数值：

- 非 `ad9834_frequency_register_0/1` 的值返回非法寄存器状态；
- 非 `ad9834_phase_register_0/1` 的值返回非法寄存器状态；
- 频率为 0 或超过 `AD9834_MAX_OUTPUT_HZ` 时沿用非法频率状态；
- 相位大于 359 度时返回非法相位状态；
- 参数校验失败时不拉低 FSYNC、不发送 SPI 数据、不修改成功写入诊断；
- SPI 任一字写入失败时立即返回现有 SPI 错误状态，并保留已经发生的底层错误计数。

选择函数保持 `void` 签名以兼容旧代码。传入非法枚举时不改变 GPIO 和选择诊断，避免
把非法值静默映射到寄存器 0。

## 文件影响

- `Core/User/ad9834.h`：声明新接口、错误状态及扩展诊断字段，补全中文接口说明；
- `Core/User/ad9834.c`：实现双频率、双相位写入及兼容包装函数；
- `tests/`：增加寄存器地址、换算边界、兼容接口和初始化顺序的契约测试；
- `README.md`：说明四个寄存器的设置与选择调用方法。

不修改 `dds_control`。它继续调用 `ad9834_set_frequency_hz()` 并使用 `FREQ0`，因此
现有自动本振补偿流程不受影响。

## 验证标准

- `FREQ0` 数据字使用 `0x4000` 地址，`FREQ1` 数据字使用 `0x8000` 地址；
- `PHASE0` 数据字使用 `0xC000` 地址，`PHASE1` 数据字使用 `0xE000` 地址；
- 0 度换算为 0，359 度换算后不超过 4095；
- 非法寄存器、非法频率和非法相位均不触发 SPI 写入；
- 原 `ad9834_set_frequency_hz()` 仍只写入 `FREQ0`；
- 初始化后两组频率相同、两组相位均为 0 度，最终选择 `FREQ0 + PHASE0`；
- 现有 Python 契约测试和新增测试全部通过；
- STM32CubeIDE Debug 工程构建成功；
- README 与接口注释完整记录引脚、依赖、初始化及调用方法。
