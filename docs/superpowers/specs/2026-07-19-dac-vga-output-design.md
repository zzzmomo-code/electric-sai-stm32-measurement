# DAC 与 VGA 增益控制设计

## 目标

使用 STM32H743VIT6 的 DAC1 OUT1，经片内 OPAMP1 电压跟随器从 PC4 输出六档直流控制电压。外部电路将该电压转换为 VGA 控制电压 `VG`，驱动根据档位给出理论 VGA 差分电压增益。

## 硬件与 CubeMX 前提

- 外部晶振为 25 MHz；PLL1 配置为 `25 MHz / 5 * 192 / 2 = 480 MHz`。
- DAC1 Channel 1 使用内部连接，触发源为 Software/None，采样保持关闭。
- OPAMP1 使用 Follower 模式，正输入连接 DAC Channel，输出引脚为 PC4。
- DAC1 和 OPAMP1 的 `MX_*_Init()` 仍由 CubeMX 生成并在 `system_init()` 之前调用。
- 用户驱动不修改 `.ioc`、CubeMX 生成的外设初始化代码或 PC4 配置。

## 文件与集成位置

- 新增 `Core/User/dac_output.h` 和 `Core/User/dac_output.c`。
- `system.h` 统一包含 `dac_output.h` 以及驱动所需的 CubeMX `dac.h`、`opamp.h`。
- `system_init()` 调用 `dac_output_init()`；不在 `main.c` 添加新的用户业务逻辑。
- `dac_output.c` 只包含 `system.h`，符合用户代码统一头文件入口规则。

## 可配置宏

头文件提供下列配置项，名称可在实施时按现有工程风格微调：

- 六档标称电压，单位为 mV：`0、660、1320、1980、2640、3300`。
- `DAC_OUTPUT_REFERENCE_MV`：DAC 参考电压，默认 `3300 mV`。
- `DAC_OUTPUT_RF_OHM`：外部反馈电阻，默认 `10000 ohm`。
- `DAC_OUTPUT_RG_OHM`：外部增益电阻，默认 `10000 ohm`。
- `DAC_OUTPUT_GAIN_CALIBRATION`：DAC 比例校准系数，默认 `1.0f`。
- `DAC_OUTPUT_OFFSET_CALIBRATION_V`：DAC 偏移校准电压，默认 `0.0f`。

校准后的目标电压为：

```text
calibrated_voltage = nominal_voltage * gain_calibration + offset_calibration
```

换算 DAC 码值前将校准电压限制到 `0` 至参考电压范围，12 位 DAC 码值按最近整数计算并限制到 `0..4095`。

校准宏只影响实际写入 DAC 的码值；VG 与 VGA 理论增益函数以校准后的目标电压计算，从而反映配置预期的控制电压。它们不代表万用表实测值，硬件误差仍需通过测量确定校准系数。

## 公式与理论档位

外部控制电压关系：

```text
VG = (20 / 33) * VDAC - 1
```

VGA 输出关系：

```text
VOUT = ((+VIN) - (-VIN)) * (1 + VG) * RF / RG
```

驱动返回的 VGA 增益定义为输出差分幅度与输入差分幅度之比：

```text
gain = (1 + VG) * RF / RG
```

默认无校准且 `RF = RG = 10 kohm` 时，各档理论结果为：

| 档位 | VDAC/V | VG | VGA 增益 |
|---:|---:|---:|---:|
| 0 | 0.00 | -1.0 | 0.0 |
| 1 | 0.66 | -0.6 | 0.4 |
| 2 | 1.32 | -0.2 | 0.8 |
| 3 | 1.98 | 0.2 | 1.2 |
| 4 | 2.64 | 0.6 | 1.6 |
| 5 | 3.30 | 1.0 | 2.0 |

## 驱动接口与行为

- `dac_output_init()`：启动 OPAMP1 和 DAC1 Channel 1，成功后默认输出 0 档；返回初始化状态。
- `dac_output_set_level(uint8_t level)`：设置 `0..5` 档；成功后记录当前档位。
- `dac_output_get_level()`：返回最近一次成功设置的档位。
- `dac_output_get_voltage(uint8_t level, float *voltage_v)`：返回指定档位校准后的理论 DAC 电压。
- `dac_output_get_vg(uint8_t level, float *vg)`：返回指定档位的理论 VG。
- `dac_output_get_vga_gain(uint8_t level, float *gain)`：返回指定档位的理论 VGA 差分电压增益。

所有需要输出参数的函数检查空指针。非法档位、空指针或 HAL 调用失败均返回明确的失败状态；失败时不更新当前档位。驱动不使用中断，不增加共享中断标志。

## 初始化顺序

`main.c` 中 CubeMX 生成的初始化顺序保持为 DAC1 在前、OPAMP1 在后。进入 `system_init()` 后，驱动先启动 OPAMP1，再设置 DAC 0 档并启动 DAC Channel 1，避免启动瞬间出现非预期高控制电压。若 HAL 启动失败，`dac_output_init()` 返回失败，其他测量模块仍按现有顺序初始化；错误状态可由调试器检查，不在中断或主循环中阻塞。

## 测试与验证

实施采用测试先行：

1. 主机端契约测试先验证六档宏、接口声明、统一头文件集成和 `system_init()` 调用，确认在实现前失败。
2. 计算测试验证默认校准下六档 DAC 码值约为 `0、819、1638、2457、3276、4095`，并验证对应 VG 和 VGA 增益。
3. 测试增益/偏移校准、上下限钳位、非法档位、空指针和 HAL 失败时不更新档位。
4. 使用 STM32CubeIDE 当前 Debug 配置编译，确认 HAL DAC/OPAMP 依赖和用户文件构建路径正确。
5. 烧录后用万用表或示波器测量 PC4 六档电压。由于 DAC 参考、OPAMP 输出摆幅和负载影响，3.3 V 档可能无法达到理想值；根据实测值调整校准宏。

## 非目标

- 不实现自动闭环校准或保存校准参数到 Flash。
- 不修改外部 VGA 电路参数。
- 不在本次工作中调整 CubeMX 时钟或外设配置。
- 不承诺 PC4 在实际负载下精确输出电源轨电压。
