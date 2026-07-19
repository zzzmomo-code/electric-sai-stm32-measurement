# DAC 与 VGA 六档增益控制设计

## 目标

为 STM32H743VIT6 工程增加 DAC1 OUT1 六档直流输出控制，并依据档位计算外接 VGA 的理论差分增益。系统上电默认选择第 0 档；非法档位不得改变当前 DAC 输出。

## 已有硬件与 CubeMX 配置

- MCU：STM32H743VIT6。
- 开发环境：STM32CubeIDE 1.19.0。
- DAC 输出引脚：PA4，对应 DAC1_OUT1。
- DAC 通道：DAC_CHANNEL_1。
- 触发方式：DAC_TRIGGER_NONE。
- 输出缓冲：开启。
- DAC1 已由 CubeMX 生成 `MX_DAC1_Init()`，且在 `system_init()` 之前调用。
- 本功能不修改 `.ioc`、`Core/Inc/dac.h`、`Core/Src/dac.c` 或其他 CubeMX 生成代码。

## 六档定义

档位、电压、片上 DAC 自动换算结果、控制电压和默认理论增益如下。片上 DAC 固定使用 HAL 的 12 位右对齐模式，数字码由目标电压和 3.3 V 参考电压自动换算，不作为用户可调参数；默认 `Rf = RG = 1.0f`。

| 档位 | DAC 电压 VDAC | DAC 码 | VG | VGA 增益 |
|---:|---:|---:|---:|---:|
| 0 | 0.00 V | 0 | -1.0 V | 0.0 |
| 1 | 0.66 V | 819 | -0.6 V | 0.4 |
| 2 | 1.32 V | 1638 | -0.2 V | 0.8 |
| 3 | 1.98 V | 2457 | 0.2 V | 1.2 |
| 4 | 2.64 V | 3276 | 0.6 V | 1.6 |
| 5 | 3.30 V | 4095 | 1.0 V | 2.0 |

使用公式：

```text
VG = (20 / 33) * VDAC - 1
VGA_GAIN = (1 + VG) * Rf / RG
VOUT = (+VIN - -VIN) * (1 + VG) * Rf / RG
```

这里的 `VGA_GAIN` 是从差分输入 `(+VIN - -VIN)` 到 `VOUT` 的电压增益系数。

## 宏定义

`Core/User/vga_control.h` 保留以下可修改参数：

- DAC 参考电压，用于按实际 VDDA 校准输出电压。
- 六档 DAC 输出电压。
- `VG` 公式的比例系数与偏置。
- `Rf` 和 `RG`，初始值均为 `1.0f`；实际阻值确定后只需修改宏。
- 从 DAC 电压计算 `VG` 的函数式宏。
- 从 `VG` 计算 VGA 增益的函数式宏。
- 从差分输入、`VG`、`Rf` 和 `RG` 计算 `VOUT` 的函数式宏。

六个 DAC 码不定义为可调宏。模块内部固定使用 `DAC_ALIGN_12B_R`，并按 `round(VDAC / VREF * 4095)` 自动得到写入值。宏参数均加括号，避免表达式优先级问题。`RG` 必须为非零正数，并在编译期进行约束检查。

## 模块与接口

新建 `Core/User/vga_control.h` 和 `Core/User/vga_control.c`。模块文件头使用中文说明用途、PA4 引脚映射、DAC1 配置依赖、初始化方法和调用方法。

公开接口：

```c
typedef enum
{
    vga_control_status_ok = 0,
    vga_control_status_invalid_level,
    vga_control_status_null_pointer,
    vga_control_status_dac_error
} vga_control_status_t;

void vga_control_init(void);
vga_control_status_t vga_control_set_level(uint8_t level);
vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain);
```

`vga_control_init()` 先启动 DAC1 OUT1，再调用档位设置接口写入第 0 档。DAC 数据寄存器复位值和目标初值均为 0，因此启动过程中保持安全的 0 V 目标。初始化结果由模块状态变量保留，便于调试器观察。

`vga_control_set_level()` 使用 `switch (level)`：

- 六个 `case` 分别取得对应档位的 DAC 电压；
- 模块内部将目标电压自动换算为 12 位右对齐 DAC 码；
- 使用 `DAC_ALIGN_12B_R` 调用 HAL 设置 DAC 数据；DAC 通道只在初始化时启动一次；
- 只有 HAL 设置成功后，才更新当前档位和相关诊断值；
- `default` 返回 `vga_control_status_invalid_level`，不得调用 HAL，也不得改变当前状态；
- HAL 调用失败时返回 `vga_control_status_dac_error`，不得把失败档位记录为当前有效档位。

`vga_control_gain_from_level()` 使用 `switch (level)`：

- 六个 `case` 分别取得对应档位的 DAC 电压；
- 使用统一宏依次计算 `VG` 和 VGA 增益；
- 输出指针为空时返回 `vga_control_status_null_pointer`；
- 非法档位进入 `default` 并返回 `vga_control_status_invalid_level`；
- 失败时不得写入输出指针指向的值。

## 系统集成

- `Core/User/system.h` 统一包含 `vga_control.h`，并包含 CubeMX 生成的 `dac.h`，使用户模块能够访问 `hdac1`。
- `Core/User/system.c` 的 `system_init()` 增加 `vga_control_init()`。
- `main.c` 保持不变：用户初始化区仍只有 `system_init();`，主循环仍只有 `system_process();`。
- 本功能不使用中断，不新增中断标志。

## 错误处理与安全行为

- 上电默认第 0 档，理论 DAC 输出为 0 V，理论 VGA 增益为 0。
- 档位有效范围为 0 至 5。
- 非法档位保持当前 DAC 输出和当前有效档位不变。
- 空增益输出指针不执行计算。
- HAL 启动或设置失败时记录错误状态，不把目标档位标记为有效。
- 浮点计算只发生在查询增益或切换档位时，不进入中断，不影响实时采样回调。

## 测试与验证

新增离线契约测试，至少覆盖：

- 六档电压宏正确，且自动换算结果为 `0、819、1638、2457、3276、4095`；
- HAL 写入固定使用片上 DAC 的 `DAC_ALIGN_12B_R`，不存在六个可手动修改的 DAC 码宏；
- 六档 `VG` 为 `-1.0` 至 `1.0`，步进 `0.4`；
- 默认 `Rf/RG = 1` 时六档增益为 `0.0` 至 `2.0`，步进 `0.4`；
- 设置档位和计算增益均使用 `switch`，并包含非法档位 `default`；
- 非法档位不调用 HAL 设置接口；
- `system.h` 包含新模块，`system_init()` 调用初始化函数；
- `MX_DAC1_Init()` 先于 `system_init()`；
- `main.c` 用户初始化区和主循环结构保持符合工程规则。

随后运行现有全部 Python 契约测试，并在当前环境允许时执行 STM32CubeIDE Debug 构建。硬件交付时使用万用表或示波器依次测量 PA4 六档电压；再测量外接放大器的 `VG` 和 VGA 差分输入输出比，核对实际增益。理论值不替代模拟链路的实测校准。

## README 更新

README 增加：

- PA4/DAC1_OUT1 与外接放大器、VGA 的连接说明；
- DAC1 CubeMX 参数；
- 六档输出电压、`VG` 和默认理论增益表；
- 宏的修改位置；
- 模块初始化、档位设置和增益查询示例；
- DAC 参考电压、放大器误差、电阻误差和 VGA 器件特性导致的实际偏差说明。
