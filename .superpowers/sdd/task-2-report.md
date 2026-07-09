# Task 2：ADS8688 存储与电压换算实现报告

## 实现范围

- 在 `Core/User/ads8688_storage.h` 中声明 4096 条历史容量、8 个通道以及全部存储接口。
- 新增 `Core/User/ads8688_storage.c`，实现静态环形缓冲区、各通道最新值、覆盖计数和直二进制电压换算。
- 在 `tests/test_project_contract.py` 中增加去除注释后的常量与函数原型契约检查。
- 未修改 `.ioc`、CubeMX 生成的外设初始化代码或其他无关源码。

## RED 证据

命令：

```powershell
python -m unittest tests.test_project_contract -v
```

实现前结果：退出码 `1`，共运行 6 个测试；新增的存储 API 测试产生 6 个子测试失败，容量常量测试产生 1 个失败。失败原因均为 `ads8688_storage.h` 尚未声明规定的常量或函数原型；其余既有契约测试通过。

## GREEN 证据

命令：

```powershell
python -m unittest tests.test_project_contract -v
```

实现后结果：退出码 `0`，运行 6 个测试，全部通过。

## ARM 语法编译

命令：

```powershell
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe' -fsyntax-only -std=gnu11 -DSTM32H750xx -DUSE_HAL_DRIVER -ICore/User -ICore/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include Core/User/ads8688_storage.c
```

结果：退出码 `0`，无编译器输出和警告。

## 行为与边界自审

- 环形缓冲区静态数组容量严格为 `ADS8688_HISTORY_CAPACITY`（4096）。
- 初始化会清空历史索引、记录数量、覆盖计数和 8 个通道的全部最新值字段。
- 无效通道在任何状态变更前直接返回；有效采样同时更新最新值并追加历史记录。
- 缓冲区未满时增加记录数量；已满时覆盖头记录、推进头下标并增加覆盖计数。
- 历史读取拒绝空指针和零容量，按最旧到最新复制并移除已返回记录。
- 清空历史仅重置历史索引、数量和覆盖计数，不修改各通道最新值。
- 最新值读取对通道越界和空输出指针返回 `ADS8688_STATUS_INVALID_ARGUMENT`。
- 五种量程均按需求的 65536 分母进行直二进制换算；未知量程防御性返回 `0.0f`。
- 新增 `.c` 文件仅包含 `system.h`；新增标识符均采用小写下划线命名。
- 用户代码的模块、函数、参数、返回值、副作用和重要变量均已添加中文注释。
- 未触碰工作树中用户已有的 `.settings/language.settings.xml` 改动和 `tests/__pycache__/`。

## 提交

- 提交信息：`feat: add ADS8688 sample storage`
- 提交哈希：`0f926c7`

## 顾虑与未验证项

- 本任务指定的自动化测试只验证头文件契约；存储运行时行为通过代码审查确认，未在目标板上执行动态测试。
- 未执行完整 STM32CubeIDE 工程构建或硬件采集验证；本任务仅完成指定 ARM 语法编译。

## Important finding 修复：完整存储 API 签名契约

### RED 证据

新增回归测试后，仅运行该测试：

```powershell
python -m unittest tests.test_project_contract.ProjectContractTest.test_storage_api_with_wrong_signature_is_rejected -v
```

结果：退出码 `1`，运行 1 个测试并失败。旧检查把
`int ads8688_storage_read(void);` 误判为有效原型，断言信息为
`AssertionError: True is not false`，证明测试准确覆盖了审查指出的问题。

### GREEN 证据

实现去注释、移除预处理行、规范化空白与标点，并按分号分隔的完整声明比较后，运行：

```powershell
python -m unittest tests.test_project_contract -v
```

结果：退出码 `0`，运行 7 个测试，全部通过：

```text
Ran 7 tests in 0.005s

OK
```

回归测试分别拒绝错误返回类型、错误参数列表和错误参数顺序。存储接口契约测试逐个匹配以下六个完整声明：

```c
void ads8688_storage_init(void);
void ads8688_storage_push(uint8_t channel, uint16_t raw_code, ads8688_range_t range, uint32_t sample_index);
ads8688_status_t ads8688_storage_get_latest(uint8_t channel, ads8688_latest_t *latest);
uint32_t ads8688_storage_read(ads8688_sample_t *samples, uint32_t max_count);
void ads8688_storage_clear(void);
uint32_t ads8688_storage_get_overwrite_count(void);
```

### 修复自审

- 只修改了 `tests/test_project_contract.py` 与本报告，未修改生产代码。
- 完整声明比较锁定返回类型、参数类型、参数名称和参数顺序。
- 注释中的伪声明、换行和普通空白差异不会造成误判。
- 保留用户已有的 `.settings/language.settings.xml` 改动和未跟踪的 `tests/__pycache__/`，未纳入本次提交。
