# 当前工程 README 与契约同步设计

## 目标

以当前已经完成实板调试的源码和 `h743_pre1.ioc` 为唯一事实来源，重写 README，
同步已经过时的静态契约测试，并将当前工作区完整提交到 GitHub 的
`h743_pre1` 分支。不得修改现有业务逻辑，也不得推送远程 `main`。

## 当前硬件基线

- MCU：STM32H743VIT6，STM32CubeIDE 1.19.0。
- PA4：DAC1_OUT1，提供 VGA 六档控制电压。
- PC4：ADC1_INP4，双 ADC 同步采集 CH1。
- PB1：ADC2_INP5，双 ADC 同步采集 CH2。
- PA0：TIM5_CH1，输入频率计数。
- PB12、PB13、PB15、PB14、PD8：AD9834 控制链路。
- PA9、PA10：USART1 淘晶驰串口屏。

## README 重写范围

README 应覆盖：

1. 项目用途、开发环境和当前已实现功能。
2. 完整硬件连接与 GPIO 映射。
3. ADC 双通道、TIM2、TIM3、TIM5、SPI2、DAC1 和 USART1 的 CubeMX 配置。
4. TIM5 粗测频、DDS 低侧本振、双通道 FFT、当前分段频率校准、测量结果换算、
   VGA 六档控制和 HMI 的数据流。
5. `Core/User` 模块职责、初始化顺序和主循环调用方式。
6. 编译、烧录、离线测试、实板验证和调试变量。
7. 当前限制以及需要重新进行实板校准的条件。

频率校准文档必须采用当前源码中的反向修正系数：

```text
f_raw <= 40000 Hz: f_cal = 1.0000193004 * f_raw + 0.24140466
f_raw >  40000 Hz: f_cal = 1.0000414617 * f_raw + 0.3261338
```

README 必须明确 `raw_peak_frequency_hz` 与
`secondary_raw_peak_frequency_hz` 保留原始 FFT 插值频率，最终发布值使用
`measurement_fft_calibrate_frequency()` 的结果。

## 契约测试同步

只修改与当前已调试实现冲突的旧契约：

- FFT 宏和数学边界测试改为当前反向修正公式。
- README 测试改为检查当前公式、字段和函数说明。
- HMI 测试继续要求电压与峰峰值来自 `measurement_result`，但允许
  `adc_dual_get_stats()` 仅用于 ADC 错误状态和溢出计数。
- DAC/VGA README 测试改为当前 PA4 DAC 输出、六档实测电压、VG 和增益接口。

不通过删除测试或降低核心计算断言来获得通过结果。

## 提交与推送

- 保留并提交当前工作区中的源码、CubeMX/IDE 配置和 Debug 构建产物。
- README 和测试同步完成后运行完整 Python 契约测试，并运行适用的 ARM GCC
  语法或工程构建验证。
- 将所有当前修改形成清晰的本地提交。
- 使用普通 `git push origin HEAD:h743_pre1`，不设置本地 `main` 的上游，
  不推送远程 `main`，远程非快进时停止并汇报，不使用强制推送。

## 完成标准

- README 与当前 `.ioc`、源码接口及数据流一致。
- 静态契约测试反映当前已调试行为并全部通过。
- 当前业务代码没有因文档同步被改写。
- 所有计划上传的文件均已提交。
- GitHub 的 `origin/h743_pre1` 指向本次提交。
