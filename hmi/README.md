# TJC4827T143_011R_I_P20 串口屏工程

源工程为 `hmi/ads8688_measure.HMI`。基础五控件页面已经烧录并完成实屏通信验证。

## 已验证基础页面

页面名为 `main`，分辨率 480 x 272，串口为 9600、8N1、无校验、无流控。

| 控件名 | 初始 txt | `txt_maxl` | 用途 |
| --- | --- | ---: | --- |
| `t_amp` | `--` | 20 | `Vdc` 或 `Vpp` |
| `t_freq` | `--` | 20 | Hz 或 kHz |
| `t_phase` | `--` | 20 | 相位差 |
| `t_wave` | `UNKNOWN` | 16 | 波形类型或 DC |
| `t_status` | `WAIT` | 12 | WAIT、LIVE、FAULT |

每个动态文本控件的 `vscope` 设为全局。MCU 下发文本只使用 ASCII。

基础命令示例：

```text
t_amp.txt="3.300 Vpp"
t_freq.txt="12.345 kHz"
t_phase.txt="-90.0 deg"
t_wave.txt="SINE"
t_status.txt="LIVE"
```

每条命令末尾由 MCU 追加原始三个字节 `FF FF FF`。当前固件约 4 Hz 刷新基础五控件。

## 明日增加详细指标

在 `main` 页面新增三个全局 Text 控件：

| 控件名 | 初始 txt | `txt_maxl` | 显示示例 |
| --- | --- | ---: | --- |
| `t_dc` | `--` | 20 | `0.001 Vdc` |
| `t_rms` | `--` | 20 | `0.707 Vrms` |
| `t_thd` | `--` | 20 | `1.25 %` |

固件已经提供 `hmi_tjc_build_detail_frame()`。控件未加入屏幕工程前，不要自动发送这三条命令。

## 明日增加 64 点频谱

1. 在 `main` 页面增加 Waveform 控件，建议命名为 `s_spectrum`。
2. 宽度至少 320 像素，高度按页面布局决定，通道 0 前景色应与背景有明显对比。
3. 在控件属性中记录实际数字 `id`，不要用控件创建顺序猜测。
4. 重新编译并烧录 `.HMI` 工程。
5. 将实际数字 ID 传给 `hmi_tjc_build_spectrum_frame()`，曲线通道使用 0。

固件构建的频谱命令为：

```text
cle <id>,0
add <id>,0,<0..255>
```

64 个频谱点覆盖 0 至 20 kHz，相对幅度 -80 至 0 dB 映射到 0 至 255。普通 `add` 不要求屏幕回传，保持当前只连接 MCU TX 到屏幕 RX 即可。

不要直接改成 `addt`。`addt` 使用透明传输握手，需要屏幕 TX 回 MCU RX，且会改变当前已经验证的单向轮询通信边界。

9600 波特率发送完整 64 点频谱约需接近 1 秒，因此当前固件只提供构帧接口，不会在每个 250 ms 基础刷新周期自动发送。实板确认控件 ID 后，应采用低频触发或分批加点，再检查 SPI2 DMA 的连续性。

## CubeMX 与接线

| USART1 信号 | STM32H750VBT6 | 接线 |
| --- | --- | --- |
| TX | PA9 | 接串口屏 RX |
| RX | PA10 | 当前不接，留给后续触摸或透明传输 |
| GND | 公共地 | 与屏幕 GND 共地 |

- USART1：Asynchronous，9600，8 Bits，None，1 Stop Bit，No Flow Control。
- SPI2 已占用 DMA1 Stream0 和 DMA1 Stream1；串口屏不使用 USART1 DMA，DMA Settings 保持为空。
- 不启用 USART1 global interrupt。
- 串口屏由能力足够的 5 V 电源供电，不能把 MCU IO 当作供电源。

## 验收顺序

1. 在 USART HMI 模拟器逐条发送基础五控件命令。
2. 加入 `t_dc/t_rms/t_thd` 后，用固定字符串确认文本长度和布局。
3. 加入 Waveform 后，先用 `cle` 和少量 `add` 确认实际 ID、通道号和纵轴方向。
4. 烧录 MCU 固件，确认基础测量仍正常刷新。
5. 最后接入低频频谱刷新，观察是否影响 ADS8688 DMA、FFT 周期和基础文本显示。

每次修改后都保存源工程，并重新生成、烧录对应的 TJC 编译文件。
