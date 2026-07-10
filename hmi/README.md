# TJC4827T143_011R_I_P20 串口屏工程

本目录保存串口屏源工程。第一版只显示测量结果，不做触摸控制和波形曲线。

## USART HMI 页面

1. 新建分辨率为 480 x 272 的工程，建立页面 `main`。
2. 在项目串口设置中选择 `9600`、8N1、无校验、无硬件流控。
3. 在 `main` 中建立下面五个 Text 控件。每个控件的 `vscope` 设为 global，动态内容只使用 ASCII。
4. 将工程保存为 `hmi/ads8688_measure.HMI`，并将对应的编译产物按需要保留在本目录。

| 控件名 | 初始 txt | `txt_maxl` | 建议位置 (x, y, w, h) |
| --- | --- | ---: | --- |
| `t_amp` | `--` | 20 | 190, 40, 260, 32 |
| `t_freq` | `--` | 20 | 190, 80, 260, 32 |
| `t_phase` | `--` | 20 | 190, 120, 260, 32 |
| `t_wave` | `UNKNOWN` | 16 | 190, 160, 260, 32 |
| `t_status` | `WAIT` | 12 | 190, 200, 260, 32 |

可在左侧放置静态 ASCII 标签 `VPP`、`FREQ`、`PHASE`、`WAVE`、`STATUS`。不要把中文作为 MCU 运行时下发的文本，避免字库和编码不一致。

## 固件协议

固件每约 250 ms 向 `main` 页面下发五条文本赋值指令。例如：

    main.t_amp.txt="3.300 Vpp"
    main.t_freq.txt="12.345 kHz"
    main.t_phase.txt="-90.0 deg"
    main.t_wave.txt="SINE"
    main.t_status.txt="LIVE"

每一条命令后都必须附加原始三个字节 `FF FF FF`，这三个字节不属于 ASCII 文本。固件上电绑定 UART 后会先发 `00 FF FF FF` 清理可能残留的串口输入。

没有算法结果时屏幕应显示 `WAIT`；收到无效数值时显示 `FAULT`；正常结果显示 `LIVE`。波类型只允许 `SINE`、`SQUARE`、`TRIANGLE`、`UNKNOWN`。

## CubeMX 与接线

本工程固定选择 `USART1`：

| USART1 信号 | STM32H750VBT6 引脚 | 第一版接线 |
| --- | --- | --- |
| TX | PA9 | 接串口屏 RX |
| RX | PA10 | 预留接串口屏 TX，第一版不处理触摸输入 |
| GND | 任一公共地 | 与屏幕 GND 共地 |

SPI2/ADS8688 已占用 PB12-PB15、PD8、PD9，且 DMA1 Stream0、DMA1 Stream1 已被 SPI2 DMA 使用。第一版串口屏明确不使用 USART1 DMA；保持 USART1 的 DMA Settings 为空，也不启用 USART1 global interrupt。固件使用主循环中的轮询发送。

屏幕供电需要单独确认 5 V 供电能力和公共地；屏幕 TX 是否接入 PA10 留待触摸版本确认。两项均为待硬件验证。

## 联调顺序

1. 用 USART HMI 调试器确认页面为 `main`、控件名完全一致、`txt_maxl` 足够，并以 9600 验证上述命令。
2. 在 CubeIDE 的 CubeMX 页面启用 USART1 Asynchronous，配置 9600、8 Bits、None、1 Stop Bit、No Flow Control，并生成代码。
3. 烧录后确认屏幕先显示 `WAIT`。算法模块调用 `measurement_result_publish()` 后，屏幕应刷新为 `LIVE`。
4. 上板检查 PA9 到屏幕 RX、公共地、SPI2 DMA 连续采集和约 4 Hz 的 HMI 刷新。轮询发送每次会短暂阻塞主循环，需确认不会影响 SPI2 DMA 连续采集。ADC 精度和测量结果均为待硬件验证。
