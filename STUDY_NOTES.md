# phase_locking_ported_codex 技术学习总结

> 本文档不是用户手册，是学习这个**成功项目**后总结的关键技术点，用于对比之前失败的 `phase_locking_test` 工程，指导后续修复。
> 完整用户手册见同目录 `README.md`。

## 1. 核心结论：为什么这个项目能成功

之前 `phase_locking_test` 一直输出平线/固定电压，根本原因有 5 个，这个成功项目全部解决：

| # | 失败原因（phase_locking_test） | 成功方案（本项目） |
|---|---|---|
| 1 | ADC 用 TIM2 触发，DAC 用 TIM6 触发，两个独立 TIM 会累积相位差 | **ADC 和 DAC 都用同一个 TIM2_TRGO 触发**，天然同步 |
| 2 | DMA buffer 放在 RAM_D1（0x24000000），但没处理 D-Cache 一致性 | DMA buffer 放在 **RAM_D2（0x30000000）.dma_buffer 段** + **D-Cache 开启** + invalidate/clean |
| 3 | DAC 用 TIM6 触发 + DMA，和 ADC DMA 冲突 | DAC 也用 **TIM2 触发 + DMA**，三个 DMA（ADC Stream0 / DAC CH1 Stream1 / DAC CH2 Stream2）独立运行 |
| 4 | TIM2 period=1199（200kHz），采样率太低 | TIM2 period=95（**2.5 MSPS**），高频信号也能采 |
| 5 | 轮询模式 + memcpy 回调，时序不可控 | **DMA 事件标志 + 主循环消费**，回调只递增计数器 |

## 2. 关键架构

### 2.1 数据流

```
TIM2 (2.5MHz TRGO) ──┬─触发─> ADC1 (16-bit, PC0)
                     │              ↓ DMA1 Stream0 Circular
                     │         adc_dma_buffer[1024] (D2 SRAM, 32B 对齐)
                     │              ↓ invalidate + memcpy
                     │         adc_analysis_buffer[512] (CPU 专用)
                     │              ↓ 相关检测 / FFT / PLL
                     │         active_component[2] (频率/相位/幅度/波形)
                     │              ↓ Q32 NCO + phase_offset
                     │         dac1_dma_buffer[2048] / dac2_dma_buffer[2048]
                     │              ↓ clean + DMA1 Stream1/2 Circular
                     └─触发─> DAC1_CH1 (PA4) / DAC1_CH2 (PA5)
                                    ↓
                              示波器 CH1 / CH2
```

### 2.2 三种频率识别模式

| 模式 | 算法 | 精度 | 内存 | 适用 |
|---|---|---|---|---|
| `GRID_5KHZ` | 5kHz 栅格相关检测，4 帧平均 | ±2.5kHz | 小 | 已验证方案，5kHz 整数倍信号 |
| `CONTINUOUS` | 5kHz 粗搜 + 4096 点细搜 + 250Hz 步进 + 抛物线插值 | ±125Hz | 中 | 任意频率，快速 |
| `PRECISE_FFT` ⭐默认 | 原始/32 倍抽取两条 32768 点记录 + Hann FFT + 插值 + 相位斜率 | 结果格式 0.001 Hz | 大 | 40 Hz～400 kHz 高精度首次判频 |

### 2.3 锁相环（PLL）

- **Q32 NCO**：32 位相位累加器，`phase_step = freq × 2^32 / fs`
- **PI 型数字 PLL**：测相位误差 -> Kp + Ki 积分（带泄漏）-> 修正 phase_step
- **捕获范围 ±2%**（HSI 精度差，扩大范围）
- **积分限幅**：避免长期频差饱和
- **单信号测相**：1kHz 以上用 500 点 2×2 中心化最小二乘；40Hz～1kHz 用迟滞/插值上升过零
- **双信号测相**：同一混合帧 4×4 联合最小二乘，先消除双音非正交串扰，再分别更新两个 PLL
- **低幅度门控**：信号低于有效阈值时冻结 PLL，避免积分噪声相位
- **幅度平滑**：`amp += (measured - amp) / 8`（一阶低通）

### 2.4 波形识别

- **单信号判波形记录**：与判频共用完整 32768 点长记录，不再使用最后 500 点
- **正弦波**：三次/五次谐波幅度低于阈值
- **三角波**：三次谐波 > 基波 × 6%（或五次 > 2.5%）
- **方波**（仅单信号模式）：三次谐波 > 基波 × 22%
- **谐波避让**：双信号模式如果另一分量频率 = 3 次谐波，改用 5 次谐波判别

## 3. 关键配置（.ioc）

### 3.1 时钟树（HSI）

```
HSI 64MHz -> PLL1 (M=4, N=60, P=2) -> SYSCLK 480MHz
                                      ↓ HPRE /2
                                    HCLK 240MHz
                                      ↓ APB1 /2
                                    APB1 120MHz -> APB1 Timer 240MHz

PLL2 (M=4, N=49, P=10) -> PLL2P 78.4MHz -> ADC Clock Mux
                         (H743 Rev.V 内部再 /2 = 39.2MHz ADC 实际时钟)
```

⚠️ **PLL2P 必须是 78.4MHz，不能恢复成 49MHz**。Rev.V 的 ADC 内部会再除 2，49MHz 实际只有 24.5MHz，2.5MSPS 触发会漏 20% 采样。

### 3.2 TIM2（2.5 MSPS 触发源）

```
Clock Source: Internal
Prescaler: 0
Period: 95           ← 240MHz / 96 = 2.5MHz
TRGO: Update Event   ← 同时触发 ADC + DAC CH1 + DAC CH2
```

### 3.3 ADC1

```
PC0 / ADC1_INP10
Resolution: 16 Bits
External Trig: TIM2_TRGO, Rising edge
Conversion Data Mgmt: DMA Circular
Overrun: Overwritten
Sampling Time: 1.5 cycles
DMA1 Stream0, Half Word, Priority HIGH, NVIC preemption 5
```

### 3.4 DAC1（双通道，都用 TIM2 触发）

```
PA4 / DAC1_OUT1: Trigger TIM2_TRGO, DMA1 Stream1, Priority Very High, NVIC 6
PA5 / DAC1_OUT2: Trigger TIM2_TRGO, DMA1 Stream2, Priority Very High, NVIC 7
Output Buffer: Enable
```

### 3.5 Cache + MPU

```c
SCB_EnableICache();    // I-Cache 开
SCB_EnableDCache();    // D-Cache 开（关键！）
MPU_Config();          // MPU 配置
```

### 3.6 链接脚本（.ld）关键段

```
.dma_buffer (NOLOAD) : {
    *(.dma_buffer)
    *(.dma_buffer*)
} > RAM_D2              ← DMA buffer 必须放 D2 SRAM (0x30000000)
```

DMA1 不能访问 DTCM（0x20000000），buffer 放错位置 DMA 会读到全 0 或旧数据。

## 4. D-Cache 一致性处理

```c
// ADC: DMA 写完，CPU 读前 -> invalidate
SCB_InvalidateDCache_by_Addr(adc_buffer, byte_count);
__DSB(); __ISB();
memcpy(analysis_buffer, adc_buffer, ...);

// DAC: CPU 写完，DMA 读前 -> clean
fill_dac_half(...);
SCB_CleanDCache_by_Addr(dac_buffer, byte_count);
__DSB(); __ISB();
```

**半区长度必须是 32 字节整数倍**（D-Cache 行大小）：
- ADC 半区 512 点 × 2 字节 = 1024 字节 ✓
- DAC 半区 1024 点 × 2 字节 = 2048 字节 ✓

## 5. DMA 事件驱动（不用 bool 标志）

```c
// 回调里只递增单调计数器
volatile uint32_t adc_dma_event_flag;
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) adc_dma_event_flag++;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) adc_dma_event_flag++;
}

// 主循环比较已处理序号，能识别丢帧和积压
uint32_t pending = adc_event_count - adc_event_handled;
if (pending > 1U) adc_frame_overrun += pending - 1U;  // 记录丢帧
```

**优点**：
- 不会丢事件（单调递增）
- 能检测丢帧（pending > 1）
- 绝对采样点时间不破坏（PLL 时间轴连续）

## 6. 双信号分离算法（CONTINUOUS 模式）

1. 8 个 DMA 半区（4096 点）连续采集
2. 5kHz 栅格粗搜主峰
3. 主峰附近 250Hz 步进细搜 + 抛物线插值 -> 频率1
4. **残差消除**：从长记录中减去频率1 的正弦波
5. 在残差上再搜第二主峰（屏蔽频率1 附近 4kHz）-> 频率2
6. 两个频率送入各自 Q32 NCO + PLL 实时跟踪

## 7. 代码结构

```
Core/User/
├─ system.h                       唯一统一头文件（main.c 只 include 这个）
├─ system.c                       system_init() + system_process()
├─ signal_separation_config.h     所有超参（模式/采样率/PLL/频率范围）
├─ signal_separation.h            API + 状态结构
├─ signal_separation.c            采样/识别/PLL/DAC/Cache 全部在这
├─ frequency_estimator.h/.c       32768 点 FFT 高精度判频
└─ uart_debug.h/.c                启动信息 + 一次性锁定结果打印
```

main.c 的 USER CODE 只有：
```c
system_init();
while (1) { system_process(); }
```

## 8. 超参快速参考

| 宏 | 默认值 | 作用 |
|---|---|---|
| `SIGSEP_OPERATION_MODE` | `SINGLE` | 单信号/双信号模式 |
| `SIGSEP_FREQUENCY_MODE` | `PRECISE_FFT` | 频率识别算法 |
| `SIGSEP_SAMPLE_RATE_HZ` | `2500000` | 2.5 MSPS |
| `SIGSEP_PHASE_OFFSET_DEFAULT_DEG` | `150` | DAC2 额外相位（0-180, 步进 5） |
| `SIGSEP_COMMON_SOURCE_LOCK` | `0` | 0=双 PLL 独立；1=主从同源 |
| `SIGSEP_PLL_MAX_CORR_DIV` | `50` | PLL 捕获范围 ±2% |
| `SIGSEP_PRECISE_FFT_LEN` | `32768` | FFT 点数 |
| `SIGSEP_PRECISE_FREQ_MIN_HZ` | `40` | 单信号搜索下限 |
| `SIGSEP_PRECISE_FREQ_MAX_HZ` | `400000` | 三类波形统一实用候选上限 |
| `SIGSEP_LOW_FREQ_DECIMATION` | `32` | 低频记录抽取倍数 |

## 9. 之前 phase_locking_test 修复路线图

基于学习到的成功方案，phase_locking_test 要成功需要：

1. **改 .ioc**：DAC 触发从 TIM6_TRGO 改成 **TIM2_TRGO**（和 ADC 同源）
2. **改 .ioc**：开启 **I-Cache + D-Cache**
3. **改 .ld**：加 `.dma_buffer >RAM_D2` 段
4. **改代码**：adc_buffer / dac_buffer 加 `__attribute__((section(".dma_buffer"), aligned(32)))`
5. **改代码**：ADC 读前 invalidate，DAC 写后 clean
6. **改代码**：DMA 半区改成 512 点（1024 字节，32 字节整数倍）
7. **改 TIM2**：Period 从 1199 改成 **95**（2.5 MSPS）
8. **加 PLL**：Q32 NCO + PI 控制器（不能只做直通回放）

## 10. 关键文件位置

```
phase_locking_ported_codex/
├─ phase_locking_ported_codex.ioc    CubeMX 配置
├─ STM32H743VITX_FLASH.ld           链接脚本（含 .dma_buffer 段）
├─ Core/Src/main.c                   含 SCB_EnableICache/DCache + MPU_Config
├─ Core/User/signal_separation.c     核心算法（2199 行）
├─ Core/User/signal_separation_config.h  超参
├─ Core/User/frequency_estimator.c   32768 点 FFT
└─ README.md                         完整用户手册（367 行）
```
