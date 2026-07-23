/**
 * @file fft_analyzer.h
 * @brief 低频正弦基波的分块 FFT 捕获接口。
 *
 * 模块用途：对 ADC 数据先低通抽取，再执行 32768 点浮点 FFT，输出基波频率初值。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；输入来自 ADC DMA 数组。
 * 初始化方法：DPLL 初始化时调用 fft_analyzer_init()。
 * 调用方法：每个 ADC 半缓冲调用 fft_analyzer_process_block() 一次。
 */

#ifndef USER_FFT_ANALYZER_H
#define USER_FFT_ANALYZER_H

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    float frequency_hz;
    float peak_energy;
    uint32_t peak_bin;
} fft_analyzer_result_t;

void fft_analyzer_init(void);
void fft_analyzer_reset(void);
uint8_t fft_analyzer_process_block(const uint16_t *samples,
                                   size_t sample_count,
                                   float offset_adc_counts,
                                   fft_analyzer_result_t *result);

#endif /* USER_FFT_ANALYZER_H */
