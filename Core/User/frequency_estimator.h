#ifndef FREQUENCY_ESTIMATOR_H
#define FREQUENCY_ESTIMATOR_H

#include <stdint.h>

/*
 * 模块用途：在锁相启动前，用连续长记录、Hann 窗 FFT 和三点峰值插值估计输入基波。
 * GPIO 映射：无直接 GPIO 引脚；输入样点由 signal_separation 模块从 PC0 ADC DMA 提供。
 * 外设依赖：无直接外设依赖；采样率和捕获长度来自 signal_separation_config.h。
 * 初始化方法：由 signal_separation_start() 间接调用 frequency_estimator_reset()。
 * 调用方法：搜索状态下逐个传入完整 ADC DMA 半区；返回 1 时读取 result。
 */

typedef struct
{
  uint8_t component_count;          /* 本次识别到的有效基波数量。 */
  uint32_t frequency_millihz[2];    /* 按频率升序排列，单位 0.001 Hz。 */
  uint32_t phase_q32[2];            /* 输入在最后一块原始 ADC 数据首点处的 Q32 基波相位。 */
  float amplitude_adc[2];           /* 长记录测得的基波峰值，单位为 ADC 码。 */
  float harmonic3_ratio[2];         /* 三次谐波幅值与基波幅值之比。 */
  float harmonic5_ratio[2];         /* 五次谐波幅值与基波幅值之比。 */
  float mean_adc;                   /* 长记录直流平均值，供低频过零锁相设置中心门限。 */
  uint8_t low_frequency_path;       /* 1 表示使用 32 倍抽取低频记录，0 表示原始采样记录。 */
} frequency_estimator_result_t;

/**
 * @brief 丢弃尚未完成的长记录并回到首次采样状态。
 * @param 无。
 * @return 无。
 */
void frequency_estimator_reset(void);

/**
 * @brief 累积连续 ADC 样点，并在记录完整后执行高精度频率估计。
 * @param samples 当前连续 ADC 样点块。
 * @param sample_count 当前样点块长度。
 * @param requested_components 请求识别的分量数，只允许 1 或 2。
 * @param result 非空结果指针；仅在返回 1 时内容有效。
 * @return 成功得到全部请求分量返回 1；仍在采集或本次记录无效返回 0。
 * @note 该函数仅在首次识别阶段执行 FFT；锁定后的实时 PLL 路径不会调用 FFT。
 */
uint8_t frequency_estimator_push(const uint16_t *samples,
                                 uint32_t sample_count,
                                 uint8_t requested_components,
                                 frequency_estimator_result_t *result);

#endif
