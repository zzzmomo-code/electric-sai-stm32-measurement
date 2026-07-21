/**
 * @file measurement_conversion.h
 * @brief 外差测量结果换算接口。
 *
 * 模块用途：把中频测量值换算为被测信号的频率和幅度，并为后续实测拟合保留入口。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：无需初始化。
 * 调用方法：串口屏刷新时调用两个换算函数；得到标定数据后只修改本模块公式。
 */

#ifndef MEASUREMENT_CONVERSION_H
#define MEASUREMENT_CONVERSION_H

#include <stdint.h>

/**
 * @brief 按低侧本振关系换算被测信号频率。
 * @param timer_frequency_hz TIM5 粗测频率，单位为 Hz，用作中频不可用时的后备值。
 * @param dds_frequency_hz AD9834 本振频率，单位为 Hz。
 * @param adc_frequency_hz ADC FFT 测得的中频，单位为 Hz。
 * @return 本振与中频均有效时返回二者之和，否则返回 TIM5 粗测频率。
 * @note 当前未加入频率拟合；后续只需在本函数内加入校准系数。
 */
float measurement_conversion_frequency_hz(float timer_frequency_hz,
                                           float dds_frequency_hz,
                                           float adc_frequency_hz);

/**
 * @brief 换算被测信号峰峰值幅度。
 * @param adc_amplitude_vpp ADC FFT 测得的中频峰峰值，单位为 Vpp。
 * @param vga_level 当前 VGA 档位，范围为 0 至 5。
 * @return 当前暂时原样返回 ADC 幅度。
 * @note 混频器、滤波器和 VGA 的实测拟合尚未完成，后续只需在本函数内替换公式。
 */
float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level);

#endif /* MEASUREMENT_CONVERSION_H */
