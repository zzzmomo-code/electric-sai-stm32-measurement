/**
 * @file measurement_conversion.c
 * @brief 外差测量结果换算实现。
 *
 * 模块用途：集中保存被测信号频率与幅度的换算公式，避免标定公式散落到显示代码。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：无需初始化。
 * 调用方法：由串口屏显示模块在生成实时页面数据时调用。
 */

#include "system.h"

/**
 * @brief 按低侧本振关系换算被测信号频率。
 * @param timer_frequency_hz TIM5 粗测频率，单位为 Hz。
 * @param dds_frequency_hz AD9834 本振频率，单位为 Hz。
 * @param adc_frequency_hz ADC FFT 测得的中频，单位为 Hz。
 * @return 本振与中频均有效时返回二者之和，否则返回 TIM5 粗测频率。
 * @note 后续频率拟合只修改此处，不影响采集和显示模块。
 */
float measurement_conversion_frequency_hz(float timer_frequency_hz,
                                           float dds_frequency_hz,
                                           float adc_frequency_hz)
{
    if ((dds_frequency_hz > 0.0f) && (adc_frequency_hz > 0.0f))
    {
        return dds_frequency_hz + adc_frequency_hz;
    }

    return timer_frequency_hz;
}

/**
 * @brief 换算被测信号峰峰值幅度。
 * @param adc_amplitude_vpp ADC FFT 测得的中频峰峰值，单位为 Vpp。
 * @param vga_level 当前 VGA 档位，范围为 0 至 5。
 * @return 当前暂时原样返回 ADC 幅度。
 * @note 待整机扫频标定后，在此加入频率、档位和幅度拟合参数。
 */
float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level)
{
    float gain;

    if((vga_control_gain_from_level(vga_level,&gain)==vga_control_status_ok)&&(gain>0.01f))
    		{
    	return adc_amplitude_vpp/gain;
    		}
    return adc_amplitude_vpp;
}
