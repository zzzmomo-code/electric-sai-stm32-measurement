/**
 * @file nco.h
 * @brief 64 位累加器使用的高分辨率数控振荡器查表接口。
 *
 * 模块用途：提供 Q32 周相位、频率步进转换和插值正弦值。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；只依赖标准数学库。
 * 初始化方法：调用 nco_init() 建立正弦查找表。
 * 调用方法：DPLL 使用 nco_sin_q32() 和 nco_increment_from_hz()。
 */

#ifndef USER_NCO_H
#define USER_NCO_H

#include <stdint.h>

void nco_init(void);
float nco_sin_q32(uint32_t phase_q32);
uint32_t nco_increment_from_hz(float frequency_hz, float sample_rate_hz);
uint32_t nco_phase_offset_from_deg(float phase_deg);

#endif /* USER_NCO_H */
