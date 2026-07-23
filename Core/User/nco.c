/**
 * @file nco.c
 * @brief 高分辨率数控振荡器查表实现。
 *
 * 模块用途：以 Q32 周相位驱动 2048 点正弦表，并进行线性插值。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无。
 * 初始化方法：系统启动时调用 nco_init() 一次。
 * 调用方法：DPLL 检相和 DAC 波形生成时调用。
 */

#include "system.h"

static float nco_sine_table[NCO_TABLE_SIZE];

/**
 * @brief 建立一个完整周期的正弦查找表。
 * @param 无。
 * @return 无。
 * @note 调用期间使用 sinf()，只允许在系统初始化阶段执行。
 */
void nco_init(void)
{
    uint32_t index;

    for (index = 0u; index < NCO_TABLE_SIZE; ++index)
    {
        const float phase = PHASE_TWO_PI_F * (float)index / (float)NCO_TABLE_SIZE;
        nco_sine_table[index] = sinf(phase);
    }
}

/**
 * @brief 根据 Q32 周相位返回线性插值后的正弦值。
 * @param phase_q32 一周对应 2^32 的无符号相位。
 * @return -1.0～1.0 的正弦值。
 * @note 只读取初始化后的查找表，不修改全局状态。
 */
float nco_sin_q32(uint32_t phase_q32)
{
    const uint32_t fractional_bits = 32u - NCO_TABLE_BITS;
    const uint32_t table_index = phase_q32 >> fractional_bits;
    const uint32_t next_index = (table_index + 1u) & (NCO_TABLE_SIZE - 1u);
    const uint32_t fraction_mask = (1u << fractional_bits) - 1u;
    const float fraction = (float)(phase_q32 & fraction_mask)
                         / (float)(1u << fractional_bits);
    const float first = nco_sine_table[table_index];

    return first + ((nco_sine_table[next_index] - first) * fraction);
}

/**
 * @brief 将频率转换成每采样点的 Q32 周相位增量。
 * @param frequency_hz 输出频率，单位 Hz。
 * @param sample_rate_hz 采样率，单位 Hz。
 * @return Q32 周相位增量。
 * @note 输入会限制到 0～Nyquist 频率。
 */
uint32_t nco_increment_from_hz(float frequency_hz, float sample_rate_hz)
{
    double scaled_increment;

    if (frequency_hz < 0.0f)
    {
        frequency_hz = 0.0f;
    }
    if (frequency_hz > (sample_rate_hz * 0.5f))
    {
        frequency_hz = sample_rate_hz * 0.5f;
    }

    scaled_increment = ((double)frequency_hz * 4294967296.0) / (double)sample_rate_hz;
    return (uint32_t)(scaled_increment + 0.5);
}

/**
 * @brief 将角度转换成 Q32 周相位偏移。
 * @param phase_deg 相位角，单位度，可为任意正负值。
 * @return 归一化到一周内的 Q32 相位。
 * @note 只执行数值转换，无硬件副作用。
 */
uint32_t nco_phase_offset_from_deg(float phase_deg)
{
    double wrapped_degrees = fmod((double)phase_deg, 360.0);

    if (wrapped_degrees < 0.0)
    {
        wrapped_degrees += 360.0;
    }

    return (uint32_t)((wrapped_degrees / 360.0) * 4294967296.0 + 0.5);
}
