/**
 * @file measurement_input.c
 * @brief 内部双 ADC 与 ADS8688 动态采集源管理实现。
 *
 * 模块用途：串行化采集源切换，配置 FFT 采样率、通道数、轮询相位延迟和电压换算。
 * GPIO 引脚映射：由 adc_dual 和 ads8688 模块直接管理，本模块无直接 GPIO 操作。
 * 依赖的外设和 CubeIDE 配置：依赖 ADC1/ADC2、TIM2、SPI3 和对应 DMA 配置。
 * 初始化方法：system_init() 调用 measurement_input_init()。
 * 调用方法：主循环调用 measurement_input_process()，业务层调用公共选择接口。
 */

#include "system.h"

#define MEASUREMENT_INPUT_INTERNAL_SAMPLE_RATE_HZ 600000.0f

/** 当前活动采集源与累计切换诊断。 */
static measurement_input_diagnostics_t measurement_input_diagnostics;
/** ADS8688 配置模式，默认 AIN0/AIN1 双通道。 */
static uint8_t measurement_input_ads_single;
/** ADS8688 单通道模式选择的物理通道。 */
static uint8_t measurement_input_ads_channel;

/**
 * @brief 将 ADS8688 量程换算为 FFT 线性校准参数。
 * @param range ADS8688 硬件量程。
 * @param calibration 接收线性校准参数的指针。
 * @return 量程有效返回 1，否则返回 0。
 */
static uint8_t measurement_input_make_ads_calibration(
    ads8688_range_t range,
    measurement_fft_calibration_t *calibration)
{
    float minimum_v;
    float span_v;

    if (calibration == 0)
    {
        return 0u;
    }
    switch (range)
    {
        case ADS8688_RANGE_BIPOLAR_10V24:
            minimum_v = -10.24f;
            span_v = 20.48f;
            break;
        case ADS8688_RANGE_BIPOLAR_5V12:
            minimum_v = -5.12f;
            span_v = 10.24f;
            break;
        case ADS8688_RANGE_BIPOLAR_2V56:
            minimum_v = -2.56f;
            span_v = 5.12f;
            break;
        case ADS8688_RANGE_UNIPOLAR_10V24:
            minimum_v = 0.0f;
            span_v = 10.24f;
            break;
        case ADS8688_RANGE_UNIPOLAR_5V12:
            minimum_v = 0.0f;
            span_v = 5.12f;
            break;
        default:
            return 0u;
    }

    calibration->volts_per_code = span_v / 65536.0f;
    calibration->offset_v = minimum_v;
    calibration->valid = 1u;
    return 1u;
}

/**
 * @brief 配置内部双 ADC 对应的 FFT 参数与校准。
 * @param 无。
 * @return FFT 接受配置返回 1，否则返回 0。
 */
static uint8_t measurement_input_configure_internal_fft(void)
{
    measurement_fft_input_profile_t profile;
    measurement_fft_calibration_t calibration;

    profile.sample_rate_hz = MEASUREMENT_INPUT_INTERNAL_SAMPLE_RATE_HZ;
    profile.ch2_delay_seconds = 0.0f;
    profile.mode = MEASUREMENT_FFT_INPUT_DUAL_CHANNEL;
    calibration.volts_per_code = 0.00005035400390625f;
    calibration.offset_v = 0.0f;
    calibration.valid = 1u;
    (void)measurement_fft_set_calibration(0u, &calibration);
    (void)measurement_fft_set_calibration(1u, &calibration);
    return measurement_fft_configure_input(&profile);
}

/**
 * @brief 配置当前 ADS8688 模式对应的 FFT 参数与校准。
 * @param 无。
 * @return 全部配置有效返回 1，否则返回 0。
 */
static uint8_t measurement_input_configure_ads_fft(void)
{
    measurement_fft_input_profile_t profile;
    measurement_fft_calibration_t calibration;
    ads8688_range_t range;
    float sample_rate_hz = ads8688_get_effective_sample_rate_hz();

    if (sample_rate_hz <= 0.0f)
    {
        return 0u;
    }
    profile.sample_rate_hz = sample_rate_hz;
    profile.mode = (measurement_input_ads_single != 0u)
                       ? MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL
                       : MEASUREMENT_FFT_INPUT_DUAL_CHANNEL;
    profile.ch2_delay_seconds =
        (measurement_input_ads_single != 0u)
            ? 0.0f
            : 1.0f / (sample_rate_hz * 2.0f);

    if (ads8688_get_channel_range(
            (measurement_input_ads_single != 0u)
                ? measurement_input_ads_channel
                : 0u,
            &range) != ADS8688_STATUS_OK)
    {
        return 0u;
    }
    if (measurement_input_make_ads_calibration(range, &calibration) == 0u)
    {
        return 0u;
    }
    (void)measurement_fft_set_calibration(0u, &calibration);
    if (measurement_input_ads_single == 0u)
    {
        if ((ads8688_get_channel_range(1u, &range) != ADS8688_STATUS_OK)
            || (measurement_input_make_ads_calibration(
                    range, &calibration) == 0u))
        {
            return 0u;
        }
        (void)measurement_fft_set_calibration(1u, &calibration);
    }
    return measurement_fft_configure_input(&profile);
}

/**
 * @brief 确保 ADS8688 已初始化并写入当前管理层保存的通道模式。
 * @param single 非零选择单通道。
 * @param channel 单通道物理通道号。
 * @return 驱动配置成功返回 1，否则返回 0。
 * @note 初始化失败后再次选择 ADS 时会在此重新尝试初始化。
 */
static uint8_t measurement_input_apply_ads_mode(uint8_t single,
                                                uint8_t channel)
{
    ads8688_status_t status =
        (single != 0u)
            ? ads8688_set_single_channel(channel)
            : ads8688_set_dual_channel();

    if (status == ADS8688_STATUS_NOT_INITIALIZED)
    {
        status = ads8688_init();
        if (status == ADS8688_STATUS_OK)
        {
            status = (single != 0u)
                         ? ads8688_set_single_channel(channel)
                         : ads8688_set_dual_channel();
        }
    }
    measurement_input_diagnostics.last_driver_status = (int32_t)status;
    return (status == ADS8688_STATUS_OK) ? 1u : 0u;
}

/**
 * @brief 恢复活动 ADS8688 的旧模式、FFT 参数和 DMA。
 * @param single 旧单通道标志。
 * @param channel 旧单通道物理通道。
 * @return 完整恢复成功返回 1，否则返回 0。
 */
static uint8_t measurement_input_restore_ads(uint8_t single,
                                             uint8_t channel)
{
    measurement_input_ads_single = single;
    measurement_input_ads_channel = channel;
    if ((measurement_input_apply_ads_mode(single, channel) == 0u)
        || (measurement_input_configure_ads_fft() == 0u)
        || (ads8688_start() != ADS8688_STATUS_OK))
    {
        return 0u;
    }
    return 1u;
}

/**
 * @brief 停止指定采集源。
 * @param source 待停止采集源。
 * @return 底层停止成功返回 1，否则返回 0。
 */
static uint8_t measurement_input_stop_source(
    measurement_input_source_t source)
{
    if (source == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
    {
        measurement_input_diagnostics.last_driver_status =
            (int32_t)adc_dual_stop();
        return (measurement_input_diagnostics.last_driver_status
                == (int32_t)ADC_DUAL_STATUS_OK)
                   ? 1u
                   : 0u;
    }
    measurement_input_diagnostics.last_driver_status =
        (int32_t)ads8688_stop();
    return (measurement_input_diagnostics.last_driver_status
            == (int32_t)ADS8688_STATUS_OK)
               ? 1u
               : 0u;
}

/**
 * @brief 启动指定采集源。
 * @param source 待启动采集源。
 * @return 底层启动成功返回 1，否则返回 0。
 */
static uint8_t measurement_input_start_source(
    measurement_input_source_t source)
{
    if (source == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
    {
        measurement_input_diagnostics.last_driver_status =
            (int32_t)adc_dual_start();
        return (measurement_input_diagnostics.last_driver_status
                == (int32_t)ADC_DUAL_STATUS_OK)
                   ? 1u
                   : 0u;
    }
    measurement_input_diagnostics.last_driver_status =
        (int32_t)ads8688_start();
    return (measurement_input_diagnostics.last_driver_status
            == (int32_t)ADS8688_STATUS_OK)
               ? 1u
               : 0u;
}

/**
 * @brief 初始化两个采集驱动并默认启动内部双 ADC。
 * @param 无。
 * @return 初始化状态。
 * @note ADS8688 初始化失败不会阻止内部 ADC 启动，但会记录诊断。
 */
measurement_input_status_t measurement_input_init(void)
{
    ads8688_status_t ads_status;
    measurement_input_ads_single = 0u;
    measurement_input_ads_channel = 0u;
    measurement_input_diagnostics.active_source =
        MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC;
    measurement_input_diagnostics.switch_count = 0u;
    measurement_input_diagnostics.switch_failure_count = 0u;
    measurement_input_diagnostics.sample_rate_hz =
        MEASUREMENT_INPUT_INTERNAL_SAMPLE_RATE_HZ;
    measurement_input_diagnostics.ads8688_single_channel = 0u;
    measurement_input_diagnostics.ads8688_channel = 0u;
    measurement_input_diagnostics.last_driver_status = 0;

    adc_dual_init();
    ads_status = ads8688_init();
    if (ads_status != ADS8688_STATUS_OK)
    {
        measurement_input_diagnostics.switch_failure_count++;
        measurement_input_diagnostics.last_driver_status =
            (int32_t)ads_status;
    }
    if ((measurement_input_configure_internal_fft() == 0u)
        || (adc_dual_start() != ADC_DUAL_STATUS_OK))
    {
        measurement_input_diagnostics.switch_failure_count++;
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    return MEASUREMENT_INPUT_STATUS_OK;
}

/**
 * @brief 动态切换当前采集源。
 * @param source 目标内部 ADC 或 ADS8688。
 * @return 切换状态。
 * @note 失败时尝试恢复原采集源。
 */
measurement_input_status_t measurement_input_select(
    measurement_input_source_t source)
{
    measurement_input_source_t previous_source =
        measurement_input_diagnostics.active_source;
    uint8_t configured;

    if ((source != MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
        && (source != MEASUREMENT_INPUT_SOURCE_ADS8688))
    {
        return MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT;
    }
    if (source == previous_source)
    {
        return MEASUREMENT_INPUT_STATUS_OK;
    }
    if (measurement_input_stop_source(previous_source) == 0u)
    {
        measurement_input_diagnostics.switch_failure_count++;
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }

    if (source == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
    {
        configured = measurement_input_configure_internal_fft();
    }
    else
    {
        configured = measurement_input_apply_ads_mode(
            measurement_input_ads_single,
            measurement_input_ads_channel);
        if (configured != 0u)
        {
            configured = measurement_input_configure_ads_fft();
        }
    }
    if ((configured != 0u)
        && (measurement_input_start_source(source) != 0u))
    {
        measurement_input_diagnostics.active_source = source;
        measurement_input_diagnostics.sample_rate_hz =
            (source == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
                ? MEASUREMENT_INPUT_INTERNAL_SAMPLE_RATE_HZ
                : ads8688_get_effective_sample_rate_hz();
        measurement_input_diagnostics.switch_count++;
        return MEASUREMENT_INPUT_STATUS_OK;
    }

    measurement_input_diagnostics.switch_failure_count++;
    configured = (previous_source == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
                     ? measurement_input_configure_internal_fft()
                     : measurement_input_configure_ads_fft();
    if ((configured == 0u)
        || (measurement_input_start_source(previous_source) == 0u))
    {
        return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
    }
    return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
}

/**
 * @brief 设置 ADS8688 单通道并在活动时同步重启采样。
 * @param channel ADS8688 物理通道。
 * @return 配置状态。
 */
measurement_input_status_t measurement_input_set_ads8688_single_channel(
    uint8_t channel)
{
    uint8_t previous_single = measurement_input_ads_single;
    uint8_t previous_channel = measurement_input_ads_channel;
    uint8_t was_active =
        (measurement_input_diagnostics.active_source
         == MEASUREMENT_INPUT_SOURCE_ADS8688)
            ? 1u
            : 0u;

    if (channel >= ADS8688_CHANNEL_COUNT)
    {
        return MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT;
    }
    if ((was_active != 0u) && (ads8688_stop() != ADS8688_STATUS_OK))
    {
        measurement_input_diagnostics.switch_failure_count++;
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    if (measurement_input_apply_ads_mode(1u, channel) == 0u)
    {
        measurement_input_diagnostics.switch_failure_count++;
        if ((was_active != 0u)
            && (measurement_input_restore_ads(
                    previous_single, previous_channel) == 0u))
        {
            return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    measurement_input_ads_single = 1u;
    measurement_input_ads_channel = channel;
    if ((was_active != 0u)
        && ((measurement_input_configure_ads_fft() == 0u)
            || (ads8688_start() != ADS8688_STATUS_OK)))
    {
        measurement_input_diagnostics.switch_failure_count++;
        if (measurement_input_restore_ads(
                previous_single, previous_channel) == 0u)
        {
            return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    measurement_input_diagnostics.ads8688_single_channel = 1u;
    measurement_input_diagnostics.ads8688_channel = channel;
    measurement_input_diagnostics.sample_rate_hz =
        ads8688_get_effective_sample_rate_hz();
    return MEASUREMENT_INPUT_STATUS_OK;
}

/**
 * @brief 设置 ADS8688 AIN0/AIN1 双通道并同步 FFT。
 * @param 无。
 * @return 配置状态。
 */
measurement_input_status_t measurement_input_set_ads8688_dual_channel(void)
{
    uint8_t previous_single = measurement_input_ads_single;
    uint8_t previous_channel = measurement_input_ads_channel;
    uint8_t was_active =
        (measurement_input_diagnostics.active_source
         == MEASUREMENT_INPUT_SOURCE_ADS8688)
            ? 1u
            : 0u;

    if ((was_active != 0u) && (ads8688_stop() != ADS8688_STATUS_OK))
    {
        measurement_input_diagnostics.switch_failure_count++;
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    if (measurement_input_apply_ads_mode(0u, 0u) == 0u)
    {
        measurement_input_diagnostics.switch_failure_count++;
        if ((was_active != 0u)
            && (measurement_input_restore_ads(
                    previous_single, previous_channel) == 0u))
        {
            return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    measurement_input_ads_single = 0u;
    measurement_input_ads_channel = 0u;
    if ((was_active != 0u)
        && ((measurement_input_configure_ads_fft() == 0u)
            || (ads8688_start() != ADS8688_STATUS_OK)))
    {
        measurement_input_diagnostics.switch_failure_count++;
        if (measurement_input_restore_ads(
                previous_single, previous_channel) == 0u)
        {
            return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    measurement_input_diagnostics.ads8688_single_channel = 0u;
    measurement_input_diagnostics.ads8688_channel = 0u;
    measurement_input_diagnostics.sample_rate_hz =
        ads8688_get_effective_sample_rate_hz();
    return MEASUREMENT_INPUT_STATUS_OK;
}

/**
 * @brief 设置 ADS8688 通道量程并同步 FFT 电压校准。
 * @param channel ADS8688 物理通道。
 * @param range 目标硬件量程。
 * @return 配置状态。
 */
measurement_input_status_t measurement_input_set_ads8688_channel_range(
    uint8_t channel,
    ads8688_range_t range)
{
    ads8688_status_t status;
    ads8688_range_t previous_range;
    measurement_fft_calibration_t validation_calibration;
    uint8_t was_active =
        (measurement_input_diagnostics.active_source
         == MEASUREMENT_INPUT_SOURCE_ADS8688)
            ? 1u
            : 0u;

    if ((channel >= ADS8688_CHANNEL_COUNT)
        || (measurement_input_make_ads_calibration(
                range, &validation_calibration) == 0u))
    {
        return MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT;
    }
    status = ads8688_get_channel_range(channel, &previous_range);
    if (status == ADS8688_STATUS_NOT_INITIALIZED)
    {
        if (measurement_input_apply_ads_mode(
                measurement_input_ads_single,
                measurement_input_ads_channel) == 0u)
        {
            measurement_input_diagnostics.switch_failure_count++;
            return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
        }
        status = ads8688_get_channel_range(channel, &previous_range);
    }
    measurement_input_diagnostics.last_driver_status = (int32_t)status;
    if (status != ADS8688_STATUS_OK)
    {
        return (status == ADS8688_STATUS_INVALID_ARGUMENT)
                   ? MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT
                   : MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    if ((was_active != 0u) && (ads8688_stop() != ADS8688_STATUS_OK))
    {
        measurement_input_diagnostics.switch_failure_count++;
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    if (ads8688_set_channel_range(channel, range) != ADS8688_STATUS_OK)
    {
        measurement_input_diagnostics.switch_failure_count++;
        if (was_active != 0u)
        {
            if ((ads8688_set_channel_range(channel, previous_range)
                 != ADS8688_STATUS_OK)
                || (ads8688_start() != ADS8688_STATUS_OK))
            {
                return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
            }
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    if ((was_active != 0u)
        && ((measurement_input_configure_ads_fft() == 0u)
            || (ads8688_start() != ADS8688_STATUS_OK)))
    {
        measurement_input_diagnostics.switch_failure_count++;
        if ((ads8688_set_channel_range(channel, previous_range)
             != ADS8688_STATUS_OK)
            || (measurement_input_configure_ads_fft() == 0u)
            || (ads8688_start() != ADS8688_STATUS_OK))
        {
            return MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR;
        }
        return MEASUREMENT_INPUT_STATUS_DRIVER_ERROR;
    }
    return MEASUREMENT_INPUT_STATUS_OK;
}

/**
 * @brief 处理当前采集源事件并推进 FFT 状态机。
 * @param 无。
 * @return 无。
 */
void measurement_input_process(void)
{
    if (measurement_input_diagnostics.active_source
        == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
    {
        adc_dual_process();
    }
    else
    {
        ads8688_process();
    }
    measurement_fft_process();
    if (measurement_input_diagnostics.active_source
        == MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC)
    {
        adc_dual_process();
    }
    else
    {
        ads8688_process();
    }
}

/**
 * @brief 复制动态采集源诊断快照。
 * @param diagnostics 接收快照的指针。
 * @return 指针有效返回 1，否则返回 0。
 */
uint8_t measurement_input_get_diagnostics(
    measurement_input_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return 0u;
    }
    *diagnostics = measurement_input_diagnostics;
    return 1u;
}
