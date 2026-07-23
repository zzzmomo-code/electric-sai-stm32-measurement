/**
 * @file adc_dual.c
 * @brief STM32H743VIT6 ADC1/ADC2 双路同步 DMA 采集实现。
 *
 * 模块用途：管理 32 字节对齐的双 ADC DMA 缓冲区、Cortex-M7 D-Cache 一致性、
 * DMA 事件领取和同步样本对提交。
 * GPIO 引脚映射：PC4/ADC1_INP4 为 CH1，PB1/ADC2_INP5 为 CH2。
 * 依赖的外设和 CubeIDE 配置：ADC1/ADC2 Dual Regular Simultaneous、TIM2 TRGO
 * 600 kHz、ADC1 DMA1 Stream0 Circular Word/Word；未生成 adc.h/tim.h 时编译为安全占位实现。
 * 初始化方法：measurement_input_init() 调用 adc_dual_init() 并按默认源启动。
 * 调用方法：measurement_input 管理 start/stop，并周期调用 adc_dual_process()。
 */

#include "system.h"

volatile uint8_t adc_dual_dma_half_flag;
volatile uint8_t adc_dual_dma_full_flag;
volatile uint8_t adc_dual_error_flag;

static adc_dual_stats_t adc_dual_stats;

#if defined(SYSTEM_ADC_DUAL_AVAILABLE)

static uint32_t adc_dual_dma_buffer[ADC_DUAL_DMA_WORD_COUNT]
    __attribute__((aligned(32)));

/**
 * @brief 判断 Cortex-M7 数据缓存是否已经启用。
 * @param 无。
 * @return D-Cache 已启用时返回 1，否则返回 0。
 * @note 未启用 D-Cache 时禁止执行按地址清理操作，避免触发 AXI 写总线故障。
 */
static uint8_t adc_dual_dcache_is_enabled(void)
{
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0u) ? 1u : 0u;
}

/**
 * @brief 领取并清除三个中断共享标志。
 * @param half_flag 用于接收前半区标志的指针。
 * @param full_flag 用于接收后半区标志的指针。
 * @param error_flag 用于接收错误标志的指针。
 * @return 无。
 * @note 临界区只包含三个字节的读取与清零。
 */
static void adc_dual_claim_flags(uint8_t *half_flag,
                                 uint8_t *full_flag,
                                 uint8_t *error_flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *half_flag = adc_dual_dma_half_flag;
    adc_dual_dma_half_flag = 0u;
    *full_flag = adc_dual_dma_full_flag;
    adc_dual_dma_full_flag = 0u;
    *error_flag = adc_dual_error_flag;
    adc_dual_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/**
 * @brief 根据 FFT 状态启停 TIM2 采样触发。
 * @param 无。
 * @return 无。
 * @note DMA 始终保持已装载，仅在连续窗口和 HMI 阻塞发送之间暂停触发。
 */
static void adc_dual_update_trigger_state(void)
{
    HAL_StatusTypeDef status;

    if (adc_dual_stats.state == ADC_DUAL_STATE_ERROR)
    {
        return;
    }

    if ((measurement_fft_sampling_required() != 0u)
        && (adc_dual_stats.timer_running == 0u))
    {
        status = HAL_TIM_Base_Start(&htim2);
        adc_dual_stats.last_hal_status = (int32_t)status;
        if (status == HAL_OK)
        {
            adc_dual_stats.timer_running = 1u;
            adc_dual_stats.state = ADC_DUAL_STATE_RUNNING;
        }
        else
        {
            adc_dual_stats.error_count++;
            adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        }
    }
    else if ((measurement_fft_sampling_required() == 0u)
             && (adc_dual_stats.timer_running != 0u))
    {
        status = HAL_TIM_Base_Stop(&htim2);
        adc_dual_stats.last_hal_status = (int32_t)status;
        if (status == HAL_OK)
        {
            adc_dual_stats.timer_running = 0u;
            adc_dual_stats.state = ADC_DUAL_STATE_STOPPED;
        }
        else
        {
            adc_dual_stats.error_count++;
            adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        }
    }
}

/**
 * @brief 处理一个由 DMA 写入的半缓冲区。
 * @param start_index 半缓冲区首元素索引。
 * @param word_count 本次处理的 32 位同步样本对数量。
 * @return 无。
 * @note 缓冲区和长度均为 32 字节缓存行整数倍，失效操作不会影响邻接变量。
 */
static void adc_dual_process_block(uint32_t start_index,
                                   uint32_t word_count)
{
    uint32_t index;
    uint32_t processed_count = 0u;
    uint32_t ch1_sum = 0u;
    uint32_t ch2_sum = 0u;
    uint16_t ch1_recent_min = 65535u;
    uint16_t ch1_recent_max = 0u;
    uint16_t ch2_recent_min = 65535u;
    uint16_t ch2_recent_max = 0u;

    if (adc_dual_dcache_is_enabled() != 0u)
    {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)&adc_dual_dma_buffer[start_index],
            (int32_t)(word_count * sizeof(adc_dual_dma_buffer[0])));
    }

    for (index = 0u; index < word_count; index++)
    {
        uint16_t ch1_code;
        uint16_t ch2_code;

        if (measurement_fft_sampling_required() == 0u)
        {
            adc_dual_stats.dropped_pair_count += word_count - index;
            break;
        }

        adc_dual_unpack_word(adc_dual_dma_buffer[start_index + index],
                             &ch1_code,
                             &ch2_code);
        if (ch1_code < adc_dual_stats.ch1_min_code)
        {
            adc_dual_stats.ch1_min_code = ch1_code;
        }
        if (ch1_code > adc_dual_stats.ch1_max_code)
        {
            adc_dual_stats.ch1_max_code = ch1_code;
        }
        if (ch2_code < adc_dual_stats.ch2_min_code)
        {
            adc_dual_stats.ch2_min_code = ch2_code;
        }
        if (ch2_code > adc_dual_stats.ch2_max_code)
        {
            adc_dual_stats.ch2_max_code = ch2_code;
        }

        if (ch1_code < ch1_recent_min)
        {
            ch1_recent_min = ch1_code;
        }
        if (ch1_code > ch1_recent_max)
        {
            ch1_recent_max = ch1_code;
        }
        if (ch2_code < ch2_recent_min)
        {
            ch2_recent_min = ch2_code;
        }
        if (ch2_code > ch2_recent_max)
        {
            ch2_recent_max = ch2_code;
        }
        ch1_sum += ch1_code;
        ch2_sum += ch2_code;
        processed_count++;

        if (measurement_fft_ingest_pair(ch1_code, ch2_code) != 0u)
        {
            adc_dual_stats.sample_pair_count++;
        }
        else
        {
            adc_dual_stats.dropped_pair_count++;
        }
    }

    if (processed_count != 0u)
    {
        adc_dual_stats.ch1_recent_min_code = ch1_recent_min;
        adc_dual_stats.ch1_recent_max_code = ch1_recent_max;
        adc_dual_stats.ch1_recent_mean_code =
            (uint16_t)(ch1_sum / processed_count);
        adc_dual_stats.ch2_recent_min_code = ch2_recent_min;
        adc_dual_stats.ch2_recent_max_code = ch2_recent_max;
        adc_dual_stats.ch2_recent_mean_code =
            (uint16_t)(ch2_sum / processed_count);
    }
}

#endif /* SYSTEM_ADC_DUAL_AVAILABLE */

void adc_dual_unpack_word(uint32_t packed_word,
                          uint16_t *ch1_code,
                          uint16_t *ch2_code)
{
    if (ch1_code != 0)
    {
        *ch1_code = (uint16_t)(packed_word & 0xffffu);
    }
    if (ch2_code != 0)
    {
        *ch2_code = (uint16_t)(packed_word >> 16);
    }
}

void adc_dual_init(void)
{
    adc_dual_dma_half_flag = 0u;
    adc_dual_dma_full_flag = 0u;
    adc_dual_error_flag = 0u;
    adc_dual_stats.dma_half_count = 0u;
    adc_dual_stats.dma_full_count = 0u;
    adc_dual_stats.error_count = 0u;
    adc_dual_stats.overflow_count = 0u;
    adc_dual_stats.backlog_count = 0u;
    adc_dual_stats.sample_pair_count = 0u;
    adc_dual_stats.dropped_pair_count = 0u;
    adc_dual_stats.ch1_min_code = 65535u;
    adc_dual_stats.ch1_max_code = 0u;
    adc_dual_stats.ch2_min_code = 65535u;
    adc_dual_stats.ch2_max_code = 0u;
    adc_dual_stats.ch1_recent_min_code = 65535u;
    adc_dual_stats.ch1_recent_max_code = 0u;
    adc_dual_stats.ch1_recent_mean_code = 0u;
    adc_dual_stats.ch2_recent_min_code = 65535u;
    adc_dual_stats.ch2_recent_max_code = 0u;
    adc_dual_stats.ch2_recent_mean_code = 0u;
    adc_dual_stats.last_hal_status = 0;
    adc_dual_stats.timer_running = 0u;

#if defined(SYSTEM_ADC_DUAL_AVAILABLE)
    adc_dual_stats.cubemx_ready = 1u;
    adc_dual_stats.state = ADC_DUAL_STATE_STOPPED;

    adc_dual_stats.last_hal_status = (int32_t)HAL_ADCEx_Calibration_Start(
        &hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    if (adc_dual_stats.last_hal_status != (int32_t)HAL_OK)
    {
        adc_dual_stats.error_count++;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return;
    }

    adc_dual_stats.last_hal_status = (int32_t)HAL_ADCEx_Calibration_Start(
        &hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    if (adc_dual_stats.last_hal_status != (int32_t)HAL_OK)
    {
        adc_dual_stats.error_count++;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return;
    }

#else
    adc_dual_stats.cubemx_ready = 0u;
    adc_dual_stats.state = ADC_DUAL_STATE_CUBEMX_NOT_READY;
#endif
}

/**
 * @brief 启动双 ADC 多模式 DMA 与 TIM2 触发。
 * @param 无。
 * @return 启动状态。
 * @note 重复启动不产生副作用。
 */
adc_dual_status_t adc_dual_start(void)
{
#if defined(SYSTEM_ADC_DUAL_AVAILABLE)
    HAL_StatusTypeDef status;

    if (adc_dual_stats.cubemx_ready == 0u)
    {
        return ADC_DUAL_STATUS_NOT_READY;
    }
    if (adc_dual_stats.state == ADC_DUAL_STATE_RUNNING)
    {
        return ADC_DUAL_STATUS_OK;
    }

    adc_dual_dma_half_flag = 0u;
    adc_dual_dma_full_flag = 0u;
    adc_dual_error_flag = 0u;
    if (adc_dual_dcache_is_enabled() != 0u)
    {
        SCB_CleanInvalidateDCache_by_Addr(
            adc_dual_dma_buffer,
            (int32_t)sizeof(adc_dual_dma_buffer));
    }
    status = HAL_ADCEx_MultiModeStart_DMA(
        &hadc1, adc_dual_dma_buffer, ADC_DUAL_DMA_WORD_COUNT);
    adc_dual_stats.last_hal_status = (int32_t)status;
    if (status != HAL_OK)
    {
        adc_dual_stats.error_count++;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return ADC_DUAL_STATUS_HAL_ERROR;
    }

    status = HAL_TIM_Base_Start(&htim2);
    adc_dual_stats.last_hal_status = (int32_t)status;
    if (status != HAL_OK)
    {
        (void)HAL_ADCEx_MultiModeStop_DMA(&hadc1);
        adc_dual_stats.error_count++;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return ADC_DUAL_STATUS_HAL_ERROR;
    }
    adc_dual_stats.timer_running = 1u;
    adc_dual_stats.state = ADC_DUAL_STATE_RUNNING;
    return ADC_DUAL_STATUS_OK;
#else
    return ADC_DUAL_STATUS_NOT_READY;
#endif
}

/**
 * @brief 停止 TIM2 触发与双 ADC 多模式 DMA。
 * @param 无。
 * @return 停止状态。
 * @note 清除尚未领取的 DMA 标志，保留统计数据。
 */
adc_dual_status_t adc_dual_stop(void)
{
#if defined(SYSTEM_ADC_DUAL_AVAILABLE)
    HAL_StatusTypeDef timer_status;
    HAL_StatusTypeDef adc_status;

    if (adc_dual_stats.cubemx_ready == 0u)
    {
        return ADC_DUAL_STATUS_NOT_READY;
    }
    if (adc_dual_stats.state == ADC_DUAL_STATE_STOPPED)
    {
        return ADC_DUAL_STATUS_OK;
    }
    timer_status = HAL_TIM_Base_Stop(&htim2);
    adc_status = HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    adc_dual_stats.last_hal_status =
        (timer_status != HAL_OK) ? (int32_t)timer_status : (int32_t)adc_status;
    adc_dual_stats.timer_running = 0u;
    adc_dual_dma_half_flag = 0u;
    adc_dual_dma_full_flag = 0u;
    adc_dual_error_flag = 0u;
    if ((timer_status != HAL_OK) || (adc_status != HAL_OK))
    {
        adc_dual_stats.error_count++;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return ADC_DUAL_STATUS_HAL_ERROR;
    }
    adc_dual_stats.state = ADC_DUAL_STATE_STOPPED;
    return ADC_DUAL_STATUS_OK;
#else
    return ADC_DUAL_STATUS_NOT_READY;
#endif
}

void adc_dual_process(void)
{
#if defined(SYSTEM_ADC_DUAL_AVAILABLE)
    uint8_t half_flag;
    uint8_t full_flag;
    uint8_t error_flag;

    adc_dual_claim_flags(&half_flag, &full_flag, &error_flag);
    if (error_flag != 0u)
    {
        adc_dual_stats.error_count++;
        if ((hadc1.ErrorCode & HAL_ADC_ERROR_OVR) != 0u)
        {
            adc_dual_stats.overflow_count++;
        }
        (void)HAL_TIM_Base_Stop(&htim2);
        adc_dual_stats.timer_running = 0u;
        adc_dual_stats.state = ADC_DUAL_STATE_ERROR;
        return;
    }

    if ((half_flag != 0u) && (full_flag != 0u))
    {
        adc_dual_stats.backlog_count++;
        adc_dual_stats.dropped_pair_count += ADC_DUAL_DMA_WORD_COUNT;
        if (measurement_fft_sampling_required() != 0u)
        {
            measurement_fft_resynchronize();
        }
        adc_dual_update_trigger_state();
        return;
    }
    if (half_flag != 0u)
    {
        adc_dual_stats.dma_half_count++;
        adc_dual_process_block(0u, ADC_DUAL_DMA_HALF_WORD_COUNT);
    }
    if (full_flag != 0u)
    {
        adc_dual_stats.dma_full_count++;
        adc_dual_process_block(ADC_DUAL_DMA_HALF_WORD_COUNT,
                               ADC_DUAL_DMA_HALF_WORD_COUNT);
    }

    adc_dual_update_trigger_state();
#endif
}

uint8_t adc_dual_get_stats(adc_dual_stats_t *stats)
{
    if (stats == 0)
    {
        return 0u;
    }

    *stats = adc_dual_stats;
    return 1u;
}

#if defined(SYSTEM_ADC_DUAL_AVAILABLE)

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        adc_dual_dma_half_flag = 1u;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        adc_dual_dma_full_flag = 1u;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        adc_dual_error_flag = 1u;
    }
}

#endif /* SYSTEM_ADC_DUAL_AVAILABLE */
