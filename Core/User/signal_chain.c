/**
 * @file signal_chain.c
 * @brief ADC/DAC 同步 DMA 波形链路实现。
 *
 * 模块用途：启动共同 TIM2 触发的 ADC/DAC DMA，处理双半缓冲并选择直通或 DPLL 输出。
 * GPIO 引脚：PC0=ADC1_INP10，PA4=DAC1_OUT1。
 * 依赖外设：ADC1 16 位、DAC1 12 位、TIM2 TRGO=Update、DMA1 Stream0/1。
 * 初始化方法：CubeMX 外设初始化完成后调用 signal_chain_init()。
 * 调用方法：主循环高频调用 signal_chain_process()。
 */

#include "system.h"

volatile uint8_t adc_dma_half_ready_flag = 0u;
volatile uint8_t adc_dma_full_ready_flag = 0u;
volatile uint8_t adc_error_flag = 0u;
volatile uint8_t dac_error_flag = 0u;
volatile uint8_t dac_underrun_flag = 0u;

__attribute__((section(".dma_buffer"), aligned(SIGNAL_DMA_CACHE_LINE_BYTES)))
static uint16_t adc_dma_buffer[SIGNAL_DMA_BUFFER_SAMPLES];

__attribute__((section(".dma_buffer"), aligned(SIGNAL_DMA_CACHE_LINE_BYTES)))
static uint16_t dac_dma_buffer[SIGNAL_DMA_BUFFER_SAMPLES];

static dpll_t signal_dpll;
static signal_mode_t current_mode = SIGNAL_MODE_DPLL;
static float target_phase_deg = SIGNAL_DEFAULT_TARGET_PHASE_DEG;
static uint32_t processed_half_blocks = 0u;
static uint32_t adc_error_count = 0u;
static uint32_t dac_error_count = 0u;
static uint32_t processing_overrun_count = 0u;
static uint8_t signal_chain_running = 0u;

/**
 * @brief 原子读取并清除一个中断标志。
 * @param flag 待处理的单字节标志。
 * @return 清除前的标志值；短暂屏蔽中断。
 */
static uint8_t signal_take_flag(volatile uint8_t *flag)
{
    uint8_t value;
    const uint32_t interrupt_mask = __get_PRIMASK();

    __disable_irq();
    value = *flag;
    *flag = 0u;
    __set_PRIMASK(interrupt_mask);

    return value;
}

/**
 * @brief 使指定 ADC DMA 内存区域的 D-Cache 失效。
 * @param address 32 字节对齐的起始地址。
 * @param byte_count 32 字节整数倍长度。
 * @return 无；后续 CPU 读取将取得 DMA 最新数据。
 */
static void signal_cache_invalidate(void *address, uint32_t byte_count)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)address, (int32_t)byte_count);
    __DSB();
}

/**
 * @brief 将指定 DAC DMA 内存区域写回 D-Cache。
 * @param address 32 字节对齐的起始地址。
 * @param byte_count 32 字节整数倍长度。
 * @return 无；DMA 随后可读取 CPU 最新数据。
 */
static void signal_cache_clean(void *address, uint32_t byte_count)
{
    SCB_CleanDCache_by_Addr((uint32_t *)address, (int32_t)byte_count);
    __DSB();
}

/**
 * @brief 将一个 16 位 ADC 码按增益和偏置换算为 12 位 DAC 码。
 * @param adc_code ADC 原始码。
 * @return 0～4095 的 DAC 码，无外部副作用。
 */
static uint16_t signal_convert_adc_to_dac(uint16_t adc_code)
{
    uint32_t scaled_adc =
        (((uint32_t)adc_code * SIGNAL_DIRECT_GAIN_Q15) + 16384u) >> 15u;
    int32_t corrected_adc = (int32_t)scaled_adc + SIGNAL_DIRECT_OFFSET_ADC_COUNTS;
    int32_t dac_code;

    if (corrected_adc < 0)
    {
        corrected_adc = 0;
    }
    if (corrected_adc > 65535)
    {
        corrected_adc = 65535;
    }

    dac_code = (corrected_adc + 8) >> 4;
    if (dac_code > (int32_t)SIGNAL_DAC_MAX_CODE)
    {
        dac_code = (int32_t)SIGNAL_DAC_MAX_CODE;
    }

    return (uint16_t)dac_code;
}

/**
 * @brief 处理一个 ADC 半缓冲并更新对应 DAC 半缓冲。
 * @param sample_offset 半缓冲在数组中的起始采样下标。
 * @return 无；执行 Cache 维护并增加已处理块计数。
 */
static void signal_process_half(uint32_t sample_offset)
{
    uint16_t *adc_half = &adc_dma_buffer[sample_offset];
    uint16_t *dac_half = &dac_dma_buffer[sample_offset];
    uint32_t index;
    const uint32_t half_bytes = SIGNAL_DMA_HALF_SAMPLES * sizeof(uint16_t);

    signal_cache_invalidate(adc_half, half_bytes);

    if (current_mode == SIGNAL_MODE_DIRECT)
    {
        for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
        {
            dac_half[index] = signal_convert_adc_to_dac(adc_half[index]);
        }
    }
    else
    {
        dpll_process_block(&signal_dpll, adc_half, SIGNAL_DMA_HALF_SAMPLES);
        dpll_generate_dac(&signal_dpll,
                          dac_half,
                          SIGNAL_DMA_HALF_SAMPLES,
                          SIGNAL_DMA_HALF_SAMPLES,
                          target_phase_deg);
    }

    signal_cache_clean(dac_half, half_bytes);
    ++processed_half_blocks;
}

/**
 * @brief 校准 ADC 并启动 DAC DMA、ADC DMA 和公共 TIM2 采样时钟。
 * @param 无。
 * @return 无。
 * @note 任一 HAL 步骤失败时保持 stopped 状态，由状态串口报告。
 */
void signal_chain_init(void)
{
    uint32_t index;
    HAL_StatusTypeDef status;

    current_mode = SIGNAL_MODE_DPLL;
    target_phase_deg = SIGNAL_DEFAULT_TARGET_PHASE_DEG;
    processed_half_blocks = 0u;
    adc_error_count = 0u;
    dac_error_count = 0u;
    processing_overrun_count = 0u;
    signal_chain_running = 0u;

    adc_dma_half_ready_flag = 0u;
    adc_dma_full_ready_flag = 0u;
    adc_error_flag = 0u;
    dac_error_flag = 0u;
    dac_underrun_flag = 0u;

    nco_init();
    dpll_init(&signal_dpll, DPLL_DEFAULT_FREQUENCY_HZ);

    for (index = 0u; index < SIGNAL_DMA_BUFFER_SAMPLES; ++index)
    {
        adc_dma_buffer[index] = 0u;
        dac_dma_buffer[index] = SIGNAL_DAC_MID_CODE;
    }

    SCB_CleanInvalidateDCache_by_Addr(
        (uint32_t *)adc_dma_buffer, (int32_t)sizeof(adc_dma_buffer));
    signal_cache_clean(dac_dma_buffer, sizeof(dac_dma_buffer));

    status = HAL_ADCEx_Calibration_Start(
        &hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        ++adc_error_count;
        return;
    }

    status = HAL_DAC_Start_DMA(
        &hdac1,
        DAC_CHANNEL_1,
        (uint32_t *)dac_dma_buffer,
        SIGNAL_DMA_BUFFER_SAMPLES,
        DAC_ALIGN_12B_R);
    if (status != HAL_OK)
    {
        ++dac_error_count;
        return;
    }

    /* DAC 半满/全满中断不参与调度，只保留 DMA 错误和 DAC 欠载中断。 */
    __HAL_DMA_DISABLE_IT(&hdma_dac1_ch1, DMA_IT_HT | DMA_IT_TC);

    status = HAL_ADC_Start_DMA(
        &hadc1, (uint32_t *)adc_dma_buffer, SIGNAL_DMA_BUFFER_SAMPLES);
    if (status != HAL_OK)
    {
        ++adc_error_count;
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        return;
    }

    status = HAL_TIM_Base_Start(&htim2);
    if (status != HAL_OK)
    {
        ++adc_error_count;
        (void)HAL_ADC_Stop_DMA(&hadc1);
        (void)HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        return;
    }

    signal_chain_running = 1u;
}

/**
 * @brief 处理 DMA 标志、生成下一段 DAC 数据并统计错误。
 * @param 无。
 * @return 无。
 * @note 必须由主循环调用；所有数组复制、Cache 维护和 DPLL 运算均在此完成。
 */
void signal_chain_process(void)
{
    const uint8_t half_ready = signal_take_flag(&adc_dma_half_ready_flag);
    const uint8_t full_ready = signal_take_flag(&adc_dma_full_ready_flag);

    if ((half_ready != 0u) && (full_ready != 0u))
    {
        ++processing_overrun_count;
    }

    if (half_ready != 0u)
    {
        signal_process_half(0u);
    }
    if (full_ready != 0u)
    {
        signal_process_half(SIGNAL_DMA_HALF_SAMPLES);
    }

    if (signal_take_flag(&adc_error_flag) != 0u)
    {
        ++adc_error_count;
    }
    if (signal_take_flag(&dac_error_flag) != 0u)
    {
        ++dac_error_count;
    }
    if (signal_take_flag(&dac_underrun_flag) != 0u)
    {
        ++dac_error_count;
    }
}

/**
 * @brief 设置波形直通或 I/Q DPLL 模式。
 * @param mode 新运行模式。
 * @return 无。
 * @note 非法枚举值会被忽略。
 */
void signal_chain_set_mode(signal_mode_t mode)
{
    if (mode > SIGNAL_MODE_DPLL)
    {
        return;
    }

    if ((current_mode == SIGNAL_MODE_DIRECT) && (mode == SIGNAL_MODE_DPLL))
    {
        dpll_reset(&signal_dpll);
    }
    current_mode = mode;
}

/**
 * @brief 获取当前运行模式。
 * @param 无。
 * @return 当前 signal_mode_t。
 * @note 只读访问。
 */
signal_mode_t signal_chain_get_mode(void)
{
    return current_mode;
}

/**
 * @brief 设置 DPLL 输出相对输入的目标相位。
 * @param phase_deg 目标相位，单位度。
 * @return 无。
 * @note 自动归一化到 -180～180 度。
 */
void signal_chain_set_target_phase_deg(float phase_deg)
{
    while (phase_deg > 180.0f)
    {
        phase_deg -= 360.0f;
    }
    while (phase_deg < -180.0f)
    {
        phase_deg += 360.0f;
    }
    target_phase_deg = phase_deg;
}

/**
 * @brief 获取 DPLL 目标相位。
 * @param 无。
 * @return 目标相位，单位度。
 * @note 只读访问。
 */
float signal_chain_get_target_phase_deg(void)
{
    return target_phase_deg;
}

/**
 * @brief 清除 DPLL 状态并重新粗捕获。
 * @param 无。
 * @return 无。
 * @note 当前运行模式不变，仅重新初始化 DPLL。
 */
void signal_chain_reset_lock(void)
{
    dpll_reset(&signal_dpll);
}

/**
 * @brief 复制当前运行状态供串口输出。
 * @param status 接收状态的结构体指针。
 * @return 无。
 * @note 仅在主循环上下文调用，不涉及中断共享结构体。
 */
void signal_chain_get_status(signal_chain_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    status->mode = current_mode;
    status->lock_state = signal_dpll.lock_state;
    status->frequency_hz = signal_dpll.output_frequency_hz;
    status->phase_error_deg = dpll_get_phase_error_deg(&signal_dpll);
    status->amplitude_adc_counts = signal_dpll.amplitude_adc_counts;
    status->offset_adc_counts = signal_dpll.offset_adc_counts;
    status->target_phase_deg = target_phase_deg;
    status->processed_half_blocks = processed_half_blocks;
    status->adc_error_count = adc_error_count;
    status->dac_error_count = dac_error_count;
    status->processing_overrun_count = processing_overrun_count;
    status->running = signal_chain_running;
}

/**
 * @brief 判断是否有尚未处理的 ADC DMA 半缓冲。
 * @param 无。
 * @return 非零表示主循环应优先处理信号链路。
 * @note 只读取单字节 volatile 标志。
 */
uint8_t signal_chain_has_pending_work(void)
{
    return (uint8_t)(adc_dma_half_ready_flag | adc_dma_full_ready_flag);
}

/**
 * @brief ADC DMA 前半缓冲完成回调。
 * @param hadc ADC 句柄。
 * @return 无。
 * @note 中断上下文只设置一个对应标志。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_dma_half_ready_flag = 1u;
    }
}

/**
 * @brief ADC DMA 后半缓冲完成回调。
 * @param hadc ADC 句柄。
 * @return 无。
 * @note 中断上下文只设置一个对应标志。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_dma_full_ready_flag = 1u;
    }
}

/**
 * @brief ADC 过载或 DMA 错误回调。
 * @param hadc ADC 句柄。
 * @return 无。
 * @note 中断上下文只设置错误标志，恢复动作由主循环决定。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_error_flag = 1u;
    }
}

/**
 * @brief DAC DMA 错误回调。
 * @param hdac DAC 句柄。
 * @return 无。
 * @note 中断上下文只设置一个错误标志。
 */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC1)
    {
        dac_error_flag = 1u;
    }
}

/**
 * @brief DAC DMA 欠载回调。
 * @param hdac DAC 句柄。
 * @return 无。
 * @note 中断上下文只设置一个欠载标志。
 */
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
    if (hdac->Instance == DAC1)
    {
        dac_underrun_flag = 1u;
    }
}
