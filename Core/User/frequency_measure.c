/**
 * @file frequency_measure.c
 * @brief TIM5 外部脉冲计数频率测量模块实现。
 *
 * 模块用途：连续读取 TIM5 外部计数值，使用 DWT 补偿主循环延迟并计算平均频率。
 * GPIO 引脚映射：PA0/TIM5_CH1，输入 0～3.3 V CMOS 方波，最高频率 30 MHz。
 * 依赖的外设和 CubeIDE 配置：TIM5 外部时钟模式 1、TI1FP1 上升沿、无滤波、
 * PSC=0、ARR=0xFFFFFFFF；TIM3 以 1 Hz 产生更新中断；DWT 以 SystemCoreClock 计时。
 * 初始化方法：由 system_init() 调用 frequency_measure_init()。
 * 调用方法：由 system_process() 持续调用 frequency_measure_process()；TIM3 回调只置标志。
 */

#include "system.h"

/** TIM3 更新中断标志，由中断和主循环并发访问。 */
volatile uint8_t frequency_measure_flag = 0u;

/** 最近一次有效测量的外部信号平均频率，单位为 Hz。 */
volatile float frequency_measure_hz = 0.0f;

/** 上一次主循环快照的 TIM5 外部脉冲累计计数。 */
static uint32_t frequency_measure_previous_counter = 0u;

/** 上一次主循环快照的 Cortex-M7 DWT 周期计数。 */
static uint32_t frequency_measure_previous_cycles = 0u;

/** 模块启动成功标志，防止定时器启动失败后使用无效快照。 */
static uint8_t frequency_measure_ready = 0u;

/**
 * @brief 初始化外部频率测量模块并启动 TIM5 和 TIM3。
 * @param 无。
 * @return 无。
 * @note 初始化时清零一次 TIM5 CNT；不会清零 DWT，以免影响 FFT 耗时诊断。
 */
void frequency_measure_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    frequency_measure_flag = 0u;
    frequency_measure_hz = 0.0f;
    frequency_measure_ready = 0u;
    __HAL_TIM_SET_COUNTER(&htim5, 0u);

    if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
    {
        return;
    }
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
        return;
    }

    frequency_measure_previous_counter = __HAL_TIM_GET_COUNTER(&htim5);
    frequency_measure_previous_cycles = DWT->CYCCNT;
    frequency_measure_ready = 1u;
}

/**
 * @brief 领取 TIM3 更新标志并根据 TIM5、DWT 差值更新频率结果。
 * @param 无。
 * @return 无。
 * @note 会清除中断共享标志；32 位无符号减法允许计数器发生一次自然回绕。
 */
void frequency_measure_process(void)
{
    uint32_t current_counter;
    uint32_t current_cycles;
    uint32_t counter_delta;
    uint32_t cycle_delta;

    if ((frequency_measure_flag == 0u) || (frequency_measure_ready == 0u))
    {
        return;
    }

    frequency_measure_flag = 0u;
    current_counter = __HAL_TIM_GET_COUNTER(&htim5);
    current_cycles = DWT->CYCCNT;
    counter_delta = current_counter - frequency_measure_previous_counter;
    cycle_delta = current_cycles - frequency_measure_previous_cycles;

    if (cycle_delta != 0u)
    {
        frequency_measure_hz = (float)counter_delta
                             * ((float)SystemCoreClock / (float)cycle_delta);
    }

    frequency_measure_previous_counter = current_counter;
    frequency_measure_previous_cycles = current_cycles;
}

/**
 * @brief 请求主循环立即计算一次 TIM5 粗测频率。
 * @param 无。
 * @return 无。
 * @note 保留当前计数窗口，只复用 TIM3 中断使用的测量标志。
 */
void frequency_measure_request_now(void)
{
    frequency_measure_flag = 1u;
}

/**
 * @brief 处理 HAL 定时器周期完成回调。
 * @param htim 产生周期完成事件的定时器句柄。
 * @return 无。
 * @note TIM3 回调只置 frequency_measure_flag，不执行读取、计算或打印。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        frequency_measure_flag = 1u;
    }
}
