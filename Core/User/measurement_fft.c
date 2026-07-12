/**
 * @file measurement_fft.c
 * @brief ADS8688 双通道原始数据 FFT 采集与诊断实现。
 *
 * 模块用途：以 AIN0、AIN1 的原始直二进制采样构成双缓冲 8192 点窗口，去除直流后
 * 加 Hann 窗并执行 Q15 RFFT。当前仅输出 AIN0 的频谱峰值诊断，暂不发布未经校准的
 * 幅度、相位和波型结果。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：依赖 ADS8688 SPI2 DMA 主循环处理和 CMSIS-DSP Q15 RFFT。
 * 初始化方法：system_init() 调用 measurement_fft_init()。
 * 调用方法：ads8688_process() 内逐样本调用 measurement_fft_ingest_sample()，主循环
 * 随后调用 measurement_fft_process()。
 */

#include "system.h"
#include "arm_math.h"

#define MEASUREMENT_FFT_CHANNEL_COUNT 2u
#define MEASUREMENT_FFT_BUFFER_COUNT 2u
#define MEASUREMENT_FFT_FIRST_CHANNEL 0u
#define MEASUREMENT_FFT_SECOND_CHANNEL 1u
#define MEASUREMENT_FFT_HANN_Q15_MAX 32767.0f
#define MEASUREMENT_FFT_PI 3.14159265358979323846f

/* SPI2 为 16.125 MHz、每 ADS8688 帧 32 + 9 时钟、双通道轮询时的每通道采样率。 */
#define MEASUREMENT_FFT_SAMPLE_RATE_HZ 196646.34375f

/* 9600 波特率轮询发送的显示空档，覆盖一次 250 ms 刷新等待和最长 250 ms 发送。 */
#define MEASUREMENT_FFT_DISPLAY_GUARD_MS 500u

/* HMI blocking transmission may stall the main loop. Discard this many fresh
 * samples per channel before collecting the next contiguous FFT window. */
#define MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT 1024u

typedef enum
{
    MEASUREMENT_FFT_STATE_CAPTURE = 0,
    MEASUREMENT_FFT_STATE_READY,
    MEASUREMENT_FFT_STATE_DISPLAY,
    MEASUREMENT_FFT_STATE_SETTLING
} measurement_fft_state_t;

/** CMSIS-DSP 的 8192 点实数 Q15 FFT 实例。 */
static arm_rfft_instance_q15 measurement_fft_instance;

/** 双缓冲原始采样窗口：第一维为缓冲区，第二维为 AIN0/AIN1。 */
static q15_t measurement_fft_input[MEASUREMENT_FFT_BUFFER_COUNT]
                                    [MEASUREMENT_FFT_CHANNEL_COUNT]
                                    [MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/** AIN0 与 AIN1 的 RFFT 输出缓冲区。 */
static q15_t measurement_fft_output[MEASUREMENT_FFT_CHANNEL_COUNT]
                                     [MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/** 采样窗系数，Q15 格式，初始化阶段计算一次。 */
static q15_t measurement_fft_hann_window[MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/** 当前正在填充的双缓冲区编号。 */
static uint8_t measurement_fft_active_buffer;

/** 已收齐、等待 RFFT 处理的双缓冲区编号。 */
static uint8_t measurement_fft_ready_buffer;

/** AIN0 与 AIN1 在当前窗口内已写入的样本数量。 */
static uint16_t measurement_fft_sample_count[MEASUREMENT_FFT_CHANNEL_COUNT];

/** Number of post-display samples discarded before the next capture window. */
static uint16_t measurement_fft_settle_count[MEASUREMENT_FFT_CHANNEL_COUNT];

/** 当前采集、等待处理或显示空档状态。 */
static measurement_fft_state_t measurement_fft_state;

/** 显示空档开始的 HAL 时基，用于重新开始下一轮采集。 */
static uint32_t measurement_fft_display_start_ms;

/** FFT 运行状态和峰值诊断快照。 */
static measurement_fft_diagnostics_t measurement_fft_diagnostics;

/**
 * @brief 使能 Cortex-M7 DWT 周期计数器。
 * @param 无。
 * @return 无。
 * @note 仅用于记录 FFT 所耗 CPU 周期，不影响外设配置。
 */
static void measurement_fft_enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 将一个有符号样本限制到 Q15 表示范围。
 * @param value 待限制的整数值。
 * @return 限制后的 Q15 样本。
 * @note 无副作用。
 */
static q15_t measurement_fft_clamp_q15(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (q15_t)value;
}

/**
 * @brief 对单通道窗口去直流并施加 Hann 窗。
 * @param samples 待处理的 Q15 样本数组。
 * @return 无。
 * @note 原地修改输入窗口；该窗口随后将被 RFFT 使用。
 */
static void measurement_fft_prepare_window(q15_t *samples)
{
    int64_t sum = 0;
    int32_t mean;
    uint32_t index;

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        sum += samples[index];
    }
    mean = (int32_t)(sum / (int64_t)MEASUREMENT_FFT_LENGTH);

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        int32_t centered = (int32_t)samples[index] - mean;
        int32_t windowed =
            (centered * (int32_t)measurement_fft_hann_window[index]) >> 15;
        samples[index] = measurement_fft_clamp_q15(windowed);
    }
}

/**
 * @brief 查找 RFFT 输出中 AIN0 的最大非直流频谱峰值。
 * @param spectrum RFFT 输出数组。
 * @return 最大峰值对应的频点序号。
 * @note 跳过直流与奈奎斯特单独存储位置；使用平方幅值比较，不引入额外开方误差。
 */
static uint16_t measurement_fft_find_peak_bin(const q15_t *spectrum)
{
    int64_t largest_magnitude = -1;
    uint16_t peak_bin = 0u;
    uint16_t bin;

    for (bin = 1u; bin < (MEASUREMENT_FFT_LENGTH / 2u); bin++)
    {
        int32_t real = spectrum[(uint32_t)bin * 2u];
        int32_t imaginary = spectrum[(uint32_t)bin * 2u + 1u];
        int64_t magnitude = (int64_t)real * real + (int64_t)imaginary * imaginary;

        if (magnitude > largest_magnitude)
        {
            largest_magnitude = magnitude;
            peak_bin = bin;
        }
    }

    return peak_bin;
}

/**
 * @brief 初始化 8192 点 RFFT 和 Hann 窗。
 * @param 无。
 * @return 无。
 * @note 不启动 ADS8688；采样从 ADS8688 主循环逐样本输入开始。
 */
void measurement_fft_init(void)
{
    uint32_t index;

    measurement_fft_active_buffer = 0u;
    measurement_fft_ready_buffer = 0u;
    measurement_fft_sample_count[0] = 0u;
    measurement_fft_sample_count[1] = 0u;
    measurement_fft_settle_count[0] = 0u;
    measurement_fft_settle_count[1] = 0u;
    measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
    measurement_fft_display_start_ms = 0u;
    measurement_fft_diagnostics.init_status =
        (int32_t)arm_rfft_init_q15(&measurement_fft_instance,
                                   MEASUREMENT_FFT_LENGTH,
                                   0u,
                                   1u);
    measurement_fft_diagnostics.window_count = 0u;
    measurement_fft_diagnostics.dropped_window_count = 0u;
    measurement_fft_diagnostics.fft_count = 0u;
    measurement_fft_diagnostics.last_fft_cycles = 0u;
    measurement_fft_diagnostics.peak_bin = 0u;
    measurement_fft_diagnostics.peak_frequency_hz = 0.0f;
    measurement_fft_diagnostics.fft_ready = 0u;

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        float phase =
            (2.0f * MEASUREMENT_FFT_PI * (float)index) /
            (float)(MEASUREMENT_FFT_LENGTH - 1u);
        float coefficient = 0.5f * (1.0f - cosf(phase));
        measurement_fft_hann_window[index] =
            (q15_t)(coefficient * MEASUREMENT_FFT_HANN_Q15_MAX + 0.5f);
    }

    measurement_fft_enable_cycle_counter();
}

/**
 * @brief 接收一条 ADS8688 原始采样记录。
 * @param channel ADS8688 通道号，仅接收 AIN0 与 AIN1。
 * @param raw_code ADS8688 原始直二进制码。
 * @return 无。
 * @note 本函数仅复制样本，不执行 FFT；由 ADS8688 主循环处理函数调用，不能放入中断。
 */
void measurement_fft_ingest_sample(uint8_t channel, uint16_t raw_code)
{
    uint8_t local_channel;
    uint16_t write_index;

    if ((measurement_fft_diagnostics.init_status != (int32_t)ARM_MATH_SUCCESS)
        || (channel < MEASUREMENT_FFT_FIRST_CHANNEL)
        || (channel > MEASUREMENT_FFT_SECOND_CHANNEL))
    {
        return;
    }

    local_channel = channel - MEASUREMENT_FFT_FIRST_CHANNEL;

    if (measurement_fft_state == MEASUREMENT_FFT_STATE_SETTLING)
    {
        if (measurement_fft_settle_count[local_channel]
            < MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT)
        {
            measurement_fft_settle_count[local_channel]++;
        }

        if ((measurement_fft_settle_count[0]
             == MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT)
            && (measurement_fft_settle_count[1]
                == MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT))
        {
            measurement_fft_sample_count[0] = 0u;
            measurement_fft_sample_count[1] = 0u;
            measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
        }
        return;
    }

    if (measurement_fft_state != MEASUREMENT_FFT_STATE_CAPTURE)
    {
        return;
    }

    write_index = measurement_fft_sample_count[local_channel];
    if (write_index >= MEASUREMENT_FFT_LENGTH)
    {
        return;
    }

    measurement_fft_input[measurement_fft_active_buffer][local_channel][write_index] =
        (q15_t)((int32_t)raw_code - 32768);
    measurement_fft_sample_count[local_channel] = write_index + 1u;

    if ((measurement_fft_sample_count[0] == MEASUREMENT_FFT_LENGTH)
        && (measurement_fft_sample_count[1] == MEASUREMENT_FFT_LENGTH))
    {
        measurement_fft_ready_buffer = measurement_fft_active_buffer;
        measurement_fft_active_buffer ^= 1u;
        measurement_fft_sample_count[0] = 0u;
        measurement_fft_sample_count[1] = 0u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_READY;
        measurement_fft_diagnostics.window_count++;
    }
}

/**
 * @brief 处理已收齐的双通道采样窗口并执行 RFFT。
 * @param 无。
 * @return 无。
 * @note FFT 完成后进入显示空档，避免 9600 波特率 HMI 发送污染下一帧采样。
 */
void measurement_fft_process(void)
{
    uint32_t cycle_start;
    uint8_t channel;

    if (measurement_fft_state == MEASUREMENT_FFT_STATE_READY)
    {
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            measurement_fft_prepare_window(
                measurement_fft_input[measurement_fft_ready_buffer][channel]);
        }

        cycle_start = DWT->CYCCNT;
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            arm_rfft_q15(&measurement_fft_instance,
                         measurement_fft_input[measurement_fft_ready_buffer][channel],
                         measurement_fft_output[channel]);
        }
        measurement_fft_diagnostics.last_fft_cycles = DWT->CYCCNT - cycle_start;
        measurement_fft_diagnostics.peak_bin =
            measurement_fft_find_peak_bin(measurement_fft_output[0]);
        measurement_fft_diagnostics.peak_frequency_hz =
            ((float)measurement_fft_diagnostics.peak_bin *
             MEASUREMENT_FFT_SAMPLE_RATE_HZ) /
            (float)MEASUREMENT_FFT_LENGTH;
        measurement_fft_diagnostics.fft_count++;
        measurement_fft_diagnostics.fft_ready = 1u;
        measurement_fft_display_start_ms = HAL_GetTick();
        measurement_fft_state = MEASUREMENT_FFT_STATE_DISPLAY;
        return;
    }

    if ((measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY)
        && ((uint32_t)(HAL_GetTick() - measurement_fft_display_start_ms)
            >= MEASUREMENT_FFT_DISPLAY_GUARD_MS))
    {
        measurement_fft_settle_count[0] = 0u;
        measurement_fft_settle_count[1] = 0u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_SETTLING;
    }
}

/**
 * @brief 判断当前是否允许执行串口屏刷新。
 * @param 无。
 * @return 非零表示当前处于 FFT 完成后的显示空档；零表示正在采样或处理 FFT。
 * @note 显示空档结束后自动开始下一帧采集。
 */
uint8_t measurement_fft_hmi_refresh_allowed(void)
{
    return (measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY) ? 1u : 0u;
}

/**
 * @brief 获取最近一次 FFT 诊断快照。
 * @param diagnostics 用于接收诊断信息的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 仅供主循环或调试器读取，不修改 FFT 状态。
 */
uint8_t measurement_fft_get_diagnostics(measurement_fft_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return 0u;
    }

    *diagnostics = measurement_fft_diagnostics;
    return 1u;
}
