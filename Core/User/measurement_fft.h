/**
 * @file measurement_fft.h
 * @brief ADS8688 双通道原始数据 FFT 采集与诊断接口。
 *
 * 模块用途：采集 AIN0 与 AIN1 的原始 16 位采样，使用 8192 点 Q15 RFFT 得到
 * AIN0 主频峰值诊断数据，为后续幅度、相位差和波型识别提供输入。
 * GPIO 引脚映射：无直接 GPIO 引脚；输入数据来自 ADS8688 模块。
 * 依赖的外设和 CubeIDE 配置：依赖 ADS8688 SPI2 DMA 已工作，以及 CMSIS-DSP Q15 RFFT。
 * 初始化方法：系统启动时调用 measurement_fft_init()。
 * 调用方法：ADS8688 主循环处理每个样本后调用 measurement_fft_ingest_sample()；主循环
 * 调用 measurement_fft_process()，并仅在允许时调用 HMI 刷新。
 */

#ifndef MEASUREMENT_FFT_H
#define MEASUREMENT_FFT_H

#include <stdint.h>

/** 每通道一帧 RFFT 的采样点数。 */
#define MEASUREMENT_FFT_LENGTH 8192u

/** FFT 运行状态及最近一次峰值诊断数据。 */
typedef struct
{
    int32_t init_status;           /**< CMSIS-DSP RFFT 初始化结果，零表示成功。 */
    uint32_t window_count;         /**< 已收齐的双通道 8192 点窗口数量。 */
    uint32_t dropped_window_count; /**< 处理未完成时被丢弃的完整窗口数量。 */
    uint32_t fft_count;            /**< 已完成双通道 RFFT 的次数。 */
    uint32_t last_fft_cycles;      /**< 最近一次双通道 RFFT 的 Cortex-M7 时钟周期数。 */
    uint16_t peak_bin;             /**< AIN0 的最大非直流频谱峰值所在频点。 */
    float peak_frequency_hz;       /**< 由峰值频点得到的粗略频率，仅用于当前诊断。 */
    uint8_t fft_ready;             /**< 非零表示已有一帧 FFT 诊断结果。 */
} measurement_fft_diagnostics_t;

/**
 * @brief 初始化 8192 点 RFFT 和 Hann 窗。
 * @param 无。
 * @return 无。
 * @note 初始化失败时不会接收采样；可通过诊断接口读取 init_status。
 */
void measurement_fft_init(void);

/**
 * @brief 接收一条 ADS8688 原始采样记录。
 * @param channel ADS8688 通道号，仅接收 AIN0 与 AIN1。
 * @param raw_code ADS8688 原始直二进制码。
 * @return 无。
 * @note 本函数仅复制样本，不执行 FFT；由 ADS8688 主循环处理函数调用，不能放入中断。
 */
void measurement_fft_ingest_sample(uint8_t channel, uint16_t raw_code);

/**
 * @brief 处理已收齐的双通道采样窗口并执行 RFFT。
 * @param 无。
 * @return 无。
 * @note 为避免 9600 波特率 HMI 阻塞污染窗口，FFT 窗口采集中不允许屏幕刷新；显示结束后会先丢弃一小段过渡样本。
 */
void measurement_fft_process(void);

/**
 * @brief 判断当前是否允许执行串口屏刷新。
 * @param 无。
 * @return 非零表示当前处于 FFT 完成后的显示空档；零表示正在采样或处理 FFT。
 * @note 显示空档结束后自动开始下一帧采集。
 */
uint8_t measurement_fft_hmi_refresh_allowed(void);

/**
 * @brief 获取最近一次 FFT 诊断快照。
 * @param diagnostics 用于接收诊断信息的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 仅供主循环或调试器读取，不修改 FFT 状态。
 */
uint8_t measurement_fft_get_diagnostics(measurement_fft_diagnostics_t *diagnostics);

#endif /* MEASUREMENT_FFT_H */
