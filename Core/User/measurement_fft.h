/**
 * @file measurement_fft.h
 * @brief 双通道同步信号分析与 8192 点 Q15 FFT 公共接口。
 *
 * 模块用途：接收 CH1 与 CH2 的同步样本对，计算直流、峰峰值、有效值、主频率、
 * 失真度、频谱和波形类型，并计算 CH2 相对 CH1 的相位差。
 * GPIO 引脚映射：无直接 GPIO 引脚；输入数据由 adc_dual 模块提交。
 * 依赖的外设和 CubeIDE 配置：依赖片上 ADC1/ADC2 同步采样；
 * Q15 旋转因子在 RAM 中生成，以适配 STM32H750VBT6 的 128 KiB Flash；
 * 不直接访问 ADC 或 DMA 缓冲区。
 * 初始化方法：系统启动时调用 measurement_fft_init()。
 * 调用方法：采集模块调用 measurement_fft_ingest_pair()；主循环调用
 * measurement_fft_process()，并仅在 measurement_fft_hmi_refresh_allowed() 允许时刷新 HMI。
 */

#ifndef MEASUREMENT_FFT_H
#define MEASUREMENT_FFT_H

#include <stdint.h>

#include "measurement_result.h"

/** 每通道一帧 FFT 的采样点数。 */
#define MEASUREMENT_FFT_LENGTH 8192u

/** Q15 复数 FFT 的单通道交错实部/虚部输出元素数量。 */
#define MEASUREMENT_FFT_OUTPUT_LENGTH (2u * MEASUREMENT_FFT_LENGTH)

/** 面向串口屏显示压缩后的频谱点数。 */
#define MEASUREMENT_FFT_SPECTRUM_POINT_COUNT 64u

/** 压缩频谱覆盖训练题规定的最高频率，单位为 Hz。 */
#define MEASUREMENT_FFT_SPECTRUM_MAX_HZ 20000.0f

/** 频谱无有效能量时使用的相对幅度下限，单位为 0.1 dB。 */
#define MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10 (-800)

/** 最近一次测量结果的质量状态，供调试器和后续诊断界面读取。 */
typedef enum
{
    MEASUREMENT_FFT_QUALITY_NOT_READY = 0,
    MEASUREMENT_FFT_QUALITY_OK,
    MEASUREMENT_FFT_QUALITY_INIT_ERROR,
    MEASUREMENT_FFT_QUALITY_RANGE_ERROR,
    MEASUREMENT_FFT_QUALITY_SIGNAL_TOO_SMALL,
    MEASUREMENT_FFT_QUALITY_CLIPPED,
    MEASUREMENT_FFT_QUALITY_CHANNEL_MISMATCH,
    MEASUREMENT_FFT_QUALITY_DC_INPUT
} measurement_fft_quality_t;

/** 供低速 HMI 使用的 0 至 20 kHz 压缩频谱快照。 */
typedef struct
{
    int16_t relative_db_x10[MEASUREMENT_FFT_SPECTRUM_POINT_COUNT];
    float point_width_hz;      /**< 每个显示点覆盖的频率宽度，单位为 Hz。 */
    float nyquist_hz;          /**< 当前抽取配置的奈奎斯特频率，单位为 Hz。 */
    uint32_t sequence;         /**< 频谱更新序号。 */
    uint8_t valid;             /**< 非零表示数组已由一帧有效 FFT 更新。 */
} measurement_fft_spectrum_t;

/** 单通道原始码到输入电压的线性校准系数。 */
typedef struct
{
    float volts_per_code; /**< 每个 ADC 原始码对应的输入电压，单位为 V/code。 */
    float offset_v;       /**< 原始码为零时对应的输入电压，单位为 V。 */
    uint8_t valid;        /**< 非零表示前级比例与偏置已经确认，可发布电压字段。 */
} measurement_fft_calibration_t;

/** FFT 运行状态及最近一次完整测量诊断数据。 */
typedef struct
{
    int32_t init_status;                /**< Q15 FFT 初始化结果，零表示成功。 */
    uint32_t window_count;              /**< 已收齐的双通道 8192 点窗口数量。 */
    uint32_t discarded_sample_count;    /**< 处理和显示期间主动忽略的已取出样本数量。 */
    uint32_t fft_count;                 /**< 已完成双通道 FFT 的次数。 */
    uint32_t publish_count;             /**< 已发布到 measurement_result 的结果数量。 */
    uint32_t last_fft_cycles;           /**< 最近一次双通道 FFT 的 Cortex-M7 周期数。 */
    float raw_sample_rate_hz;           /**< ADC1/ADC2 每通道同步原始采样率，单位为 Hz。 */
    float effective_sample_rate_hz;     /**< 当前抽取后的 FFT 有效采样率，单位为 Hz。 */
    float bin_width_hz;                 /**< 当前 FFT 本征频点间隔，单位为 Hz。 */
    uint8_t decimation_factor;          /**< 固定为 1，表示全部 80 kSPS 同步样本均进入 FFT。 */
    uint16_t peak_bin;                  /**< AIN0 主峰整数频点。 */
    uint16_t secondary_peak_bin;        /**< AIN1 主峰整数频点。 */
    float peak_offset_bins;             /**< AIN0 三点抛物线插值得到的亚频点偏移。 */
    float peak_frequency_hz;            /**< AIN0 插值主频率，单位为 Hz。 */
    float amplitude_vpp;                /**< AIN0 时域峰峰值，单位为 V。 */
    float secondary_amplitude_vpp;      /**< AIN1 时域峰峰值，单位为 V。 */
    float dc_voltage;                   /**< AIN0 窗口平均直流电压，单位为 V。 */
    float secondary_dc_voltage;         /**< AIN1 窗口平均直流电压，单位为 V。 */
    float rms_voltage;                  /**< AIN0 去直流交流有效值，单位为 V。 */
    float secondary_rms_voltage;        /**< AIN1 去直流交流有效值，单位为 V。 */
    float thd_percent;                  /**< AIN0 可用谐波范围内 THD，单位为百分比。 */
    uint8_t thd_harmonic_count;         /**< 本帧实际纳入 THD 的谐波数量。 */
    float raw_phase_deg;                /**< 未补偿通道轮询时差的 AIN1-AIN0 相位。 */
    float phase_deg;                    /**< 补偿标称通道时差后的 AIN1-AIN0 相位。 */
    float harmonic_ratio_3;             /**< AIN0 三次谐波与基波幅值比。 */
    float harmonic_ratio_5;             /**< AIN0 五次谐波与基波幅值比。 */
    measurement_wave_type_t wave_type;  /**< AIN0 初步波形分类结果。 */
    measurement_fft_quality_t quality;  /**< 最近一次结果质量状态。 */
    uint8_t clipping_mask;              /**< 位 0/1 分别表示 AIN0/AIN1 接近满量程削顶。 */
    uint8_t result_valid;               /**< 非零表示最近一次结果已经作为 LIVE 数据发布。 */
    uint8_t fft_ready;                  /**< 非零表示至少完成过一次双通道 FFT。 */
    uint8_t voltage_calibrated_mask;    /**< 位 0/1 表示 CH1/CH2 已具备有效电压校准。 */
} measurement_fft_diagnostics_t;

/**
 * @brief 初始化 8192 点 Q15 FFT 旋转因子、Hann 窗和测量状态。
 * @param 无。
 * @return 无。
 * @note 初始化失败时停止接收样本，并在诊断结构中记录 INIT_ERROR。
 */
void measurement_fft_init(void);

/**
 * @brief 接收同一触发时刻的双通道原始采样对。
 * @param ch1_raw_code ADC1/CH1 的 16 位原始码。
 * @param ch2_raw_code ADC2/CH2 的 16 位原始码。
 * @return 无。
 * @note 本函数只复制样本，不执行 FFT；只能由主循环中的采集处理函数调用。
 */
void measurement_fft_ingest_pair(uint16_t ch1_raw_code,
                                 uint16_t ch2_raw_code);

/**
 * @brief 迁移期间接收一条旧 ADS8688 顺序采样记录。
 * @param channel 旧 ADS8688 通道号，仅接收 0 与 1。
 * @param raw_code 旧 ADS8688 直二进制原始码。
 * @return 无。
 * @note 仅用于让旧驱动继续编译；片上 ADC 正式链路不得调用本接口。
 */
void measurement_fft_ingest_sample(uint8_t channel, uint16_t raw_code);

/**
 * @brief 判断采集模块当前是否应继续提供同步样本对。
 * @param 无。
 * @return 采集或过渡状态返回 1，FFT 就绪或 HMI 显示状态返回 0。
 * @note adc_dual 模块据此启停 TIM2，避免 9600 波特率发送期间覆盖窗口。
 */
uint8_t measurement_fft_sampling_required(void);

/**
 * @brief 处理已收齐窗口，计算并发布幅度、频率、相位差和波形类型。
 * @param 无。
 * @return 无。
 * @note FFT 完成后进入 HMI 显示空档；下次采集前丢弃过渡样本，避免 9600 波特率发送造成窗口断裂。
 */
void measurement_fft_process(void);

/**
 * @brief 判断当前是否允许执行串口屏刷新。
 * @param 无。
 * @return 非零表示处于 FFT 完成后的显示空档；零表示正在采样、处理或过渡。
 * @note 显示空档结束后自动开始下一帧采集。
 */
uint8_t measurement_fft_hmi_refresh_allowed(void);

/**
 * @brief 获取最近一次 FFT 测量诊断快照。
 * @param diagnostics 用于接收诊断信息的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 仅供主循环或调试器读取，不修改 FFT 状态。
 */
uint8_t measurement_fft_get_diagnostics(measurement_fft_diagnostics_t *diagnostics);

/**
 * @brief 获取最近一次压缩频谱快照。
 * @param spectrum 用于接收 64 点相对幅度频谱的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 频谱固定覆盖 0 至 20 kHz，幅度以本帧最大谱线为 0 dB。
 */
uint8_t measurement_fft_get_spectrum(measurement_fft_spectrum_t *spectrum);

/**
 * @brief 设置 AIN0 或 AIN1 的软件校准系数。
 * @param channel 通道号，只允许 0 或 1。
 * @param calibration 码值比例、零码偏置和有效状态。
 * @return 参数有效时返回 1，否则返回 0。
 * @note valid 为零时仍分析频率、相位和波形，但不发布电压、幅度和有效值。
 */
uint8_t measurement_fft_set_calibration(
    uint8_t channel,
    const measurement_fft_calibration_t *calibration);

/**
 * @brief 读取 AIN0 或 AIN1 当前的软件校准系数。
 * @param channel 通道号，只允许 0 或 1。
 * @param calibration 用于接收校准系数的指针。
 * @return 参数有效时返回 1，否则返回 0。
 * @note 默认 valid 为零，前级方案确认前不得把原始码标注为真实电压。
 */
uint8_t measurement_fft_get_calibration(
    uint8_t channel,
    measurement_fft_calibration_t *calibration);

#endif /* MEASUREMENT_FFT_H */
