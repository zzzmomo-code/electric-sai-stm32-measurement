/**
 * @file measurement_result.h
 * @brief 测量结果快照公共接口。
 *
 * 模块用途：在测量算法和显示模块之间传递直流、幅度、有效值、频率、失真度、
 * 相位差及波形类型结果。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖，只能由主循环上下文访问。
 * 初始化方法：系统启动时调用 measurement_result_init()。
 * 调用方法：算法调用 measurement_result_publish()，显示模块调用
 * measurement_result_get_snapshot()。
 */

#ifndef MEASUREMENT_RESULT_H
#define MEASUREMENT_RESULT_H

#include <stdint.h>

/** 已识别的输入波形类型。 */
typedef enum
{
    MEASUREMENT_WAVE_UNKNOWN = 0,
    MEASUREMENT_WAVE_SINE,
    MEASUREMENT_WAVE_SQUARE,
    MEASUREMENT_WAVE_TRIANGLE
} measurement_wave_type_t;

/** 输入信号的测量模式。 */
typedef enum
{
    MEASUREMENT_MODE_UNKNOWN = 0,
    MEASUREMENT_MODE_DC,
    MEASUREMENT_MODE_AC
} measurement_mode_t;

/** 测量结果字段有效位；允许单通道接入时仍发布可用的部分结果。 */
#define MEASUREMENT_VALID_DC_VOLTAGE   (1u << 0)
#define MEASUREMENT_VALID_AMPLITUDE    (1u << 1)
#define MEASUREMENT_VALID_RMS          (1u << 2)
#define MEASUREMENT_VALID_FREQUENCY    (1u << 3)
#define MEASUREMENT_VALID_THD          (1u << 4)
#define MEASUREMENT_VALID_WAVE_TYPE    (1u << 5)
#define MEASUREMENT_VALID_PHASE        (1u << 6)
#define MEASUREMENT_VALID_SPECTRUM     (1u << 7)

/** 一组可供显示或记录的测量结果。 */
typedef struct
{
    float dc_voltage;                  /**< AIN0 平均直流电压，单位为伏。 */
    float amplitude_vpp;               /**< 峰峰值幅度，单位为伏。 */
    float rms_voltage;                 /**< 去直流后的交流有效值，单位为伏。 */
    float frequency_hz;                /**< 频率，单位为赫兹。 */
    float thd_percent;                 /**< 可用谐波范围内的总谐波失真，单位为百分比。 */
    float phase_deg;                   /**< 相位差，单位为度。 */
    measurement_wave_type_t wave_type; /**< 算法识别的波形类型。 */
    measurement_mode_t mode;           /**< 当前输入被判定为直流、交流或未知。 */
    uint16_t valid_mask;               /**< 各字段有效位，定义见 MEASUREMENT_VALID_*。 */
    uint8_t valid;                     /**< 非零表示现有五控件 HMI 可显示完整交流结果。 */
    uint32_t sequence;                 /**< 结果发布序号，由算法模块维护。 */
} measurement_result_t;

/**
 * @brief 初始化测量结果快照。
 * @param 无。
 * @return 无。
 * @note 初始化后快照无效，串口屏应显示 WAIT。
 */
void measurement_result_init(void);

/**
 * @brief 发布一组新的测量结果。
 * @param result 待发布结果的指针，空指针会被忽略。
 * @return 无。
 * @note 只能由主循环中的算法模块调用，不能在中断或 DMA 回调中调用。
 */
void measurement_result_publish(const measurement_result_t *result);

/**
 * @brief 读取当前测量结果快照。
 * @param result 用于接收快照的指针。
 * @return 已存在已发布快照时返回 1；空指针或尚未发布时返回 0。
 * @note 即使尚未发布，也会向有效输出指针写入 valid 为零的默认快照。
 */
uint8_t measurement_result_get_snapshot(measurement_result_t *result);

#endif /* MEASUREMENT_RESULT_H */
