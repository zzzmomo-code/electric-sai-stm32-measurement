/**
 * @file adc_dual.h
 * @brief STM32H750 ADC1/ADC2 双路同步采集公共接口。
 *
 * 模块用途：接收 ADC1 与 ADC2 双重规则同步模式产生的 32 位 DMA 数据，拆分为
 * 同一触发时刻的两路 16 位采样，并在主循环中送入测量算法。
 * GPIO 引脚映射：计划使用 PC4/ADC1_INP4 作为 CH1，PB1/ADC2_INP5 作为 CH2；
 * 在 CubeMX 尚未生成 ADC 配置前，本模块保持“未配置”状态且不访问硬件。
 * 依赖的外设和 CubeIDE 配置：ADC1/ADC2 Dual Regular Simultaneous、TIM2 TRGO
 * 80 kHz、ADC1 DMA1 Stream0 Circular Word/Word，以及 DMA1 Stream0 中断。
 * 初始化方法：由 system_init() 调用 adc_dual_init()。
 * 调用方法：主循环只调用 system_process()，由其间接调用 adc_dual_process()。
 */

#ifndef ADC_DUAL_H
#define ADC_DUAL_H

#include <stdint.h>

/** 双 ADC DMA 中包含的 32 位同步样本对数量。 */
#define ADC_DUAL_DMA_WORD_COUNT 1024u

/** 单个 DMA 半缓冲区包含的同步样本对数量。 */
#define ADC_DUAL_DMA_HALF_WORD_COUNT (ADC_DUAL_DMA_WORD_COUNT / 2u)

/** 片上双 ADC 采集状态。 */
typedef enum
{
    ADC_DUAL_STATE_CUBEMX_NOT_READY = 0,
    ADC_DUAL_STATE_STOPPED,
    ADC_DUAL_STATE_RUNNING,
    ADC_DUAL_STATE_ERROR
} adc_dual_state_t;

/** 片上双 ADC 运行统计，供调试器和后续诊断页面读取。 */
typedef struct
{
    uint32_t dma_half_count;       /**< 已处理的 DMA 前半区次数。 */
    uint32_t dma_full_count;       /**< 已处理的 DMA 后半区次数。 */
    uint32_t error_count;          /**< ADC/DMA 错误事件数量。 */
    uint32_t overflow_count;       /**< ADC 硬件上溢错误数量。 */
    uint32_t backlog_count;        /**< 主循环一次领到前后半区标志的积压次数。 */
    uint32_t sample_pair_count;    /**< 已送入 FFT 边界的同步样本对数量。 */
    uint32_t dropped_pair_count;   /**< FFT 暂停采样时主动忽略的样本对数量。 */
    uint16_t ch1_min_code;         /**< CH1 启动以来的最小原始码。 */
    uint16_t ch1_max_code;         /**< CH1 启动以来的最大原始码。 */
    uint16_t ch2_min_code;         /**< CH2 启动以来的最小原始码。 */
    uint16_t ch2_max_code;         /**< CH2 启动以来的最大原始码。 */
    int32_t last_hal_status;       /**< 最近一次 HAL 初始化或启停操作返回值。 */
    adc_dual_state_t state;        /**< 当前配置、运行或错误状态。 */
    uint8_t cubemx_ready;          /**< 非零表示 adc.h 与 tim.h 已由 CubeMX 生成。 */
    uint8_t timer_running;         /**< 非零表示 TIM2 正在产生采样触发。 */
} adc_dual_stats_t;

/**
 * @brief 初始化双 ADC 采集模块。
 * @param 无。
 * @return 无。
 * @note CubeMX 配置就绪后按 ADC2、ADC1 的顺序校准，再启动多模式 DMA 与 TIM2。
 */
void adc_dual_init(void);

/**
 * @brief 在主循环领取 DMA 标志、维护缓存并提交同步样本对。
 * @param 无。
 * @return 无。
 * @note 不在中断中执行缓存维护、循环或 FFT。
 */
void adc_dual_process(void);

/**
 * @brief 拆分 ADC 双模式公共数据寄存器的 32 位数据。
 * @param packed_word ADC1 低 16 位、ADC2 高 16 位的打包数据。
 * @param ch1_code 用于接收 ADC1 原始码的指针，可为空。
 * @param ch2_code 用于接收 ADC2 原始码的指针，可为空。
 * @return 无。
 * @note 纯计算接口，供固件和主机测试共用。
 */
void adc_dual_unpack_word(uint32_t packed_word,
                          uint16_t *ch1_code,
                          uint16_t *ch2_code);

/**
 * @brief 获取双 ADC 运行统计快照。
 * @param stats 用于接收统计数据的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 只复制主循环维护的数据，不访问 DMA 缓冲区。
 */
uint8_t adc_dual_get_stats(adc_dual_stats_t *stats);

#endif /* ADC_DUAL_H */
