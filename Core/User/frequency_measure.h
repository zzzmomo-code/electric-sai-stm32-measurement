/**
 * @file frequency_measure.h
 * @brief TIM5 外部脉冲计数频率测量模块接口。
 *
 * 模块用途：使用 TIM5 连续统计外部脉冲，并结合 DWT 实际时间间隔计算平均频率。
 * GPIO 引脚映射：PA0/TIM5_CH1，输入 0～3.3 V CMOS 方波，最高频率 30 MHz。
 * 依赖的外设和 CubeIDE 配置：TIM5 外部时钟模式 1、TI1FP1 上升沿、无滤波；
 * TIM3 配置为 1 Hz 更新中断；Cortex-M7 DWT 周期计数器可用。
 * 初始化方法：由 system_init() 调用 frequency_measure_init()。
 * 调用方法：由 system_process() 持续调用 frequency_measure_process()，结果从
 * frequency_measure_hz 读取。
 */

#ifndef FREQUENCY_MEASURE_H
#define FREQUENCY_MEASURE_H

#include <stdint.h>

/** TIM3 更新中断标志，由中断回调置位并由主循环领取和清除。 */
extern volatile uint8_t frequency_measure_flag;

/** 最近一次有效测量的外部信号平均频率，单位为 Hz。 */
extern volatile float frequency_measure_hz;

/**
 * @brief 初始化外部频率测量模块并启动 TIM5 和 TIM3。
 * @param 无。
 * @return 无。
 * @note 初始化时清零一次 TIM5 CNT；不会清零 DWT，以免影响 FFT 耗时诊断。
 */
void frequency_measure_init(void);

/**
 * @brief 领取 TIM3 更新标志并根据 TIM5、DWT 差值更新频率结果。
 * @param 无。
 * @return 无。
 * @note 会在主循环中清除 frequency_measure_flag，并更新内部快照和频率结果。
 */
void frequency_measure_process(void);

#endif /* FREQUENCY_MEASURE_H */
