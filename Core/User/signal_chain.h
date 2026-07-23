/**
 * @file signal_chain.h
 * @brief ADC/DAC DMA 同步波形链路和运行模式控制接口。
 *
 * 模块用途：管理 ADC1、DAC1、TIM2、DMA 双半缓冲和 D-Cache 一致性。
 * GPIO 引脚：PC0=ADC1_INP10，PA4=DAC1_OUT1。
 * 依赖外设：ADC1、DAC1、TIM2、DMA1 Stream0/1，均由 CubeMX 初始化。
 * 初始化方法：system_init() 中调用 signal_chain_init()。
 * 调用方法：主循环持续调用 signal_chain_process()。
 */

#ifndef USER_SIGNAL_CHAIN_H
#define USER_SIGNAL_CHAIN_H

#include <stdint.h>

typedef enum
{
    SIGNAL_MODE_DIRECT = 0,
    SIGNAL_MODE_DPLL
} signal_mode_t;

typedef struct
{
    signal_mode_t mode;
    dpll_lock_state_t lock_state;
    float frequency_hz;
    float coarse_frequency_hz;
    float phase_error_deg;
    float amplitude_adc_counts;
    float offset_adc_counts;
    float target_phase_deg;
    uint32_t processed_half_blocks;
    uint32_t adc_error_count;
    uint32_t dac_error_count;
    uint32_t processing_overrun_count;
    uint8_t running;
} signal_chain_status_t;

extern volatile uint8_t adc_dma_half_ready_flag;
extern volatile uint8_t adc_dma_full_ready_flag;
extern volatile uint8_t adc_error_flag;
extern volatile uint8_t dac_error_flag;
extern volatile uint8_t dac_underrun_flag;

void signal_chain_init(void);
void signal_chain_process(void);
void signal_chain_set_mode(signal_mode_t mode);
signal_mode_t signal_chain_get_mode(void);
void signal_chain_set_target_phase_deg(float phase_deg);
float signal_chain_get_target_phase_deg(void);
void signal_chain_reset_lock(void);
void signal_chain_get_status(signal_chain_status_t *status);
uint8_t signal_chain_has_pending_work(void);

#endif /* USER_SIGNAL_CHAIN_H */
