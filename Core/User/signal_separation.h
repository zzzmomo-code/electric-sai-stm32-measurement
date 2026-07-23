#ifndef SIGNAL_SEPARATION_H
#define SIGNAL_SEPARATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 模块用途：从 PC0 的混合输入中识别两个 5 kHz 栅格分量，并在 PA4/PA5
 *           上重建与输入相位跟踪的分离波形。
 * GPIO 映射：PC0=ADC1_INP10，PA4=DAC1_OUT1，PA5=DAC1_OUT2。
 * 外设依赖：ADC1、DAC1、TIM2、DMA1，以及已启用的 I-Cache/D-Cache。
 * 初始化方法：system_init() 调用 signal_separation_start()。
 * 调用方法：system_process() 持续调用 signal_separation_process()。
 */

typedef enum
{
  signal_wave_sine = 0,
  signal_wave_triangle = 1
} signal_wave_type_t;

typedef struct
{
  uint8_t identified;             /* 1 表示已经获得两个有效分量。 */
  uint32_t frequency_hz[2];       /* 两路已识别频率，单位 Hz。 */
  signal_wave_type_t wave[2];     /* 两路识别出的波形类型。 */
  uint32_t amplitude_adc[2];      /* 两路 ADC 峰值估计，单位为 ADC 码。 */
  int32_t phase_error_mdeg[2];    /* 两路最近相位误差，单位 0.001°。 */
  uint32_t adc_frame_count;       /* 主循环已接收的 ADC DMA 半帧数。 */
  uint32_t adc_frame_overrun;     /* ADC 事件来不及处理的累计次数。 */
  uint32_t dac_half_overrun;      /* DAC 半区来不及回填的累计次数。 */
} signal_separation_status_t;

/**
 * @brief 初始化数学表、校准 ADC，并同步启动 ADC/DAC DMA 与 TIM2。
 * @param 无。
 * @return 无。HAL 启动失败时进入 Error_Handler()。
 * @note 启动后 PC0 必须始终保持在 0～3.3 V，且输入不能悬空。
 */
void signal_separation_start(void);

/**
 * @brief 处理 DMA 标志、频率识别、锁相更新以及 DAC 半区回填。
 * @param 无。
 * @return 无。
 * @note 必须在主循环中尽可能频繁调用，函数内部不在中断里做算法计算。
 */
void signal_separation_process(void);

/**
 * @brief 清除当前识别结果并重新搜索两个输入分量。
 * @param 无。
 * @return 无。
 * @note 重识别完成前，两路 DAC 暂时输出中点电平。
 */
void signal_separation_restart_identify(void);

/**
 * @brief 设置 DAC2 相对于锁定输入的额外相位偏移。
 * @param degree 目标角度，自动限制到 0～180°并按 5°量化。
 * @return 无。
 */
void signal_separation_set_phase_offset_deg(int32_t degree);

/**
 * @brief 获取 DAC2 当前额外相位偏移。
 * @param 无。
 * @return 当前相位偏移，单位为度。
 */
int32_t signal_separation_get_phase_offset_deg(void);

/**
 * @brief 获取不会修改算法状态的运行快照。
 * @param status 非空的状态输出指针。
 * @return 1 表示参数有效并完成复制，0 表示 status 为空。
 */
uint8_t signal_separation_get_status(signal_separation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
