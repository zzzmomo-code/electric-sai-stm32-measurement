/**
 * @file fft_probe.h
 * @brief CMSIS-DSP Q15 RFFT 链接与资源占用验证接口。
 *
 * 模块用途：在接入真实采样算法之前，验证 8192 点 Q15 RFFT 所需代码和查表是否
 * 能被工程正确链接，并为 Flash 占用评估提供实际构建结果。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖；依赖已加入工程的 CMSIS-DSP Q15
 * 源文件、DSP Include 路径及 8192 点查表宏。
 * 初始化方法：系统启动时调用 fft_probe_init() 一次。
 * 调用方法：仅在本轮资源验证期间调用，不参与 ADS8688 实时采样或 HMI 显示。
 */

#ifndef FFT_PROBE_H
#define FFT_PROBE_H

#include <stdint.h>

/** CMSIS-DSP 初始化返回值，ARM_MATH_SUCCESS 为 0。 */
extern int32_t fft_probe_init_status;

/** 非零表示 8192 点 RFFT 已实际执行一次。 */
extern uint8_t fft_probe_executed;

/**
 * @brief 执行一次全零 8192 点 Q15 实数 FFT 验证。
 * @param 无。
 * @return 无。
 * @note 该函数不访问 ADS8688 DMA 缓冲区；输入与输出为本模块私有静态缓冲区。
 */
void fft_probe_init(void);

#endif /* FFT_PROBE_H */
