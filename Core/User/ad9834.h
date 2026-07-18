/**
 * @file ad9834.h
 * @brief AD9834 DDS底层驱动接口。
 *
 * 模块用途：通过SPI2向AD9834写入16位控制字并设置正弦输出频率。
 * GPIO引脚映射：PB12/FSYNC，PB13/SPI2_SCK，PB15/SPI2_MOSI。
 * 依赖的外设和CubeIDE配置：SPI2主机只发送、16位、MSB优先、
 * CPOL=High、CPHA=1 Edge、8 Mbit/s，FSYNC为空闲高电平GPIO输出。
 * 初始化方法：由dds_control_init()调用ad9834_init()。
 * 调用方法：初始化后调用ad9834_set_frequency_hz()更新频率。
 */

#ifndef AD9834_H
#define AD9834_H

#include <stdint.h>

/** AD9834板载主时钟频率，单位Hz。 */
#define AD9834_MCLK_HZ 75000000u

/** 本项目允许的最高DDS输出频率，单位Hz。 */
#define AD9834_MAX_OUTPUT_HZ 30000000u

/** AD9834驱动返回状态。 */
typedef enum
{
    ad9834_status_ok = 0,
    ad9834_status_invalid_frequency,
    ad9834_status_spi_error
} ad9834_status_t;

/** AD9834运行诊断，便于在调试器Expressions中直接观察。 */
typedef struct
{
    uint32_t output_frequency_hz; /**< 最近成功设置的输出频率。 */
    uint32_t tuning_word;         /**< 最近成功设置的28位频率字。 */
    uint32_t write_count;         /**< 成功发送的16位字数量。 */
    uint32_t error_count;         /**< SPI发送失败次数。 */
    uint16_t last_word;           /**< 最近尝试发送的16位字。 */
    int32_t last_hal_status;       /**< 最近一次HAL SPI返回值。 */
    uint8_t initialized;           /**< 初始化成功后为1。 */
} ad9834_diagnostics_t;

/** AD9834运行诊断快照。 */
extern volatile ad9834_diagnostics_t ad9834_diagnostics;

/**
 * @brief 初始化AD9834并输出指定频率的正弦波。
 * @param initial_frequency_hz 初始输出频率，单位Hz。
 * @return 驱动状态。
 * @note 使用软件RESET位，不需要MCU控制AD9834的RESET引脚。
 */
ad9834_status_t ad9834_init(uint32_t initial_frequency_hz);

/**
 * @brief 更新AD9834的FREQ0输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 函数使用阻塞式SPI发送两个16位字，调用时间很短但不能放在中断中。
 */
ad9834_status_t ad9834_set_frequency_hz(uint32_t frequency_hz);

/**
 * @brief 计算AD9834的28位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 四舍五入后的28位频率控制字。
 */
uint32_t ad9834_calculate_tuning_word(uint32_t frequency_hz);

#endif /* AD9834_H */
