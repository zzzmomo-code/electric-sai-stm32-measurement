/**
 * @file ad9834_2.h
 * @brief 第二块AD9834 DDS独立底层驱动接口。
 *
 * 模块用途：通过SPI6向第二块AD9834写入16位控制字，独立设置两组频率和相位寄存器。
 * GPIO引脚映射：PB3/SPI6_SCK，PB5/SPI6_MOSI，PD5/FSYNC，
 * PD6/FSELECT，PD7/PSELECT，PB4/RESET。
 * 依赖的外设和CubeIDE配置：SPI6主机只发送、16位、MSB优先、
 * CPOL=High、CPHA=1 Edge、30 Mbit/s；FSYNC空闲为高电平，
 * FSELECT、PSELECT和RESET正常运行时为低电平。
 * 初始化方法：由system_init()调用ad9834_2_init(900000u)，初始化两组频率和相位寄存器。
 * 调用方法：主循环可写入目标寄存器并切换FSELECT/PSELECT，禁止在中断中调用。
 */

#ifndef AD9834_2_H
#define AD9834_2_H

#include <stdint.h>

/** 第二块AD9834板载主时钟频率，单位Hz。 */
#define AD9834_2_MCLK_HZ 75000000u

/** 第二块AD9834允许的最高输出频率，单位Hz。 */
#define AD9834_2_MAX_OUTPUT_HZ 30000000u

/** 第二块AD9834驱动返回状态。 */
typedef enum
{
    ad9834_2_status_ok = 0,
    ad9834_2_status_invalid_frequency,
    ad9834_2_status_spi_error,
    ad9834_2_status_invalid_register,
    ad9834_2_status_invalid_phase
} ad9834_2_status_t;

/** 第二块AD9834频率寄存器选择，对应PD6/FSELECT电平。 */
typedef enum
{
    ad9834_2_frequency_register_0 = 0,
    ad9834_2_frequency_register_1 = 1
} ad9834_2_frequency_register_t;

/** 第二块AD9834相位寄存器选择，对应PD7/PSELECT电平。 */
typedef enum
{
    ad9834_2_phase_register_0 = 0,
    ad9834_2_phase_register_1 = 1
} ad9834_2_phase_register_t;

/** 第二块AD9834运行诊断，便于在调试器Expressions中观察。 */
typedef struct
{
    uint32_t output_frequency_hz; /**< 最近成功设置的输出频率。 */
    uint32_t tuning_word; /**< 最近成功设置的28位频率字。 */
    uint32_t write_count; /**< SPI6成功发送的16位字数量。 */
    uint32_t error_count; /**< SPI6发送失败次数。 */
    uint16_t last_word; /**< 最近尝试发送的16位字。 */
    int32_t last_hal_status; /**< 最近一次HAL SPI返回值。 */
    uint8_t initialized; /**< 完整初始化成功后为1。 */
    uint8_t selected_frequency_register; /**< 当前FSELECT选择，0为FREQ0。 */
    uint8_t selected_phase_register; /**< 当前PSELECT选择，0为PHASE0。 */
    uint32_t frequency_hz[2]; /**< FREQ0和FREQ1最近成功写入的频率。 */
    uint32_t frequency_tuning_word[2]; /**< 两组频率寄存器的28位频率字。 */
    uint16_t phase_degrees[2]; /**< PHASE0和PHASE1最近成功写入的整数角度。 */
    uint16_t phase_word[2]; /**< 两组相位寄存器的12位相位字。 */
    uint8_t last_frequency_register; /**< 最近成功写入的频率寄存器编号。 */
    uint8_t last_phase_register; /**< 最近成功写入的相位寄存器编号。 */
} ad9834_2_diagnostics_t;

/** 第二块AD9834运行诊断快照。 */
extern volatile ad9834_2_diagnostics_t ad9834_2_diagnostics;

/**
 * @brief 初始化第二块AD9834并输出指定频率的正弦波。
 * @param initial_frequency_hz 初始输出频率，单位Hz。
 * @return 驱动状态。
 * @note 初始化期间硬件RESET保持高电平；失败时保持复位，成功后切换至FREQ0和PHASE0。
 */
ad9834_2_status_t ad9834_2_init(uint32_t initial_frequency_hz);

/**
 * @brief 更新第二块AD9834的FREQ0输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 阻塞发送两个16位字，不改变FSELECT，禁止在中断中调用。
 */
ad9834_2_status_t ad9834_2_set_frequency_hz(uint32_t frequency_hz);

/**
 * @brief 将指定频率写入第二块AD9834的FREQ0或FREQ1。
 * @param frequency_register 目标频率寄存器。
 * @param frequency_hz 目标频率，单位Hz。
 * @return 驱动状态。
 * @note 完整发送低14位和高14位后才更新诊断，不改变FSELECT。
 */
ad9834_2_status_t ad9834_2_set_frequency_register_hz(
    ad9834_2_frequency_register_t frequency_register,
    uint32_t frequency_hz);

/**
 * @brief 将整数角度写入第二块AD9834的PHASE0或PHASE1。
 * @param phase_register 目标相位寄存器。
 * @param phase_degrees 目标相位，范围0至359度。
 * @return 驱动状态。
 * @note 阻塞发送一个16位字，不改变PSELECT，禁止在中断中调用。
 */
ad9834_2_status_t ad9834_2_set_phase_register_degrees(
    ad9834_2_phase_register_t phase_register,
    uint16_t phase_degrees);

/**
 * @brief 通过PD6/FSELECT选择第二块AD9834频率寄存器。
 * @param frequency_register 要选择的FREQ0或FREQ1。
 * @return 无。
 * @note 只切换引脚，不发送SPI数据；非法枚举不改变输出。
 */
void ad9834_2_select_frequency_register(
    ad9834_2_frequency_register_t frequency_register);

/**
 * @brief 通过PD7/PSELECT选择第二块AD9834相位寄存器。
 * @param phase_register 要选择的PHASE0或PHASE1。
 * @return 无。
 * @note 只切换引脚，不发送SPI数据；非法枚举不改变输出。
 */
void ad9834_2_select_phase_register(
    ad9834_2_phase_register_t phase_register);

/**
 * @brief 计算第二块AD9834的28位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 按75MHz MCLK四舍五入后的28位频率控制字。
 */
uint32_t ad9834_2_calculate_tuning_word(uint32_t frequency_hz);

/**
 * @brief 使用双频率寄存器无中断地更新第二块AD9834输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 先写入当前非活动频率寄存器，完整成功后才切换FSELECT；
 * 参数或SPI写入失败时保持当前输出不变，禁止在中断中调用。
 */
ad9834_2_status_t dds2_set_frequency(uint32_t frequency_hz);

#endif /* AD9834_2_H */
