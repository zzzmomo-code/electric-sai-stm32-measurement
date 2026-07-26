/**
 * @file ad9959.h
 * @brief AD9959 双通道 DDS 驱动公开接口。
 *
 * 模块用途：通过 SPI4 配置并回读 AD9959，启用 CH0、CH1，关闭 CH2、CH3。
 * GPIO 引脚映射：PE2/SPI4_SCK，PE5/SPI4_MISO 接 SDIO_2，
 * PE6/SPI4_MOSI 接 SDIO_0，PD5/CS，PD4/IO_UPDATE，PB4/RESET。
 * 依赖的外设和 CubeIDE 配置：SPI4 全双工主机、Motorola、8 位、MSB、
 * CPOL Low、CPHA 1 Edge、软件 NSS、15 MHz；控制脚由 GPIO 初始化。
 * 初始化方法：CubeMX 外设初始化完成后，由 system_init() 调用 ad9959_init()。
 * 调用方法：仅在主循环或普通任务中调用本接口，禁止在中断回调中阻塞访问 SPI。
 */

#ifndef AD9959_H
#define AD9959_H

#include <stdint.h>

/** AD9959 片内系统时钟：模块 25 MHz 晶振经 PLL 20 倍频。 */
#define AD9959_SYSTEM_CLOCK_HZ 500000000u

/** 上电自检时 CH0、CH1 的初始输出频率。 */
#define AD9959_INITIAL_FREQUENCY_HZ 1000000u

/** 当前模块低通滤波器允许的最高输出频率。 */
#define AD9959_MAX_OUTPUT_FREQUENCY_HZ 200000000u

/** AD9959 手动幅度缩放的满量程数值。 */
#define AD9959_AMPLITUDE_FULL_SCALE 1023u

/** 初始化回读不一致：FR1。 */
#define AD9959_MISMATCH_FR1       0x0001u
/** 初始化回读不一致：FR2。 */
#define AD9959_MISMATCH_FR2       0x0002u
/** 初始化回读不一致：CH0 CFR。 */
#define AD9959_MISMATCH_CH0_CFR   0x0004u
/** 初始化回读不一致：CH0 FTW。 */
#define AD9959_MISMATCH_CH0_FTW   0x0008u
/** 初始化回读不一致：CH0 CPOW。 */
#define AD9959_MISMATCH_CH0_CPOW  0x0010u
/** 初始化回读不一致：CH0 ACR。 */
#define AD9959_MISMATCH_CH0_ACR   0x0020u
/** 初始化回读不一致：CH1 CFR。 */
#define AD9959_MISMATCH_CH1_CFR   0x0040u
/** 初始化回读不一致：CH1 FTW。 */
#define AD9959_MISMATCH_CH1_FTW   0x0080u
/** 初始化回读不一致：CH1 CPOW。 */
#define AD9959_MISMATCH_CH1_CPOW  0x0100u
/** 初始化回读不一致：CH1 ACR。 */
#define AD9959_MISMATCH_CH1_ACR   0x0200u
/** 初始化回读不一致：CH2 未保持关闭。 */
#define AD9959_MISMATCH_CH2_CFR   0x0400u
/** 初始化回读不一致：CH3 未保持关闭。 */
#define AD9959_MISMATCH_CH3_CFR   0x0800u

/** AD9959 驱动返回状态。 */
typedef enum
{
    ad9959_status_ok = 0,
    ad9959_status_not_initialized,
    ad9959_status_invalid_argument,
    ad9959_status_invalid_channel,
    ad9959_status_invalid_register,
    ad9959_status_invalid_frequency,
    ad9959_status_invalid_phase,
    ad9959_status_invalid_amplitude,
    ad9959_status_spi_error,
    ad9959_status_readback_mismatch
} ad9959_status_t;

/** 对外开放的两个 AD9959 输出通道。 */
typedef enum
{
    ad9959_channel_0 = 0,
    ad9959_channel_1 = 1
} ad9959_channel_t;

/** 最小移植使用到的 AD9959 寄存器地址。 */
typedef enum
{
    ad9959_register_csr = 0x00,
    ad9959_register_fr1 = 0x01,
    ad9959_register_fr2 = 0x02,
    ad9959_register_cfr = 0x03,
    ad9959_register_ftw0 = 0x04,
    ad9959_register_cpow0 = 0x05,
    ad9959_register_acr = 0x06
} ad9959_register_t;

/** AD9959 运行诊断快照，供调试器和上板验收读取。 */
typedef struct
{
    uint32_t write_count;                 /**< 成功完成的完整寄存器写事务数。 */
    uint32_t read_count;                  /**< 成功完成的完整寄存器读事务数。 */
    uint32_t update_count;                /**< 已产生的 IO_UPDATE 脉冲数。 */
    uint32_t error_count;                 /**< 对外接口返回失败的次数。 */
    int32_t last_hal_status;              /**< 最近一次 SPI HAL 返回值。 */
    ad9959_status_t last_status;          /**< 最近一次驱动返回状态。 */
    uint32_t frequency_hz[2];             /**< CH0、CH1 当前期望频率。 */
    uint32_t frequency_tuning_word[2];    /**< CH0、CH1 当前期望 FTW。 */
    uint16_t phase_word[2];               /**< CH0、CH1 当前期望 14 位相位字。 */
    uint16_t amplitude_scale[2];          /**< CH0、CH1 当前期望 10 位幅度值。 */
    uint8_t initialized;                  /**< 写配置流程完成标志。 */
    uint8_t verified;                     /**< 最近一次完整回读校验通过标志。 */
    uint8_t selected_channel;             /**< 最近一次选择的物理通道 0 至 3。 */
    uint16_t readback_mismatch_mask;      /**< AD9959_MISMATCH_* 位图。 */
    uint8_t fr1_readback[3];              /**< 最近一次 FR1 回读。 */
    uint8_t fr2_readback[2];              /**< 最近一次 FR2 回读。 */
    uint8_t cfr_readback[4][3];           /**< 四个物理通道最近一次 CFR 回读。 */
    uint8_t ftw_readback[2][4];           /**< CH0、CH1 最近一次 FTW 回读。 */
    uint8_t cpow_readback[2][2];          /**< CH0、CH1 最近一次 CPOW 回读。 */
    uint8_t acr_readback[2][3];           /**< CH0、CH1 最近一次 ACR 回读。 */
} ad9959_diagnostics_t;

/** AD9959 诊断快照，仅由驱动调用上下文修改。 */
extern volatile ad9959_diagnostics_t ad9959_diagnostics;

/**
 * @brief 复位、配置并回读验证 AD9959。
 * @param 无。
 * @return ad9959_status_ok 表示写入和回读均通过，其他值表示失败原因。
 * @note 成功后 CH0、CH1 输出 1 MHz、0°、满幅；CH2、CH3 数字核和 DAC 均关闭。
 */
ad9959_status_t ad9959_init(void);

/**
 * @brief 按 500 MHz 系统时钟计算 32 位频率控制字。
 * @param frequency_hz 目标频率，单位 Hz。
 * @return 四舍五入后的 32 位 FTW。
 * @note 本函数只做确定性整数计算，不访问硬件。
 */
uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz);

/**
 * @brief 把整数角度转换为 14 位相位控制字。
 * @param phase_degrees 相位角，范围 0 至 359°。
 * @return 四舍五入后的 14 位相位字。
 * @note 本函数只做确定性整数计算，不访问硬件。
 */
uint16_t ad9959_calculate_phase_word(uint16_t phase_degrees);

/**
 * @brief 设置一个输出通道的频率并立即回读确认。
 * @param channel 输出通道，只允许 CH0 或 CH1。
 * @param frequency_hz 目标频率，范围 1 Hz 至 200 MHz。
 * @return 驱动状态。
 * @note 写 FTW 后产生一次 IO_UPDATE；回读只验证配置，不代表模拟输出相位。
 */
ad9959_status_t ad9959_set_frequency(ad9959_channel_t channel,
                                    uint32_t frequency_hz);

/**
 * @brief 设置一个输出通道的 14 位相位字并立即回读确认。
 * @param channel 输出通道，只允许 CH0 或 CH1。
 * @param phase_word 目标相位字，范围 0 至 0x3FFF。
 * @return 驱动状态。
 * @note 该相位偏置在 IO_UPDATE 后生效。
 */
ad9959_status_t ad9959_set_phase_word(ad9959_channel_t channel,
                                     uint16_t phase_word);

/**
 * @brief 使用整数角度设置一个输出通道的相位。
 * @param channel 输出通道，只允许 CH0 或 CH1。
 * @param phase_degrees 相位角，范围 0 至 359°。
 * @return 驱动状态。
 * @note 内部转换为 14 位相位字后调用 ad9959_set_phase_word()。
 */
ad9959_status_t ad9959_set_phase_degrees(ad9959_channel_t channel,
                                        uint16_t phase_degrees);

/**
 * @brief 设置一个输出通道的手动幅度缩放值并立即回读确认。
 * @param channel 输出通道，只允许 CH0 或 CH1。
 * @param amplitude_scale 幅度值，范围 0 至 1023。
 * @return 驱动状态。
 * @note 1023 为满量程，0 为最小幅度。
 */
ad9959_status_t ad9959_set_amplitude(ad9959_channel_t channel,
                                    uint16_t amplitude_scale);

/**
 * @brief 读取指定通道的一个寄存器。
 * @param channel 读取通道，只允许 CH0 或 CH1。
 * @param register_address 寄存器地址。
 * @param data 接收缓冲区。
 * @param length 必须等于目标寄存器的规定字节数。
 * @return 驱动状态。
 * @note 每次通道寄存器读取前均只选择一个通道，避免多通道回读冲突。
 */
ad9959_status_t ad9959_read_register(ad9959_channel_t channel,
                                    ad9959_register_t register_address,
                                    uint8_t *data,
                                    uint8_t length);

/**
 * @brief 完整回读 FR1/FR2、CH0/CH1 配置及 CH2/CH3 关闭状态。
 * @param 无。
 * @return 驱动状态。
 * @note 失败位保存在 ad9959_diagnostics.readback_mismatch_mask。
 */
ad9959_status_t ad9959_verify_configuration(void);

#endif /* AD9959_H */
