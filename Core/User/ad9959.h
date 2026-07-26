/**
 * @file ad9959.h
 * @brief AD9959 DDS底层驱动接口。
 *
 * 模块用途：向AD9959写入寄存器，分别设置两个通道（CH0+CH1）的频率、相位和
 * 幅度。同时提供读寄存器能力，便于数字链路验证。
 * GPIO引脚映射：PE2/SPI4_SCK，PE5/SPI4_MISO，PE6/SPI4_MOSI，
 * PD4/IO_UPDATE，PD5/CS（低有效），PB4/RESET（高有效复位）。
 * 模块固定电平：PDC、SDIO_3/SYNC_I/O及P0～P3必须从模块端接GND；
 * SDIO_1未使用。SDIO_3在单位串行模式下禁止浮空。
 * 依赖的外设和CubeIDE配置：保留SPI4主机配置；当前排障版本在ad9959_init()
 * 中临时接管PE2/PE5/PE6为GPIO并模拟Mode 0串行时序，以隔离H7 SPI外设问题。
 * CS和RESET由软件控制，
 * IO_UPDATE上升沿刷新寄存器；25MHz外部晶振经片内PLL 20倍频得到500MHz系统时钟。
 * 初始化方法：由system_init()调用ad9959_init()，初始化两个通道为1 MHz正弦波、
 * 相位0度、幅度满量程。
 * 调用方法：主循环中可分别设置两个通道的频率、相位和幅度；禁止在中断中调用。
 */

#ifndef AD9959_H
#define AD9959_H

#include <stdint.h>

/** AD9959板载25MHz晶振经片内PLL 20倍频后的系统时钟，单位Hz。 */
#define AD9959_MCLK_HZ 500000000u

/** 本项目允许的最高DDS输出频率，单位Hz（取SYSCLK/2的保守值）。 */
#define AD9959_MAX_OUTPUT_HZ 200000000u

/** AD9959幅度量程上限（10位分辨率，0-1023）。 */
#define AD9959_AMPLITUDE_MAX 1023u

/** AD9959读寄存器时单次最大字节数，用于防止越界。 */
#define AD9959_READ_MAX_BYTES 8u

/** 串行传输选择：1表示临时使用与商家例程同序的GPIO模拟串行接口。 */
#define AD9959_USE_GPIO_BITBANG 1u

/**
 * 临时总线探针开关：完成串行链路验证后保持为0，使初始化后的AD9959总线静止。
 * 需要重新观察CS/SCLK/SDIO时可临时改为1，验证结束后必须恢复为0。
 */
#define AD9959_BUS_PROBE_ENABLE 0u

/** 临时总线探针重复周期，单位ms。 */
#define AD9959_BUS_PROBE_PERIOD_MS 200u

/** 临时总线探针每周期保持CS低电平的时间，单位ms，便于示波器和万用表确认。 */
#define AD9959_BUS_PROBE_CS_LOW_MS 40u

/** IO_UPDATE诊断高电平时间，单位ms；40ms脉冲便于在模块排针处直接观察。 */
#define AD9959_IO_UPDATE_HIGH_MS 40u

/** 临时总线探针固件签名，ASCII为“RDE1”，用于确认下降沿读回诊断固件。 */
#define AD9959_BUS_PROBE_SIGNATURE 0x52444531u

/** 初始化回读不一致位：FR1 全局寄存器。 */
#define AD9959_READBACK_MISMATCH_FR1      0x01u
/** 初始化回读不一致位：CH0 的 CFR 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH0_CFR  0x02u
/** 初始化回读不一致位：CH0 的 CFTW0 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH0_FTW  0x04u
/** 初始化回读不一致位：CH1 的 CFR 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH1_CFR  0x08u
/** 初始化回读不一致位：CH1 的 CFTW0 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH1_FTW  0x10u
/** 初始化回读不一致位：CH0 的 ACR 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH0_ACR  0x20u
/** 初始化回读不一致位：CH1 的 ACR 寄存器。 */
#define AD9959_READBACK_MISMATCH_CH1_ACR  0x40u

/** AD9959驱动返回状态。 */
typedef enum
{
    ad9959_status_ok = 0,
    ad9959_status_invalid_frequency,
    ad9959_status_invalid_phase,
    ad9959_status_invalid_amplitude,
    ad9959_status_invalid_channel,
    ad9959_status_invalid_length,
    ad9959_status_spi_error
} ad9959_status_t;

/** AD9959通道选择，对应CSR寄存器bit4/bit5；驱动同时保持CSR[2:1]三线模式。 */
typedef enum
{
    ad9959_channel_0 = 0,
    ad9959_channel_1 = 1
} ad9959_channel_t;

/** AD9959运行诊断，便于在调试器Expressions中观察。 */
typedef struct
{
    uint32_t write_count;                /**< 成功完成的串行写寄存器次数。 */
    uint32_t error_count;                /**< 串行传输流程失败次数。 */
    int32_t last_hal_status;             /**< 最近一次传输状态；GPIO模拟路径完成时为HAL_OK。 */
    uint8_t initialized;                 /**< 完整初始化成功后为1。 */
    uint8_t last_channel;                /**< 最近成功写入的通道编号。 */
    uint32_t frequency_hz[2];            /**< 两个通道最近成功写入的频率。 */
    uint32_t frequency_tuning_word[2];   /**< 两个通道的32位频率字。 */
    uint16_t phase_degrees[2];           /**< 两个通道最近成功写入的整数角度。 */
    uint16_t phase_word[2];              /**< 两个通道的14位相位字。 */
    uint16_t amplitude[2];               /**< 两个通道的10位幅度值。 */
    uint8_t readback_complete;           /**< 初始化末尾7项寄存器均完成串行回读流程后为1。 */
    uint8_t readback_mismatch_mask;      /**< 回读值不一致位，使用AD9959_READBACK_MISMATCH_*解析。 */
    uint8_t fr1_readback[3];             /**< 初始化末尾读回的FR1原始字节。 */
    uint8_t cfr_readback[2][3];          /**< 初始化末尾分别读回的CH0/CH1 CFR原始字节。 */
    uint8_t ftw_readback[2][4];          /**< 初始化末尾分别读回的CH0/CH1 CFTW0原始字节。 */
    uint8_t acr_readback[2][3];          /**< 初始化末尾分别读回的CH0/CH1 ACR原始字节。 */
    uint32_t bus_probe_count;            /**< 临时总线探针已执行的周期数。 */
    int32_t bus_probe_last_hal_status;   /**< 临时总线探针最近一次传输状态。 */
    uint8_t bus_probe_fr1[3];            /**< 临时总线探针最近一次读回的FR1字节。 */
    uint32_t bus_probe_signature;        /**< 固件签名，必须等于AD9959_BUS_PROBE_SIGNATURE。 */
    uint32_t bitbang_clock_edges;        /**< GPIO模拟串行已产生的SCLK上升沿总数。 */
    uint8_t bitbang_gpio_ready;          /**< PE2/PE5/PE6完成GPIO接管后为1。 */
    uint8_t bitbang_sclk_idle_odr;       /**< 最近事务结束后PE2输出锁存值，期望为0。 */
    uint8_t bitbang_sdio0_idle_odr;      /**< 最近事务结束后PE6输出锁存值，期望为0。 */
    uint8_t bitbang_sdio2_idle_idr;      /**< 最近事务结束后PE5实际输入值。 */
    uint32_t bus_probe_cs_low_count;     /**< 已实际下达PD5拉低命令的次数。 */
    uint32_t bus_probe_cs_low_tick;      /**< 最近一次PD5拉低命令的HAL毫秒时刻。 */
    uint32_t bus_probe_cs_high_tick;     /**< 最近一次PD5恢复高电平的HAL毫秒时刻。 */
    uint8_t bus_probe_state;             /**< 0=等待下一周期，1=正在保持CS低电平。 */
    uint8_t bus_probe_cs_low_odr;        /**< 拉低后PD5的GPIO ODR位，期望为0。 */
    uint8_t bus_probe_cs_low_idr;        /**< 拉低后PD5的GPIO IDR位，期望为0。 */
    uint8_t bus_probe_cs_high_odr;       /**< 拉高后PD5的GPIO ODR位，期望为1。 */
    uint8_t bus_probe_cs_high_idr;       /**< 拉高后PD5的GPIO IDR位，期望为1。 */
    uint32_t io_update_count;            /**< PD4已完成的IO_UPDATE高脉冲总数。 */
    uint8_t io_update_high_odr;          /**< PD4置高后的GPIO ODR位，期望为1。 */
    uint8_t io_update_high_idr;          /**< PD4置高后的GPIO IDR位，期望为1。 */
    uint8_t io_update_low_odr;           /**< PD4恢复低电平后的GPIO ODR位，期望为0。 */
    uint8_t io_update_low_idr;           /**< PD4恢复低电平后的GPIO IDR位，期望为0。 */
} ad9959_diagnostics_t;

/** AD9959运行诊断快照。 */
extern volatile ad9959_diagnostics_t ad9959_diagnostics;

/**
 * @brief 初始化AD9959并输出默认1 MHz正弦波。
 * @return 驱动状态。
 * @note 执行硬件RESET、配置PLL 20倍频、初始化CH0和CH1为1MHz/0度/满幅度。
 *       任一SPI写入失败时保持硬件复位状态，禁止在中断中调用。
 */
ad9959_status_t ad9959_init(void);

/**
 * @brief 设置指定通道的输出频率。
 * @param channel 目标通道，ad9959_channel_0或ad9959_channel_1。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 先写CSR选择通道，再写4字节CFTW0寄存器，最后产生IO_UPDATE刷新。
 *       阻塞式SPI调用，禁止在中断中调用。
 */
ad9959_status_t ad9959_set_frequency(ad9959_channel_t channel,
                                     uint32_t frequency_hz);

/**
 * @brief 设置指定通道的输出相位。
 * @param channel 目标通道。
 * @param phase_degrees 目标相位，范围0至359度。
 * @return 驱动状态。
 * @note 先写CSR选择通道，再写2字节CPOW0寄存器，最后产生IO_UPDATE刷新。
 *       阻塞式SPI调用，禁止在中断中调用。
 */
ad9959_status_t ad9959_set_phase(ad9959_channel_t channel,
                                 uint16_t phase_degrees);

/**
 * @brief 设置指定通道的输出幅度。
 * @param channel 目标通道。
 * @param amplitude 目标幅度，范围0至1023（0=零输出，1023=满量程）。
 * @return 驱动状态。
 * @note 先写CSR选择通道，再写3字节ACR寄存器（bit12置1启用手动幅度控制），
 *       最后产生IO_UPDATE刷新。阻塞式SPI调用，禁止在中断中调用。
 */
ad9959_status_t ad9959_set_amplitude(ad9959_channel_t channel,
                                     uint16_t amplitude);

/**
 * @brief 通过SPI读回AD9959的寄存器值。
 * @param address 寄存器地址（0x00-0x06）。
 * @param data 存放读回数据的缓冲区，调用者保证容量不少于length字节。
 * @param length 要读的字节数，范围1至AD9959_READ_MAX_BYTES。
 * @return 驱动状态。
 * @note 指令字节bit7置1表示读，随后通过MISO读回数据。读操作不产生IO_UPDATE。
 *       阻塞式SPI调用，禁止在中断中调用。
 */
ad9959_status_t ad9959_read_register(uint8_t address, uint8_t *data,
                                     uint8_t length);

/**
 * @brief 周期执行便于示波器触发的固定SPI写入和读取。
 * @param 无。
 * @return 无，执行次数、HAL状态和FR1数据保存在ad9959_diagnostics。
 * @note 启用AD9959_BUS_PROBE_ENABLE后每200ms先把CS保持低电平40ms，
 *       再发送CSR写帧00 12，随后发送FR1读指令81并读取3字节；仅在主循环调用。
 */
void ad9959_bus_probe_process(void);

/**
 * @brief 计算AD9959的32位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 按500MHz MCLK四舍五入后的32位频率控制字。
 */
uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz);

#endif /* AD9959_H */
