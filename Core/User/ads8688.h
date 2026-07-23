/**
 * @file ads8688.h
 * @brief ADS8688 采集模块公共接口。
 *
 * 模块用途：提供 ADS8688 初始化、生命周期、单双通道、量程、采样率及数据读取接口。
 * GPIO 引脚映射：PA15/SPI3_NSS 连接 FSYNC，PC10/SPI3_SCK 连接 SCLK，
 * PC11/SPI3_MISO 连接 SDO，PC12/SPI3_MOSI 连接 SDI，PD0 连接 DAISY，PD1 连接 RST。
 * 依赖的外设和 CubeIDE 配置：SPI3 32 位、CPOL Low、CPHA 2 Edge、硬件 NSS，
 * DMA1 Stream1 RX Circular Word/递增、Stream2 TX Circular Word/不递增及优先级 5 中断。
 * 初始化方法：CubeMX 外设初始化后调用 ads8688_init()，需要采样时再调用 ads8688_start()。
 * 调用方法：主循环调用 ads8688_process()；配置通过本头文件的公共接口完成。
 */

#ifndef ADS8688_H
#define ADS8688_H

#include <stdint.h>

/** ADS8688 公共接口返回状态。 */
typedef enum
{
    ADS8688_STATUS_OK = 0,
    ADS8688_STATUS_INVALID_ARGUMENT,
    ADS8688_STATUS_NOT_INITIALIZED,
    ADS8688_STATUS_HAL_ERROR,
    ADS8688_STATUS_VERIFY_ERROR
} ads8688_status_t;

/** ADS8688 采集模式。 */
typedef enum
{
    ADS8688_MODE_AUTO = 0,
    ADS8688_MODE_MANUAL
} ads8688_mode_t;

/** ADS8688 通道输入量程寄存器编码。 */
typedef enum
{
    ADS8688_RANGE_BIPOLAR_10V24 = 0x00,
    ADS8688_RANGE_BIPOLAR_5V12 = 0x01,
    ADS8688_RANGE_BIPOLAR_2V56 = 0x02,
    ADS8688_RANGE_UNIPOLAR_10V24 = 0x05,
    ADS8688_RANGE_UNIPOLAR_5V12 = 0x06
} ads8688_range_t;

/** 单个历史采样记录。 */
typedef struct
{
    uint32_t sample_index; /**< 采样序号。 */
    uint16_t raw_code;     /**< ADC 原始码。 */
    uint8_t channel;       /**< 采样通道号。 */
    uint8_t reserved;      /**< 对齐及后续扩展保留字段。 */
} ads8688_sample_t;

/** 单通道最新采样结果。 */
typedef struct
{
    uint32_t sample_index; /**< 最新采样序号。 */
    uint16_t raw_code;     /**< 最新 ADC 原始码。 */
    float voltage;         /**< 根据通道量程换算的电压值，单位为伏。 */
    uint8_t valid;         /**< 数据有效标志，非零表示有效。 */
} ads8688_latest_t;

/** ADS8688 运行诊断计数。 */
typedef struct
{
    uint32_t initialization_failures; /**< 初始化失败次数。 */
    uint32_t spi_dma_errors;          /**< SPI 或 DMA 传输错误次数。 */
    uint32_t lost_samples;            /**< 丢失采样数量。 */
    uint32_t history_overwrites;      /**< 历史缓冲区覆盖记录数量。 */
    uint32_t recoveries;              /**< 自动恢复成功次数。 */
} ads8688_diagnostics_t;

/**
 * @brief 初始化 ADS8688 用户模块。
 * @param 无。
 * @return 初始化结果状态。
 * @note 会配置器件工作状态并清空模块内部采集状态。
 */
ads8688_status_t ads8688_init(void);

/**
 * @brief 按当前通道配置启动 SPI3 循环 DMA 采样。
 * @param 无。
 * @return 启动结果。
 * @note 重复启动安全；初始化本身不会启动采样。
 */
ads8688_status_t ads8688_start(void);

/**
 * @brief 停止 SPI3 DMA 采样。
 * @param 无。
 * @return 停止结果。
 * @note 重复停止安全，已保存的量程和通道配置保持不变。
 */
ads8688_status_t ads8688_stop(void);

/**
 * @brief 设置单通道连续采样。
 * @param channel 物理输入通道，范围为 0 至 7。
 * @return 配置结果。
 * @note 运行中调用会安全停止并恢复 DMA；FFT 只发布主通道结果。
 */
ads8688_status_t ads8688_set_single_channel(uint8_t channel);

/**
 * @brief 设置 AIN0/AIN1 双通道自动轮询采样。
 * @param 无。
 * @return 配置结果。
 * @note 双通道结果按 AIN0、AIN1 顺序提交给 FFT，并补偿轮询时差。
 */
ads8688_status_t ads8688_set_dual_channel(void);

/**
 * @brief 获取当前模式下每个有效通道的标称采样率。
 * @param 无。
 * @return 单通道为 SPI 帧率，双通道为 SPI 帧率的一半，单位 sample/s。
 * @note SPI3 每帧按 32 位数据加 1 个周期帧间隔计算。
 */
float ads8688_get_effective_sample_rate_hz(void);

/**
 * @brief 处理 ADS8688 采集状态与待处理事件。
 * @param 无。
 * @return 无。
 * @note 应由主循环持续调用，可能更新最新数据、历史数据和诊断计数。
 */
void ads8688_process(void);

/**
 * @brief 设置自动扫描模式。
 * @param channel_mask 待自动扫描的通道位掩码。
 * @return 模式设置结果状态。
 * @note 成功后会改变后续采集通道序列。
 */
ads8688_status_t ads8688_set_auto_mode(uint8_t channel_mask);

/**
 * @brief 设置单通道手动采集模式。
 * @param channel 待采集通道号。
 * @return 模式设置结果状态。
 * @note 成功后后续采集仅针对指定通道。
 */
ads8688_status_t ads8688_set_manual_mode(uint8_t channel);

/**
 * @brief 设置指定通道的输入量程。
 * @param channel 待配置通道号。
 * @param range 目标输入量程。
 * @return 量程设置及回读校验结果状态。
 * @note 成功后会影响该通道原始码到电压的换算。
 */
ads8688_status_t ads8688_set_channel_range(uint8_t channel,
                                            ads8688_range_t range);

/**
 * @brief 读取指定通道当前使用的输入量程。
 * @param channel 待读取通道号。
 * @param range 用于接收当前量程的指针。
 * @return 成功返回 ADS8688_STATUS_OK；参数无效或模块未初始化时返回对应错误状态。
 * @note 只读取软件保存的已生效量程，不发起 SPI 传输。
 */
ads8688_status_t ads8688_get_channel_range(uint8_t channel,
                                            ads8688_range_t *range);

/**
 * @brief 按指定量程将 ADS8688 直二进制原始码换算为电压。
 * @param raw_code ADC 原始码。
 * @param range 输入量程。
 * @param voltage 用于接收换算电压的指针，单位为伏。
 * @return 换算成功返回 ADS8688_STATUS_OK，参数或量程无效时返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 纯计算接口，不访问硬件，也不依赖 ADS8688 是否已经初始化。
 */
ads8688_status_t ads8688_convert_raw_to_voltage(uint16_t raw_code,
                                                 ads8688_range_t range,
                                                 float *voltage);

/**
 * @brief 读取指定通道的最新采样结果。
 * @param channel 待读取通道号。
 * @param latest 用于接收最新采样结果的指针。
 * @return 读取结果状态。
 * @note 成功时会写入 latest 指向的结构体。
 */
ads8688_status_t ads8688_get_latest(uint8_t channel,
                                    ads8688_latest_t *latest);

/**
 * @brief 按时间顺序读取历史采样记录。
 * @param samples 用于接收历史记录的数组。
 * @param max_count 数组可接收的最大记录数。
 * @return 实际写入的历史记录数。
 * @note 已读取记录将从历史缓冲区移除。
 */
uint32_t ads8688_read_history(ads8688_sample_t *samples,
                              uint32_t max_count);

/**
 * @brief 清空全部历史采样记录。
 * @param 无。
 * @return 无。
 * @note 会丢弃当前尚未读取的历史记录。
 */
void ads8688_clear_history(void);

/**
 * @brief 读取 ADS8688 当前累计诊断信息。
 * @param diagnostics 用于接收诊断计数的结构体指针。
 * @return 参数有效时返回 ADS8688_STATUS_OK，空指针返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 读取前会同步历史缓冲区覆盖计数。
 */
ads8688_status_t ads8688_get_diagnostics(
    ads8688_diagnostics_t *diagnostics);

#endif /* ADS8688_H */
