/**
 * @file ads8688_storage.c
 * @brief ADS8688 最新采样和历史采样存储实现。
 *
 * 模块用途：完成原始码电压换算、各通道最新值维护和历史记录环形存储。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖，不需要额外 CubeIDE 配置。
 * 初始化方法：系统初始化阶段调用 ads8688_storage_init()。
 * 调用方法：每次获得有效采样后调用 ads8688_storage_push()，由读取接口消费数据。
 */

#include "system.h"

/** 历史采样环形缓冲区，固定保存最多 4096 条记录。 */
static ads8688_sample_t ads8688_history[ADS8688_HISTORY_CAPACITY];

/** 八个输入通道各自的最新采样结果。 */
static ads8688_latest_t ads8688_latest[ADS8688_CHANNEL_COUNT];

/** 当前最旧历史记录在环形缓冲区中的下标。 */
static uint32_t ads8688_history_head;

/** 当前尚未读取的历史记录数量。 */
static uint32_t ads8688_history_count;

/** 历史缓冲区满后覆盖最旧记录的累计次数。 */
static uint32_t ads8688_history_overwrite_count;

/**
 * @brief 按指定量程将 ADS8688 直二进制原始码换算为电压。
 * @param raw_code ADC 原始码。
 * @param range 输入量程。
 * @param voltage 用于接收换算电压的指针，单位为伏。
 * @return 换算成功返回 ADS8688_STATUS_OK，参数或量程无效时返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 纯计算接口，无硬件访问和内部状态副作用。
 */
ads8688_status_t ads8688_convert_raw_to_voltage(uint16_t raw_code,
                                                 ads8688_range_t range,
                                                 float *voltage)
{
    float raw_value = (float)raw_code;

    if (voltage == 0)
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    switch (range)
    {
        case ADS8688_RANGE_BIPOLAR_10V24:
            *voltage = raw_value * 20.48f / 65536.0f - 10.24f;
            break;

        case ADS8688_RANGE_BIPOLAR_5V12:
            *voltage = raw_value * 10.24f / 65536.0f - 5.12f;
            break;

        case ADS8688_RANGE_BIPOLAR_2V56:
            *voltage = raw_value * 5.12f / 65536.0f - 2.56f;
            break;

        case ADS8688_RANGE_UNIPOLAR_10V24:
            *voltage = raw_value * 10.24f / 65536.0f;
            break;

        case ADS8688_RANGE_UNIPOLAR_5V12:
            *voltage = raw_value * 5.12f / 65536.0f;
            break;

        default:
            *voltage = 0.0f;
            return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    return ADS8688_STATUS_OK;
}

/**
 * @brief 初始化 ADS8688 采样存储模块。
 * @param 无。
 * @return 无。
 * @note 清空历史状态、覆盖计数以及全部通道最新采样结果。
 */
void ads8688_storage_init(void)
{
    uint32_t channel;

    ads8688_history_head = 0u;
    ads8688_history_count = 0u;
    ads8688_history_overwrite_count = 0u;

    for (channel = 0u; channel < ADS8688_CHANNEL_COUNT; channel++)
    {
        ads8688_latest[channel].sample_index = 0u;
        ads8688_latest[channel].raw_code = 0u;
        ads8688_latest[channel].voltage = 0.0f;
        ads8688_latest[channel].valid = 0u;
    }
}

/**
 * @brief 保存一次 ADS8688 采样结果。
 * @param channel 采样通道号。
 * @param raw_code ADC 直二进制原始码。
 * @param range 当前输入量程。
 * @param sample_index 本次采样序号。
 * @return 无。
 * @note 有效通道会更新最新值和历史缓冲区，缓冲区满时覆盖最旧记录。
 */
void ads8688_storage_push(uint8_t channel,
                          uint16_t raw_code,
                          ads8688_range_t range,
                          uint32_t sample_index)
{
    float voltage;
    uint32_t write_index;

    if (channel >= ADS8688_CHANNEL_COUNT)
    {
        return;
    }

    if (ads8688_convert_raw_to_voltage(raw_code, range, &voltage)
        != ADS8688_STATUS_OK)
    {
        return;
    }

    ads8688_latest[channel].sample_index = sample_index;
    ads8688_latest[channel].raw_code = raw_code;
    ads8688_latest[channel].voltage = voltage;
    ads8688_latest[channel].valid = 1u;

    if (ads8688_history_count < ADS8688_HISTORY_CAPACITY)
    {
        write_index =
            (ads8688_history_head + ads8688_history_count) %
            ADS8688_HISTORY_CAPACITY;
        ads8688_history_count++;
    }
    else
    {
        write_index = ads8688_history_head;
        ads8688_history_head =
            (ads8688_history_head + 1u) % ADS8688_HISTORY_CAPACITY;
        ads8688_history_overwrite_count++;
    }

    ads8688_history[write_index].sample_index = sample_index;
    ads8688_history[write_index].raw_code = raw_code;
    ads8688_history[write_index].channel = channel;
    ads8688_history[write_index].reserved = 0u;
}

/**
 * @brief 读取指定通道的最新采样结果。
 * @param channel 待读取的通道号。
 * @param latest 用于接收最新采样结果的指针。
 * @return 参数有效时返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 成功时会覆盖 latest 指向的结构体内容。
 */
ads8688_status_t ads8688_storage_get_latest(uint8_t channel,
                                             ads8688_latest_t *latest)
{
    if ((channel >= ADS8688_CHANNEL_COUNT) || (latest == 0))
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    *latest = ads8688_latest[channel];
    return ADS8688_STATUS_OK;
}

/**
 * @brief 按从旧到新的顺序读取历史采样记录。
 * @param samples 用于接收历史记录的数组。
 * @param max_count 数组最多可接收的记录数量。
 * @return 实际复制并移除的历史记录数量。
 * @note 读取会推进环形缓冲区头下标并减少待读记录数量。
 */
uint32_t ads8688_storage_read(ads8688_sample_t *samples,
                              uint32_t max_count)
{
    uint32_t copy_count;
    uint32_t index;

    if ((samples == 0) || (max_count == 0u))
    {
        return 0u;
    }

    copy_count = max_count;
    if (copy_count > ads8688_history_count)
    {
        copy_count = ads8688_history_count;
    }

    for (index = 0u; index < copy_count; index++)
    {
        samples[index] =
            ads8688_history[(ads8688_history_head + index) %
                            ADS8688_HISTORY_CAPACITY];
    }

    ads8688_history_head =
        (ads8688_history_head + copy_count) % ADS8688_HISTORY_CAPACITY;
    ads8688_history_count -= copy_count;

    return copy_count;
}

/**
 * @brief 清空 ADS8688 历史采样记录。
 * @param 无。
 * @return 无。
 * @note 保留各通道最新采样结果，并将历史覆盖计数清零。
 */
void ads8688_storage_clear(void)
{
    ads8688_history_head = 0u;
    ads8688_history_count = 0u;
    ads8688_history_overwrite_count = 0u;
}

/**
 * @brief 获取历史缓冲区累计覆盖记录数量。
 * @param 无。
 * @return 自上次初始化或清空历史记录后的累计覆盖次数。
 * @note 无副作用。
 */
uint32_t ads8688_storage_get_overwrite_count(void)
{
    return ads8688_history_overwrite_count;
}
