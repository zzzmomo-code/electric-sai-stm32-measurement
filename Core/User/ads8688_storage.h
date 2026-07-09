/**
 * @file ads8688_storage.h
 * @brief ADS8688 采样数据存储模块公共接口。
 *
 * 模块用途：保存各通道最新采样值，并用环形缓冲区保存历史采样记录。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖，不需要额外 CubeIDE 配置。
 * 初始化方法：系统初始化阶段调用 ads8688_storage_init()。
 * 调用方法：采样完成后调用 ads8688_storage_push()，按需读取最新值或历史记录。
 */

#ifndef ADS8688_STORAGE_H
#define ADS8688_STORAGE_H

#include "ads8688.h"

/** 历史采样环形缓冲区可保存的记录数量。 */
#define ADS8688_HISTORY_CAPACITY 4096u

/** ADS8688 可用模拟输入通道数量。 */
#define ADS8688_CHANNEL_COUNT 8u

/**
 * @brief 初始化 ADS8688 采样存储模块。
 * @param 无。
 * @return 无。
 * @note 清空历史记录、各通道最新值和历史覆盖计数。
 */
void ads8688_storage_init(void);

/**
 * @brief 保存一次 ADS8688 采样结果。
 * @param channel 采样通道号，有效范围为 0 至 7。
 * @param raw_code ADC 直二进制原始码。
 * @param range 当前通道输入量程。
 * @param sample_index 本次采样序号。
 * @return 无。
 * @note 有效通道会更新最新值并追加历史记录；无效通道不产生副作用。
 */
void ads8688_storage_push(uint8_t channel,
                          uint16_t raw_code,
                          ads8688_range_t range,
                          uint32_t sample_index);

/**
 * @brief 读取指定通道的最新采样结果。
 * @param channel 待读取的通道号，有效范围为 0 至 7。
 * @param latest 用于接收最新采样结果的指针。
 * @return 参数有效时返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 成功时会覆盖 latest 指向的结构体内容。
 */
ads8688_status_t ads8688_storage_get_latest(uint8_t channel,
                                             ads8688_latest_t *latest);

/**
 * @brief 按从旧到新的顺序读取历史采样记录。
 * @param samples 用于接收历史记录的数组。
 * @param max_count 数组最多可接收的记录数量。
 * @return 实际复制并移除的历史记录数量。
 * @note 成功读取的记录会从环形缓冲区移除。
 */
uint32_t ads8688_storage_read(ads8688_sample_t *samples,
                              uint32_t max_count);

/**
 * @brief 清空 ADS8688 历史采样记录。
 * @param 无。
 * @return 无。
 * @note 保留各通道最新采样值，同时将历史覆盖计数清零。
 */
void ads8688_storage_clear(void);

/**
 * @brief 获取历史缓冲区累计覆盖记录数量。
 * @param 无。
 * @return 自上次初始化或清空历史记录后累计覆盖的记录数量。
 * @note 无副作用。
 */
uint32_t ads8688_storage_get_overwrite_count(void);

#endif /* ADS8688_STORAGE_H */
