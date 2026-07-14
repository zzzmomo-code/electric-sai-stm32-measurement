/**
 * @file measurement_result.c
 * @brief 测量结果快照存储实现。
 *
 * 模块用途：保存算法在主循环中发布的最新测量结果，供显示模块低频读取。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖，不允许中断上下文访问。
 * 初始化方法：系统启动时调用 measurement_result_init()。
 * 调用方法：算法发布结果，HMI 或其他主循环模块读取结果快照。
 */

#include "system.h"

/** 最新测量结果快照，仅在主循环上下文读写。 */
static measurement_result_t measurement_result_latest;

/** 是否已经收到过算法模块发布的结果。 */
static uint8_t measurement_result_available;

/**
 * @brief 初始化测量结果快照。
 * @param 无。
 * @return 无。
 * @note 清除所有字段并将结果状态设为无效。
 */
void measurement_result_init(void)
{
    measurement_result_latest.dc_voltage = 0.0f;
    measurement_result_latest.amplitude_vpp = 0.0f;
    measurement_result_latest.rms_voltage = 0.0f;
    measurement_result_latest.frequency_hz = 0.0f;
    measurement_result_latest.thd_percent = 0.0f;
    measurement_result_latest.phase_deg = 0.0f;
    measurement_result_latest.wave_type = MEASUREMENT_WAVE_UNKNOWN;
    measurement_result_latest.mode = MEASUREMENT_MODE_UNKNOWN;
    measurement_result_latest.valid_mask = 0u;
    measurement_result_latest.valid = 0u;
    measurement_result_latest.sequence = 0u;
    measurement_result_available = 0u;
}

/**
 * @brief 发布一组新的测量结果。
 * @param result 待发布结果的指针，空指针会被忽略。
 * @return 无。
 * @note 采用结构体整体复制；调用者和显示模块必须都在主循环上下文运行。
 */
void measurement_result_publish(const measurement_result_t *result)
{
    if (result == 0)
    {
        return;
    }

    measurement_result_latest = *result;
    measurement_result_available = 1u;
}

/**
 * @brief 读取当前测量结果快照。
 * @param result 用于接收快照的指针。
 * @return 已存在已发布快照时返回 1；空指针或尚未发布时返回 0。
 * @note 不会清除或修改算法模块已发布的结果。
 */
uint8_t measurement_result_get_snapshot(measurement_result_t *result)
{
    if (result == 0)
    {
        return 0u;
    }

    *result = measurement_result_latest;
    return measurement_result_available;
}
