/**
 * @file hmi_chart.h
 * @brief TJC 串口屏幅频和相频曲线构帧接口。
 *
 * 模块用途：把 FPGA 的 Bode 数据降采样为 64 点，并构建 Waveform 控件
 *          所需的 cle + add 指令。
 * GPIO 引脚映射：无直接 GPIO；构建结果由 hmi_task2 通过 USART1 发送。
 * 依赖的外设和 CubeIDE 配置：串口屏 USART1 9600 8N1；HMI 当前页面包含
 *          s0（幅频 Waveform）和 s1（相频 Waveform），两者 ch=1。
 * 初始化方法：无状态，无需单独初始化。
 * 调用方法：hmi_task2 获取 FPGA 快照后调用 hmi_chart_build_bode_frame()。
 * 协议：cle s0.id,0 和 add s0.id,0,val，每条命令以 FF FF FF 结束。
 */

#ifndef HMI_CHART_H
#define HMI_CHART_H

#include <stdint.h>

#include "fpga_link.h"

/**
 * 串口屏每条曲线显示的点数。
 * 实板上 64 点约占 Waveform 横轴四分之一，因此按 256 像素宽度输出。
 */
#define HMI_CHART_POINT_COUNT 256u

/**
 * 单个 Waveform 的 cle + 256 条 add 指令缓冲区上限。
 * 最坏情况下每条三位数 add 指令占 18 字节，单图不超过 4621 字节。
 */
#define HMI_CHART_FRAME_SIZE_PER_COMPONENT 5120u

/** 幅频和相频两条曲线的总缓冲区上限。 */
#define HMI_CHART_FRAME_SIZE_TOTAL \
    (HMI_CHART_FRAME_SIZE_PER_COMPONENT * 2u)

/** TJC add 指令的最大纵轴值。 */
#define HMI_CHART_VALUE_MAX 255u

/** 构帧状态。 */
typedef enum
{
    HMI_CHART_STATUS_OK = 0,
    HMI_CHART_STATUS_INVALID_ARGUMENT,
    HMI_CHART_STATUS_BUFFER_TOO_SMALL
} hmi_chart_status_t;

/**
 * @brief 构建幅频 s0 和相频 s1 的完整 cle + add 指令帧。
 * @param bode FPGA 链路发布的最新完整数据。
 * @param frame 输出串口字节流。
 * @param frame_capacity 输出缓冲区容量。
 * @param frame_size 输出实际字节数。
 * @return 构帧状态。
 * @note 使用 s0.id/s1.id 表达式，避免控件置顶、置底或增删后数字 ID 改变。
 *       每个频率分组内分别对 FPGA 已开方的幅度和有符号相位求算术平均值；
 *       16 位幅值均值直接线性映射到 Waveform 的 0~255。
 */
hmi_chart_status_t hmi_chart_build_bode_frame(
    const fpga_link_bode_t *bode,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size);

#endif /* HMI_CHART_H */
