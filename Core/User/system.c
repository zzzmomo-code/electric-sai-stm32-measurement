/**
 * @file system.c
 * @brief G 题无 FPGA 片上 ADC 数据链路的统一初始化和主循环入口。
 *
 * 模块用途：连接“PA6 单路 ADC1 -> FFT/谐波约束/IQ 拟合 -> 350 点显示快照
 *          -> USART1 串口屏预装”。
 * GPIO 引脚映射：PA6/ADC1_INP3、PA9/USART1_TX、PA10/USART1_RX。
 * 依赖的外设和 CubeIDE 配置：ADC1 12 位，TIM2 TRGO 3.2 MHz，
 *          DMA1 Stream0 Normal Halfword；USART1 512000 baud 8N1 DMA。
 * 初始化方法：main.c 的 USER CODE BEGIN 2 区域只调用 system_init()。
 * 调用方法：main.c 的 while(1) 用户区只调用 system_process()。
 */

#include "system.h"

/** 串口屏三图链路自检开关；正常模式必须保持为零。 */
#define HMI_CHART_SELF_TEST_ENABLE 0u

/** 已经转换为显示快照的最近片上 ADC 帧序号。 */
static uint32_t system_last_converted_sequence;

/** 是否已经转换过至少一帧，避免首帧序号为零时被跳过。 */
static uint8_t system_conversion_started;

/**
 * @brief 初始化片上 ADC 测量、显示换算和串口屏三个用户模块。
 * @param 无。
 * @return 无。
 *
 * @note CubeMX 生成的 ADC1、TIM2、DMA、USART1 和 GPIO 必须已在 main.c 中完成初始化。
 *       本函数只绑定 HAL 句柄并初始化用户状态，不重复配置硬件寄存器。
 */
void system_init(void)
{
    system_last_converted_sequence = 0u;
    system_conversion_started = 0u;

    measurement_conversion_init();
    hmi_task2_init();

#if (HMI_CHART_SELF_TEST_ENABLE != 0u)
    hmi_task2_set_chart_self_test(1u);
#endif

#if defined(SYSTEM_USART1_AVAILABLE)
    hmi_task2_bind_uart(&huart1);
#endif

    (void)onchip_measurement_init();
}

/**
 * @brief 推进“ADC1 采集分析 -> 显示换算 -> 串口屏预装”的数据链路。
 * @param 无。
 * @return 无。
 *
 * @note 测量快照只有序号变化时才换算一次；串口屏模块自行保存稳定工作快照，
 *       因此后台继续测量不会破坏正在进行的 DMA 预装。
 */
void system_process(void)
{
    const fpga_measurement_snapshot_t *measurement_snapshot;

    /* 第一步：推进 ADC1 DMA、FFT、谐波约束和联合 I/Q 拟合。 */
    onchip_measurement_process();

    /* 第二步：仅对新 frame_seq 执行一次 350 点显示换算。 */
    if (onchip_measurement_get_snapshot(
            &measurement_snapshot) != 0u)
    {
        uint32_t sequence =
            measurement_snapshot->header.frame_seq;

        if ((system_conversion_started == 0u)
            || (sequence != system_last_converted_sequence))
        {
            if (measurement_conversion_update(
                    measurement_snapshot) != 0u)
            {
                system_last_converted_sequence = sequence;
                system_conversion_started = 1u;
            }
        }
    }

    /* 第三步：后台预装三图和文本，收到按键后只切换控件可见性。 */
    hmi_task2_process();
}
