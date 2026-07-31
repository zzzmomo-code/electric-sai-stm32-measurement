/**
 * @file system.c
 * @brief G 题 STM32 正式数据链路的统一初始化和主循环入口。
 *
 * 模块用途：连接“FPGA SPI3 连续测量帧 -> 350 点显示快照 -> USART1 串口屏稳定锁存显示”。
 * GPIO 引脚映射：PC10/SPI3_SCK、PC11/SPI3_MISO、PC12/SPI3_MOSI、
 *          PA15/FPGA_CS_N、PD1/FPGA_DATA_READY、PA9/USART1_TX、PA10/USART1_RX。
 * 依赖的外设和 CubeIDE 配置：SPI3 Master Mode 0 625 kHz 双向 DMA，PD1 EXTI1；
 *          USART1 512000 baud 8N1，TX/RX DMA 与全局中断。
 * 初始化方法：main.c 的 USER CODE BEGIN 2 区域只调用 system_init()。
 * 调用方法：main.c 的 while(1) 用户区只调用 system_process()。
 */

#include "system.h"

/** 串口屏三图链路自检开关；正常模式必须保持为零。 */
#define HMI_CHART_SELF_TEST_ENABLE 0u

/** 已经转换为显示快照的最近 FPGA 帧序号。 */
static uint32_t system_last_converted_sequence;

/** 是否已经转换过至少一帧，避免 FPGA 首帧序号为零时被跳过。 */
static uint8_t system_conversion_started;

/**
 * @brief 初始化 FPGA 链路、显示换算和串口屏三个用户模块。
 * @param 无。
 * @return 无。
 *
 * @note CubeMX 生成的 SPI3、USART1、DMA 和 GPIO 必须已在 main.c 中完成初始化。
 *       本函数只绑定 HAL 句柄并初始化用户状态，不重复配置硬件寄存器。
 */
void system_init(void)
{
    system_last_converted_sequence = 0u;
    system_conversion_started = 0u;

    fpga_link_init();
    measurement_conversion_init();
    measurement_calibration_init();
    hmi_task2_init();

#if (HMI_CHART_SELF_TEST_ENABLE != 0u)
    hmi_task2_set_chart_self_test(1u);
#endif

#if defined(SYSTEM_SPI3_AVAILABLE)
    fpga_link_bind_spi(&hspi3);
#endif

#if defined(SYSTEM_USART1_AVAILABLE)
    hmi_task2_bind_uart(&huart1);
#endif
}

/**
 * @brief 推进“FPGA 接收 -> 显示换算 -> 串口屏稳定锁存显示”的非阻塞数据链路。
 * @param 无。
 * @return 无。
 *
 * @note FPGA 快照只有序号变化时才换算一次；串口屏模块自行保存稳定工作快照，
 *       因此 FPGA 继续更新不会破坏正在进行的 UART DMA 发送。
 */
void system_process(void)
{
    const fpga_measurement_snapshot_t *fpga_snapshot;

    /* 第一步：推进 SPI3 命令、DMA、CRC、ACK 状态机并发布最新有效快照。 */
    fpga_link_process();

    /* 第二步：仅对新 frame_seq 执行一次 350 点显示换算。 */
    if (fpga_link_get_snapshot(&fpga_snapshot) != 0u)
    {
        uint32_t sequence = fpga_snapshot->header.frame_seq;

        if ((system_conversion_started == 0u)
            || (sequence != system_last_converted_sequence))
        {
            if (measurement_conversion_update(fpga_snapshot) != 0u)
            {
                system_last_converted_sequence = sequence;
                system_conversion_started = 1u;
            }
        }
    }

    /*
     * 第三步：首帧立即显示，后续连续五帧稳定并平均文字后刷新三条曲线。
     * 周期键即时切换重叠控件前景，启动键重发文字和三条曲线；均不会触发 FPGA 重新计算。
     */
    hmi_task2_process();
}
