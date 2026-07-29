/**
 * @file system.c
 * @brief G 题 STM32 正式数据链路的统一初始化和主循环入口。
 *
 * 模块用途：连接“FPGA SPI3 完整测量帧 -> 700 点显示快照 -> USART1 串口屏预装”。
 * GPIO 引脚映射：PC10/SPI3_SCK、PC11/SPI3_MISO、PC12/SPI3_MOSI、
 *          PA15/FPGA_CS_N、PD1/FPGA_DATA_READY、PA9/USART1_TX、PA10/USART1_RX。
 * 依赖的外设和 CubeIDE 配置：SPI3 Master Mode 0 20 MHz 双向 DMA，PD1 EXTI1；
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

void system_init(void)
{
    system_last_converted_sequence = 0u;
    system_conversion_started = 0u;

    fpga_link_init();
    measurement_conversion_init();
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

void system_process(void)
{
    const fpga_measurement_snapshot_t *fpga_snapshot;

    fpga_link_process();

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

    hmi_task2_process();
}
