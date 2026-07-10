/**
 * @file system.c
 * @brief 用户自定义模块统一初始化入口。
 *
 * 模块用途：集中调用用户模块初始化函数，避免在 main.c 中堆放业务逻辑。
 * GPIO 引脚映射：无直接 GPIO 引脚；ADS8688 引脚映射见 ads8688.c 模块说明。
 * 依赖的外设和 CubeIDE 配置：依赖 CubeMX 已完成 GPIO、DMA、SPI2 和 NVIC 初始化；启用串口屏时还依赖 USART1。
 * 初始化方法：在 main.c 的 USER CODE BEGIN 2 区域调用 system_init()。
 * 调用方法：系统启动时调用一次，主循环继续调用各功能处理函数。
 */

#include "system.h"

/**
 * @brief 初始化全部用户功能模块。
 * @param 无。
 * @return 无。
 * @note 当前启动 ADS8688 采集；返回值被显式忽略，失败状态可通过 ADS8688 诊断接口查询。
 */
void system_init(void)
{
    measurement_result_init();
    hmi_tjc_init();
#if defined(SYSTEM_USART1_AVAILABLE)
    hmi_tjc_bind_uart(&huart1);
#endif
    (void)ads8688_init();
}
