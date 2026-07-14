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
 * 串口屏硬件联调开关：1 为发布固定自检结果，0 为等待真实测量算法结果。
 * 完成 PA9 到串口屏 RX 的实屏验证后，改为 0 并重新烧录。
 */
#define HMI_TJC_SELF_TEST_ENABLE 0u

/* AIN0 硬件联调开关：1 表示 DMA 仅采集 AIN0；0 恢复默认 AIN0/AIN1 双通道扫描。 */
#define ADS8688_AIN0_TEST_ENABLE 0u

#if (HMI_TJC_SELF_TEST_ENABLE != 0u)
/**
 * @brief 发布用于验证串口屏通信的固定测量结果。
 * @param 无。
 * @return 无。
 * @note 仅用于 HMI 联调，不读取或修改 ADS8688 DMA 数据；关闭开关后该函数不会参与编译。
 */
static void hmi_tjc_publish_self_test(void)
{
    measurement_result_t result;

    result.dc_voltage = 0.0f;
    result.amplitude_vpp = 3.300f;
    result.rms_voltage = 1.1667f;
    result.frequency_hz = 12345.0f;
    result.thd_percent = 0.10f;
    result.phase_deg = -90.0f;
    result.wave_type = MEASUREMENT_WAVE_SINE;
    result.mode = MEASUREMENT_MODE_AC;
    result.valid_mask = MEASUREMENT_VALID_DC_VOLTAGE
                        | MEASUREMENT_VALID_AMPLITUDE
                        | MEASUREMENT_VALID_RMS
                        | MEASUREMENT_VALID_FREQUENCY
                        | MEASUREMENT_VALID_THD
                        | MEASUREMENT_VALID_WAVE_TYPE
                        | MEASUREMENT_VALID_PHASE;
    result.valid = 1u;
    result.sequence = 1u;
    measurement_result_publish(&result);
}
#endif

/**
 * @brief 初始化全部用户功能模块。
 * @param 无。
 * @return 无。
 * @note 当前启动 ADS8688 采集；返回值被显式忽略，失败状态可通过 ADS8688 诊断接口查询。
 */
void system_init(void)
{
    measurement_result_init();
    measurement_fft_init();
    hmi_tjc_init();
#if defined(SYSTEM_USART1_AVAILABLE)
    hmi_tjc_bind_uart(&huart1);
#endif
#if (HMI_TJC_SELF_TEST_ENABLE != 0u)
    hmi_tjc_publish_self_test();
#endif
    (void)ads8688_init();
#if (ADS8688_AIN0_TEST_ENABLE != 0u)
    (void)ads8688_set_manual_mode(0u);
#endif
}
