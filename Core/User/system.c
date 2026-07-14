/**
 * @file system.c
 * @brief 用户自定义模块统一初始化入口。
 *
 * 模块用途：集中调用用户模块初始化函数，避免在 main.c 中堆放业务逻辑。
 * GPIO 引脚映射：无直接 GPIO 引脚；片上 ADC 计划使用 PC4/ADC1_INP4 与 PB1/ADC2_INP5。
 * 依赖的外设和 CubeIDE 配置：当前等待 CubeMX 生成 ADC1/ADC2、TIM2、DMA 和 NVIC；
 * 串口屏继续依赖 USART1，9600 8N1，轮询发送且不使用 USART DMA。
 * 初始化方法：在 main.c 的 USER CODE BEGIN 2 区域调用 system_init()。
 * 调用方法：系统启动时调用一次，主循环持续调用 system_process()。
 */

#include "system.h"

/**
 * 串口屏硬件联调开关：1 为发布固定自检结果，0 为等待真实测量算法结果。
 * 完成 PA9 到串口屏 RX 的实屏验证后，改为 0 并重新烧录。
 */
#define HMI_TJC_SELF_TEST_ENABLE 0u

#if (HMI_TJC_SELF_TEST_ENABLE != 0u)
/**
 * @brief 发布用于验证串口屏通信的固定测量结果。
 * @param 无。
 * @return 无。
 * @note 仅用于 HMI 联调，不读取或修改双 ADC DMA 数据；关闭开关后该函数不会参与编译。
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
 * @note CubeMX 未生成片上 ADC 配置时，adc_dual_init() 安全返回“未配置”状态。
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
    adc_dual_init();
}

/**
 * @brief 执行双 ADC、FFT 和串口屏主循环处理。
 * @param 无。
 * @return 无。
 * @note 第二次 adc_dual_process() 只同步 TIM2 启停状态，不重复处理已领取的 DMA 标志。
 */
void system_process(void)
{
    adc_dual_process();
    measurement_fft_process();
    adc_dual_process();
    if (measurement_fft_hmi_refresh_allowed() != 0u)
    {
        hmi_tjc_process();
    }
}
