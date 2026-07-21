/**
 * @file system.c
 * @brief 用户自定义模块统一初始化入口。
 *
 * 模块用途：集中调用用户模块初始化函数，避免在 main.c 中堆放业务逻辑。
 * GPIO 引脚映射：PA6/ADC1_INP3、PB1/ADC2_INP5、PA0/TIM5_CH1，
 * PB12/AD9834_FSYNC、PB13/SPI2_SCK、PB15/SPI2_MOSI、
 * PB14/AD9834_FSELECT、PD8/AD9834_PSELECT。
 * 依赖的外设和 CubeIDE 配置：依赖 ADC1/ADC2、TIM2、TIM3、TIM5、SPI2、DMA 和 NVIC；
 * 串口屏依赖 USART1，9600 8N1；接收使用全局中断，发送轮询且不使用 DMA。
 * 初始化方法：在 main.c 的 USER CODE BEGIN 2 区域调用 system_init()。
 * 调用方法：系统启动时调用一次，主循环持续调用 system_process()。
 */

/*
 * DAC 输出补充映射：PC4/OPAMP1_VOUT，DAC1_OUT1 通过片内连接进入 OPAMP1。
 * 依赖 DAC1 Channel 1 内部输出和 OPAMP1 Follower-DAC_OUT1-INP 配置。
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
    result.secondary_dc_voltage = 0.0f;
    result.secondary_amplitude_vpp = 0.0f;
    result.secondary_rms_voltage = 0.0f;
    result.secondary_frequency_hz = 0.0f;
    result.secondary_thd_percent = 0.0f;
    result.secondary_wave_type = MEASUREMENT_WAVE_UNKNOWN;
    result.secondary_mode = MEASUREMENT_MODE_UNKNOWN;
    result.secondary_valid_mask = 0u;
    result.estimated_mask = 0u;
    result.secondary_estimated_mask = 0u;
    result.fault_mask = 0u;
    result.valid = 1u;
    result.sequence = 1u;
    measurement_result_publish(&result);
}
#endif

/**
 * @brief 初始化全部用户功能模块。
 * @param 无。
 * @return 无。
 * @note 先初始化 FFT 的 DWT 诊断，再启动复用 DWT 的外部频率测量模块。
 */
void system_init(void)
{
    /* 先启动 VGA 控制电压输出，并以 0 档作为安全默认值。 */
    (void)dac_output_init();
    measurement_result_init();
    measurement_fft_init();
    frequency_measure_init();
    dds_control_init();
    hmi_task2_init();
#if defined(SYSTEM_USART1_AVAILABLE)
    hmi_task2_bind_uart(&huart1);
#endif
#if (HMI_TJC_SELF_TEST_ENABLE != 0u)
    hmi_tjc_publish_self_test();
#endif
    adc_dual_init();
}

/**
 * @brief 执行外部频率、双 ADC、FFT 和串口屏主循环处理。
 * @param 无。
 * @return 无。
 * @note 第二次 adc_dual_process() 只同步 TIM2 启停状态，不重复处理已领取的 DMA 标志。
 */
void system_process(void)
{
    frequency_measure_process();
    dds_control_process();
    adc_dual_process();
    measurement_fft_process();
    adc_dual_process();
    hmi_task2_process();
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
