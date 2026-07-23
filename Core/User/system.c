/**
 * @file system.c
 * @brief 数字锁相工程的统一初始化、主循环调度和串口命令实现。
 *
 * 模块用途：集中初始化用户模块、调度 ADC/DAC 信号链路并处理串口命令。
 * GPIO 引脚：PC0=ADC 输入，PA4=DAC 输出，PB14/PB15=USART1。
 * 依赖外设：CubeMX 已初始化 ADC1、DAC1、TIM2、DMA1、USART1。
 * 初始化方法：main.c 用户初始化区只调用 system_init()。
 * 调用方法：main.c while(1) 用户区只调用 system_process()。
 */

#include "system.h"

/**
 * @brief 将运行模式转换为串口显示文本。
 * @param mode 当前运行模式。
 * @return 指向静态只读字符串的指针。
 */
static const char *system_mode_name(signal_mode_t mode)
{
    if (mode == SIGNAL_MODE_DPLL)
    {
        return "dpll";
    }
    return "direct";
}

/**
 * @brief 将 DPLL 状态转换为串口显示文本。
 * @param state 当前锁定状态。
 * @return 指向静态只读字符串的指针。
 */
static const char *system_lock_name(dpll_lock_state_t state)
{
    if (state == DPLL_STATE_LOCKED)
    {
        return "locked";
    }
    if (state == DPLL_STATE_TRACKING)
    {
        return "tracking";
    }
    if (state == DPLL_STATE_ACQUIRING)
    {
        return "acquiring";
    }
    return "no_signal";
}

/**
 * @brief 生成并非阻塞发送一次当前状态。
 * @param 无。
 * @return 无；串口忙时本次状态会被放弃。
 */
static void system_report_status(void)
{
    static char status_text[256];
    signal_chain_status_t status;
    int32_t frequency_millihz;
    int32_t phase_millideg;
    int32_t target_millideg;
    int32_t amplitude_counts;
    int32_t offset_counts;

    signal_chain_get_status(&status);
    frequency_millihz = (int32_t)(status.frequency_hz * 1000.0f);
    phase_millideg = (int32_t)(status.phase_error_deg * 1000.0f);
    target_millideg = (int32_t)(status.target_phase_deg * 1000.0f);
    amplitude_counts = (int32_t)(status.amplitude_adc_counts + 0.5f);
    offset_counts = (int32_t)(status.offset_adc_counts + 0.5f);

    (void)snprintf(
        status_text,
        sizeof(status_text),
        "mode=%s lock=%s run=%u f_mHz=%ld phase_mdeg=%ld target_mdeg=%ld "
        "amp=%ld offset=%ld blocks=%lu err=%lu/%lu/%lu\r\n",
        system_mode_name(status.mode),
        system_lock_name(status.lock_state),
        (unsigned int)status.running,
        (long)frequency_millihz,
        (long)phase_millideg,
        (long)target_millideg,
        (long)amplitude_counts,
        (long)offset_counts,
        (unsigned long)status.processed_half_blocks,
        (unsigned long)status.adc_error_count,
        (unsigned long)status.dac_error_count,
        (unsigned long)status.processing_overrun_count);
    (void)uart_debug_write(status_text);
}

/**
 * @brief 执行一个串口单字节命令。
 * @param command 收到的 ASCII 命令。
 * @return 无；可能切换模式、修改目标相位或发送状态。
 */
static void system_process_command(uint8_t command)
{
    float target_phase;

    switch (command)
    {
        case '0':
            signal_chain_set_mode(SIGNAL_MODE_DIRECT);
            (void)uart_debug_write("OK mode=direct\r\n");
            break;

        case '1':
            signal_chain_set_mode(SIGNAL_MODE_DPLL);
            (void)uart_debug_write("OK mode=dpll\r\n");
            break;

        case 'r':
        case 'R':
            signal_chain_reset_lock();
            (void)uart_debug_write("OK dpll_reset\r\n");
            break;

        case '+':
            target_phase = signal_chain_get_target_phase_deg() + 5.0f;
            signal_chain_set_target_phase_deg(target_phase);
            (void)uart_debug_write("OK phase_plus_5deg\r\n");
            break;

        case '-':
            target_phase = signal_chain_get_target_phase_deg() - 5.0f;
            signal_chain_set_target_phase_deg(target_phase);
            (void)uart_debug_write("OK phase_minus_5deg\r\n");
            break;

        case '?':
        case 's':
        case 'S':
            system_report_status();
            break;

        default:
            (void)uart_debug_write(
                "ERR 0=direct 1=dpll r=reset +=phase5 -=phase5 s=status\r\n");
            break;
    }
}

/**
 * @brief 初始化串口软件层和 ADC/DAC 数字信号链路。
 * @param 无。
 * @return 无。
 * @note CubeMX 外设初始化已完成；本函数会启动 TIM2、ADC DMA 和 DAC DMA。
 */
void system_init(void)
{
    uart_debug_init();
    signal_chain_init();
    (void)uart_debug_write(
        "\r\nphase_locking_codex ready\r\n"
        "0=direct 1=dpll r=reset +=phase5 -=phase5 s=status\r\n");
}

/**
 * @brief 执行一次非阻塞主循环调度。
 * @param 无。
 * @return 无。
 * @note 始终优先处理 ADC 半缓冲，状态串口只在无待处理采样块时运行。
 */
void system_process(void)
{
    uint8_t command;

    signal_chain_process();
    uart_debug_process();

    if (uart_debug_read_command(&command) != 0u)
    {
        system_process_command(command);
    }
}
