/**
 * @file fpga_protocol.c
 * @brief FPGA—STM32 SPI V1.0 协议解析实现。
 *
 * 模块用途：对状态和完整测量帧执行显式小端解析、字段范围检查和
 *          CRC-16/CCITT-FALSE 校验。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：无需初始化。
 * 调用方法：仅由主循环中的 fpga_link 调用。
 */

#include "system.h"

#define FPGA_PROTOCOL_STATUS_CRC_OFFSET 14u
#define FPGA_PROTOCOL_FRAME_CRC_BYTES   2u

/*
 * 协议规定多字节字段均为小端。这里逐字节拼接，而不把接收缓冲区直接强制转换成
 * C 结构体，原因有三点：
 * 1. DMA 缓冲区不保证满足 uint32_t/uint64_t 对齐要求；
 * 2. 编译器可能在结构体成员之间插入填充字节；
 * 3. 显式偏移能让代码和 FPGA 协议表逐项核对。
 */

/**
 * @brief 从字节流读取一个小端无符号 16 位整数。
 * @param data 字段首字节地址，调用者保证至少有 2 字节。
 * @return 还原后的 uint16_t。
 */
uint16_t fpga_protocol_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0]
                      | ((uint16_t)data[1] << 8));
}

/**
 * @brief 从字节流读取一个小端有符号 16 位整数。
 * @param data 字段首字节地址，调用者保证至少有 2 字节。
 * @return 保留二进制补码含义的 int16_t。
 */
int16_t fpga_protocol_read_i16_le(const uint8_t *data)
{
    return (int16_t)fpga_protocol_read_u16_le(data);
}

/**
 * @brief 从字节流读取一个小端无符号 32 位整数。
 * @param data 字段首字节地址，调用者保证至少有 4 字节。
 * @return 还原后的 uint32_t。
 */
uint32_t fpga_protocol_read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
           | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16)
           | ((uint32_t)data[3] << 24);
}

/**
 * @brief 从字节流读取一个小端有符号 32 位整数。
 * @param data 字段首字节地址，调用者保证至少有 4 字节。
 * @return 保留二进制补码含义的 int32_t。
 */
int32_t fpga_protocol_read_i32_le(const uint8_t *data)
{
    return (int32_t)fpga_protocol_read_u32_le(data);
}

/**
 * @brief 从字节流读取一个小端无符号 64 位整数。
 * @param data 字段首字节地址，调用者保证至少有 8 字节。
 * @return 还原后的 uint64_t。
 */
uint64_t fpga_protocol_read_u64_le(const uint8_t *data)
{
    return (uint64_t)fpga_protocol_read_u32_le(data)
           | ((uint64_t)fpga_protocol_read_u32_le(data + 4u) << 32);
}

/**
 * @brief 把 32 位整数按小端顺序写入字节流。
 * @param data 至少可写 4 字节的目标地址。
 * @param value 待写入值。
 * @return 无；data 为空时直接返回。
 */
void fpga_protocol_write_u32_le(uint8_t *data, uint32_t value)
{
    if (data == NULL)
    {
        return;
    }

    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

uint16_t fpga_protocol_crc16(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xffffu;
    uint32_t index;

    if ((data == NULL) && (length != 0u))
    {
        return crc;
    }

    for (index = 0u; index < length; index++)
    {
        uint8_t bit;

        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 校验并解析 GET_STATUS 返回的 16 字节状态。
 * @param data 从第一个响应字节 0x5A 开始的缓冲区。
 * @param length 缓冲区长度，V1 必须等于 16。
 * @param status 成功时写入稳定的状态、序号和完整帧长度。
 * @return FPGA_PROTOCOL_OK 或具体的格式、版本、长度、CRC 错误。
 * @note 输出采用“全部校验成功后再赋值”，失败不会留下半更新状态。
 */
fpga_protocol_result_t fpga_protocol_parse_status(
    const uint8_t *data,
    uint32_t length,
    fpga_protocol_status_t *status)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    fpga_protocol_status_t parsed;

    if ((data == NULL) || (status == NULL))
    {
        return FPGA_PROTOCOL_ERROR_NULL;
    }
    if (length != FPGA_PROTOCOL_STATUS_BYTES)
    {
        return FPGA_PROTOCOL_ERROR_SIZE;
    }
    if ((data[0] != 0x5au) || (data[1] != 0xa5u))
    {
        return FPGA_PROTOCOL_ERROR_MAGIC;
    }
    if (data[2] != FPGA_PROTOCOL_VERSION)
    {
        return FPGA_PROTOCOL_ERROR_VERSION;
    }
    if ((data[3] & 0xe0u) != 0u)
    {
        return FPGA_PROTOCOL_ERROR_STATE;
    }

    received_crc = fpga_protocol_read_u16_le(
        &data[FPGA_PROTOCOL_STATUS_CRC_OFFSET]);
    calculated_crc = fpga_protocol_crc16(
        data, FPGA_PROTOCOL_STATUS_CRC_OFFSET);
    if (received_crc != calculated_crc)
    {
        return FPGA_PROTOCOL_ERROR_CRC;
    }

    parsed.version = data[2];
    parsed.state = data[3];
    parsed.frame_seq = fpga_protocol_read_u32_le(&data[4]);
    parsed.frame_length = fpga_protocol_read_u32_le(&data[8]);
    parsed.reserved = fpga_protocol_read_u16_le(&data[12]);

    if ((parsed.frame_length
         < (FPGA_PROTOCOL_HEADER_BYTES + FPGA_PROTOCOL_FRAME_CRC_BYTES))
        || (parsed.frame_length > FPGA_PROTOCOL_MAX_FRAME_BYTES))
    {
        return FPGA_PROTOCOL_ERROR_LENGTH;
    }

    *status = parsed;
    return FPGA_PROTOCOL_OK;
}

/**
 * @brief 解析一个 12 字节频率分量描述。
 * @param data 分量首字节。
 * @param component 输出结构。
 * @return 无。
 */
static void fpga_protocol_parse_component(
    const uint8_t *data,
    fpga_protocol_component_t *component)
{
    component->frequency_mhz = fpga_protocol_read_u32_le(&data[0]);
    component->amplitude_peak_uv =
        fpga_protocol_read_u32_le(&data[4]);
    component->fft_bin = fpga_protocol_read_u16_le(&data[8]);
    component->harmonic_order = data[10];
    component->flags = data[11];
}

/**
 * @brief 校验完整测量帧并解析 128 字节帧头。
 * @param frame 以 ASCII G26F 开头、以 CRC16 结尾的完整帧。
 * @param length GET_STATUS 给出的完整帧字节数。
 * @param expected_sequence GET_STATUS 给出的帧序号，用于防止读到被替换的帧。
 * @param header 成功时输出帧头；时域和频谱载荷由 fpga_link 在校验后复制。
 * @return FPGA_PROTOCOL_OK 或具体错误原因。
 * @note CRC 覆盖从 G26F 到最后一个频谱字节，不包含末尾 CRC 本身。
 */
fpga_protocol_result_t fpga_protocol_parse_frame(
    const uint8_t *frame,
    uint32_t length,
    uint32_t expected_sequence,
    fpga_protocol_frame_header_t *header)
{
    fpga_protocol_frame_header_t parsed;
    uint32_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t component_index;

    if ((frame == NULL) || (header == NULL))
    {
        return FPGA_PROTOCOL_ERROR_NULL;
    }
    if ((length < (FPGA_PROTOCOL_HEADER_BYTES + FPGA_PROTOCOL_FRAME_CRC_BYTES))
        || (length > FPGA_PROTOCOL_MAX_FRAME_BYTES))
    {
        return FPGA_PROTOCOL_ERROR_SIZE;
    }
    if ((frame[0] != (uint8_t)'G')
        || (frame[1] != (uint8_t)'2')
        || (frame[2] != (uint8_t)'6')
        || (frame[3] != (uint8_t)'F'))
    {
        return FPGA_PROTOCOL_ERROR_MAGIC;
    }

    /*
     * 下面的数字是 V1 帧头中的固定字节偏移。先把 128 字节帧头完整解析到
     * 局部变量 parsed，再统一检查固定值和范围；任何检查失败都不会污染
     * 调用者正在使用的上一份 header。
     */
    parsed.protocol_version = frame[4];
    parsed.frame_type = frame[5];
    parsed.header_bytes = fpga_protocol_read_u16_le(&frame[6]);
    parsed.total_bytes = fpga_protocol_read_u32_le(&frame[8]);
    parsed.frame_seq = fpga_protocol_read_u32_le(&frame[12]);
    parsed.timestamp_50m = fpga_protocol_read_u64_le(&frame[16]);
    parsed.flags = fpga_protocol_read_u32_le(&frame[24]);
    parsed.time_sample_rate_hz = fpga_protocol_read_u32_le(&frame[28]);
    parsed.time_count = fpga_protocol_read_u16_le(&frame[32]);
    parsed.captured_cycles = frame[34];
    parsed.time_format = frame[35];
    parsed.fft_sample_rate_hz = fpga_protocol_read_u32_le(&frame[36]);
    parsed.fft_length = fpga_protocol_read_u16_le(&frame[40]);
    parsed.spectrum_count = fpga_protocol_read_u16_le(&frame[42]);
    parsed.bin_spacing_mhz = fpga_protocol_read_u32_le(&frame[44]);
    parsed.spectrum_format = frame[48];
    parsed.window_type = frame[49];
    parsed.component_count = frame[50];
    parsed.reserved0 = frame[51];
    parsed.vpp_uv = fpga_protocol_read_u32_le(&frame[52]);
    parsed.vrms_uv = fpga_protocol_read_u32_le(&frame[56]);
    parsed.fundamental_mhz = fpga_protocol_read_u32_le(&frame[60]);
    parsed.dc_offset_uv = fpga_protocol_read_i32_le(&frame[64]);

    for (component_index = 0u;
         component_index < FPGA_PROTOCOL_COMPONENT_MAX;
         component_index++)
    {
        fpga_protocol_parse_component(
            &frame[68u + ((uint32_t)component_index * 12u)],
            &parsed.component[component_index]);
    }

    parsed.calibration_revision = fpga_protocol_read_u16_le(&frame[104]);
    parsed.reserved1 = fpga_protocol_read_u16_le(&frame[106]);
    parsed.dropped_frames = fpga_protocol_read_u32_le(&frame[108]);
    parsed.adc_min_code = fpga_protocol_read_i16_le(&frame[112]);
    parsed.adc_max_code = fpga_protocol_read_i16_le(&frame[114]);

    if (parsed.protocol_version != FPGA_PROTOCOL_VERSION)
    {
        return FPGA_PROTOCOL_ERROR_VERSION;
    }
    if (parsed.frame_type != FPGA_PROTOCOL_FRAME_TYPE_FULL)
    {
        return FPGA_PROTOCOL_ERROR_FRAME_TYPE;
    }
    if ((parsed.header_bytes != FPGA_PROTOCOL_HEADER_BYTES)
        || (parsed.total_bytes != length))
    {
        return FPGA_PROTOCOL_ERROR_LENGTH;
    }
    if (parsed.frame_seq != expected_sequence)
    {
        return FPGA_PROTOCOL_ERROR_SEQUENCE;
    }
    if ((parsed.time_sample_rate_hz
         != FPGA_PROTOCOL_TIME_SAMPLE_RATE_HZ)
        || (parsed.time_count == 0u)
        || (parsed.time_count > FPGA_PROTOCOL_MAX_TIME_SAMPLES)
        || (parsed.captured_cycles != 3u)
        || (parsed.time_format != 1u)
        || (parsed.fft_sample_rate_hz
            != FPGA_PROTOCOL_FFT_SAMPLE_RATE_HZ)
        || (parsed.fft_length != FPGA_PROTOCOL_FFT_LENGTH)
        || (parsed.spectrum_count != FPGA_PROTOCOL_SPECTRUM_COUNT)
        || (parsed.bin_spacing_mhz
            != FPGA_PROTOCOL_BIN_SPACING_MHZ)
        || (parsed.spectrum_format != 1u)
        || (parsed.window_type != 1u)
        || (parsed.component_count == 0u)
        || (parsed.component_count > FPGA_PROTOCOL_COMPONENT_MAX))
    {
        return FPGA_PROTOCOL_ERROR_FIELD;
    }

    expected_length = FPGA_PROTOCOL_HEADER_BYTES
                      + ((uint32_t)parsed.time_count * 2u)
                      + ((uint32_t)parsed.spectrum_count * 2u)
                      + FPGA_PROTOCOL_FRAME_CRC_BYTES;
    if (expected_length != length)
    {
        return FPGA_PROTOCOL_ERROR_LENGTH;
    }

    received_crc = fpga_protocol_read_u16_le(&frame[length - 2u]);
    calculated_crc = fpga_protocol_crc16(frame, length - 2u);
    if (received_crc != calculated_crc)
    {
        return FPGA_PROTOCOL_ERROR_CRC;
    }

    *header = parsed;
    return FPGA_PROTOCOL_OK;
}
