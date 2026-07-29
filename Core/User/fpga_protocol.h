/**
 * @file fpga_protocol.h
 * @brief FPGA—STM32 SPI V1.0 协议解析公共接口。
 *
 * 模块用途：定义 SPI 命令、状态响应、128 字节测量帧头，并提供显式小端
 *          读取、CRC16 和边界校验函数。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：无需初始化。
 * 调用方法：fpga_link 在 GET_STATUS、READ_FRAME 和 ACK_FRAME 时调用。
 */

#ifndef FPGA_PROTOCOL_H
#define FPGA_PROTOCOL_H

#include <stdint.h>

#define FPGA_PROTOCOL_COMMAND_PREFIX        0xa5u
#define FPGA_PROTOCOL_COMMAND_GET_STATUS    0x01u
#define FPGA_PROTOCOL_COMMAND_READ_FRAME    0x02u
#define FPGA_PROTOCOL_COMMAND_ACK_FRAME     0x03u

#define FPGA_PROTOCOL_VERSION               1u
#define FPGA_PROTOCOL_FRAME_TYPE_FULL       1u
#define FPGA_PROTOCOL_STATUS_BYTES          16u
#define FPGA_PROTOCOL_HEADER_BYTES          128u
#define FPGA_PROTOCOL_MAX_TIME_SAMPLES      3750u
#define FPGA_PROTOCOL_SPECTRUM_COUNT         1312u
#define FPGA_PROTOCOL_COMPONENT_MAX         3u
#define FPGA_PROTOCOL_MAX_FRAME_BYTES       10254u

#define FPGA_PROTOCOL_TIME_SAMPLE_RATE_HZ   12500000u
#define FPGA_PROTOCOL_FFT_SAMPLE_RATE_HZ    1562500u
#define FPGA_PROTOCOL_FFT_LENGTH            4096u
#define FPGA_PROTOCOL_BIN_SPACING_MHZ       381470u

#define FPGA_PROTOCOL_STATUS_FRAME_READY    (1u << 0)
#define FPGA_PROTOCOL_STATUS_FPGA_BUSY      (1u << 1)
#define FPGA_PROTOCOL_STATUS_ADC_OTR        (1u << 2)
#define FPGA_PROTOCOL_STATUS_FRAME_DROPPED  (1u << 3)
#define FPGA_PROTOCOL_STATUS_RESULT_INVALID (1u << 4)

#define FPGA_PROTOCOL_COMPONENT_VALID       (1u << 0)
#define FPGA_PROTOCOL_COMPONENT_IQ_READY    (1u << 1)

/** 协议解析结果。 */
typedef enum
{
    FPGA_PROTOCOL_OK = 0,
    FPGA_PROTOCOL_ERROR_NULL,
    FPGA_PROTOCOL_ERROR_SIZE,
    FPGA_PROTOCOL_ERROR_MAGIC,
    FPGA_PROTOCOL_ERROR_VERSION,
    FPGA_PROTOCOL_ERROR_STATE,
    FPGA_PROTOCOL_ERROR_CRC,
    FPGA_PROTOCOL_ERROR_LENGTH,
    FPGA_PROTOCOL_ERROR_FRAME_TYPE,
    FPGA_PROTOCOL_ERROR_SEQUENCE,
    FPGA_PROTOCOL_ERROR_FIELD
} fpga_protocol_result_t;

/** GET_STATUS 的已校验结果。 */
typedef struct
{
    uint8_t version;       /**< 协议版本，V1 固定为 1。 */
    uint8_t state;         /**< FRAME_READY、FPGA_BUSY 等状态位。 */
    uint32_t frame_seq;    /**< 当前稳定帧序号。 */
    uint32_t frame_length; /**< 完整帧长度，包含末尾 CRC16。 */
    uint16_t reserved;     /**< V1 保留字段。 */
} fpga_protocol_status_t;

/** 单个频率分量的精测结果。 */
typedef struct
{
    uint32_t frequency_mhz;     /**< 频率，单位 0.001 Hz。 */
    uint32_t amplitude_peak_uv; /**< 正弦峰值，单位 uV。 */
    uint16_t fft_bin;           /**< 对应 FFT 谱线。 */
    uint8_t harmonic_order;     /**< 1=基波，2=二次谐波。 */
    uint8_t flags;              /**< bit0=有效，bit1=IQ 精测完成。 */
} fpga_protocol_component_t;

/** 128 字节帧头的主机侧解析结果，不直接映射 DMA 缓冲区。 */
typedef struct
{
    uint8_t protocol_version;
    uint8_t frame_type;
    uint16_t header_bytes;
    uint32_t total_bytes;
    uint32_t frame_seq;
    uint64_t timestamp_50m;
    uint32_t flags;
    uint32_t time_sample_rate_hz;
    uint16_t time_count;
    uint8_t captured_cycles;
    uint8_t time_format;
    uint32_t fft_sample_rate_hz;
    uint16_t fft_length;
    uint16_t spectrum_count;
    uint32_t bin_spacing_mhz;
    uint8_t spectrum_format;
    uint8_t window_type;
    uint8_t component_count;
    uint8_t reserved0;
    uint32_t vpp_uv;
    uint32_t vrms_uv;
    uint32_t fundamental_mhz;
    int32_t dc_offset_uv;
    fpga_protocol_component_t component[FPGA_PROTOCOL_COMPONENT_MAX];
    uint16_t calibration_revision;
    uint16_t reserved1;
    uint32_t dropped_frames;
    int16_t adc_min_code;
    int16_t adc_max_code;
} fpga_protocol_frame_header_t;

uint16_t fpga_protocol_read_u16_le(const uint8_t *data);
int16_t fpga_protocol_read_i16_le(const uint8_t *data);
uint32_t fpga_protocol_read_u32_le(const uint8_t *data);
int32_t fpga_protocol_read_i32_le(const uint8_t *data);
uint64_t fpga_protocol_read_u64_le(const uint8_t *data);
void fpga_protocol_write_u32_le(uint8_t *data, uint32_t value);

/**
 * @brief 计算 CRC-16/CCITT-FALSE。
 * @param data 输入字节流。
 * @param length 输入长度。
 * @return CRC16；空指针且长度非零时返回初值 0xffff。
 */
uint16_t fpga_protocol_crc16(const uint8_t *data, uint32_t length);

/**
 * @brief 解析并校验 16 字节 GET_STATUS 响应。
 * @param data 响应首字节 5A 的地址。
 * @param length 必须为 16。
 * @param status 输出状态。
 * @return 解析结果。
 */
fpga_protocol_result_t fpga_protocol_parse_status(
    const uint8_t *data,
    uint32_t length,
    fpga_protocol_status_t *status);

/**
 * @brief 校验完整帧并解析 128 字节帧头。
 * @param frame DMA 收到的完整帧。
 * @param length 帧长度，包含末尾 CRC16。
 * @param expected_sequence GET_STATUS 给出的帧序号。
 * @param header 输出帧头字段。
 * @return 解析结果；成功后才允许访问时域和频谱载荷。
 */
fpga_protocol_result_t fpga_protocol_parse_frame(
    const uint8_t *frame,
    uint32_t length,
    uint32_t expected_sequence,
    fpga_protocol_frame_header_t *header);

#endif /* FPGA_PROTOCOL_H */
