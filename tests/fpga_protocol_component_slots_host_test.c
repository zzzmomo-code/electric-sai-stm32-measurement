#include "fpga_protocol.h"

#include <stdint.h>
#include <string.h>

#define TEST_FRAME_LENGTH 2904u
#define TEST_FRAME_SEQUENCE 0x01020304u
#define TEST_COMPONENT_FLAGS_OFFSET 13u

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static uint8_t popcount_three_bits(uint8_t value)
{
    uint8_t count = 0u;
    uint8_t index;

    for (index = 0u; index < FPGA_PROTOCOL_COMPONENT_MAX; index++)
    {
        if ((value & (uint8_t)(1u << index)) != 0u)
        {
            count++;
        }
    }
    return count;
}

static void build_minimum_frame(uint8_t *frame, uint8_t valid_pattern)
{
    uint8_t component_index;
    uint16_t crc;

    memset(frame, 0, TEST_FRAME_LENGTH);
    frame[0] = (uint8_t)'G';
    frame[1] = (uint8_t)'2';
    frame[2] = (uint8_t)'6';
    frame[3] = (uint8_t)'F';
    frame[4] = FPGA_PROTOCOL_VERSION_MAJOR;
    frame[5] = FPGA_PROTOCOL_VERSION_MINOR;
    write_u16_le(&frame[6], FPGA_PROTOCOL_HEADER_BYTES);
    fpga_protocol_write_u32_le(&frame[8], TEST_FRAME_LENGTH);
    fpga_protocol_write_u32_le(&frame[12], TEST_FRAME_SEQUENCE);
    fpga_protocol_write_u32_le(
        &frame[24], FPGA_PROTOCOL_HEADER_MEASUREMENT_VALID);
    fpga_protocol_write_u32_le(
        &frame[28], FPGA_PROTOCOL_TIME_SAMPLE_RATE_HZ);
    write_u16_le(&frame[32], FPGA_PROTOCOL_MIN_TIME_SAMPLES);
    write_u16_le(&frame[34], FPGA_PROTOCOL_TIME_UV_PER_LSB);
    fpga_protocol_write_u32_le(
        &frame[36], FPGA_PROTOCOL_FFT_SAMPLE_RATE_HZ);
    write_u16_le(&frame[40], FPGA_PROTOCOL_FFT_LENGTH);
    write_u16_le(&frame[42], FPGA_PROTOCOL_SPECTRUM_COUNT);
    fpga_protocol_write_u32_le(
        &frame[44], FPGA_PROTOCOL_BIN_SPACING_MHZ);
    write_u16_le(&frame[48], FPGA_PROTOCOL_SPECTRUM_UV_PER_LSB);
    frame[50] = popcount_three_bits(valid_pattern);
    fpga_protocol_write_u32_le(&frame[64], 12345000u);

    for (component_index = 0u;
         component_index < FPGA_PROTOCOL_COMPONENT_MAX;
         component_index++)
    {
        uint32_t offset = 68u
                          + ((uint32_t)component_index
                             * FPGA_PROTOCOL_COMPONENT_BYTES);

        fpga_protocol_write_u32_le(
            &frame[offset], 12345000u * (uint32_t)(component_index + 1u));
        fpga_protocol_write_u32_le(
            &frame[offset + 4u], 1000000u);
        write_u16_le(&frame[offset + 8u], (uint16_t)(32u + component_index));
        frame[offset + 12u] = (uint8_t)(component_index + 1u);
        if ((valid_pattern & (uint8_t)(1u << component_index)) != 0u)
        {
            frame[offset + TEST_COMPONENT_FLAGS_OFFSET] =
                FPGA_PROTOCOL_COMPONENT_VALID;
        }
    }

    crc = fpga_protocol_crc16(frame, TEST_FRAME_LENGTH - 2u);
    write_u16_le(&frame[TEST_FRAME_LENGTH - 2u], crc);
}

int main(void)
{
    static uint8_t frame[TEST_FRAME_LENGTH];
    fpga_protocol_frame_header_t header;
    uint8_t valid_pattern;

    for (valid_pattern = 0u; valid_pattern < 8u; valid_pattern++)
    {
        build_minimum_frame(frame, valid_pattern);
        if (fpga_protocol_parse_frame(
                frame,
                TEST_FRAME_LENGTH,
                TEST_FRAME_SEQUENCE,
                &header) != FPGA_PROTOCOL_OK)
        {
            return (int)(valid_pattern + 1u);
        }
        if (header.component_count != popcount_three_bits(valid_pattern))
        {
            return (int)(20u + valid_pattern);
        }
    }

    build_minimum_frame(frame, 0x05u);
    frame[50] = 1u;
    write_u16_le(
        &frame[TEST_FRAME_LENGTH - 2u],
        fpga_protocol_crc16(frame, TEST_FRAME_LENGTH - 2u));
    if (fpga_protocol_parse_frame(
            frame,
            TEST_FRAME_LENGTH,
            TEST_FRAME_SEQUENCE,
            &header) != FPGA_PROTOCOL_ERROR_FIELD)
    {
        return 40;
    }

    return 0;
}
