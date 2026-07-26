import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def find_host_c_compiler() -> list[str] | None:
    configured = os.environ.get("STM32_HOST_CC")
    if configured:
        compiler = Path(configured)
        if compiler.is_file():
            return [str(compiler), "cc"] if compiler.name == "zig.exe" else [str(compiler)]

    for name in ("clang", "gcc"):
        compiler = shutil.which(name)
        if compiler:
            return [compiler]

    zig = shutil.which("zig")
    if zig:
        return [zig, "cc"]
    return None


class Ad9959RuntimeTest(unittest.TestCase):
    def test_driver_runtime_with_bidirectional_register_model(self) -> None:
        compiler = find_host_c_compiler()
        if compiler is None:
            self.skipTest("host C compiler not available")

        stub_system_header = r'''
#ifndef SYSTEM_H
#define SYSTEM_H

#include <stddef.h>
#include <stdint.h>
#include "ad9959.h"

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

typedef enum
{
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct { uint8_t id; } GPIO_TypeDef;
typedef struct { uint8_t id; } SPI_HandleTypeDef;
typedef struct { uint32_t DEMCR; } CoreDebug_Type;
typedef struct { uint32_t CTRL; uint32_t CYCCNT; } DWT_Type;

extern SPI_HandleTypeDef hspi4;
extern GPIO_TypeDef mock_cs_gpio;
extern GPIO_TypeDef mock_update_gpio;
extern GPIO_TypeDef mock_reset_gpio;
extern CoreDebug_Type mock_core_debug;
extern DWT_Type mock_dwt;
extern uint32_t SystemCoreClock;

#define AD9959_CS_GPIO_Port (&mock_cs_gpio)
#define AD9959_CS_Pin 5u
#define AD9959_IO_UPDATE_GPIO_Port (&mock_update_gpio)
#define AD9959_IO_UPDATE_Pin 4u
#define AD9959_RESET_GPIO_Port (&mock_reset_gpio)
#define AD9959_RESET_Pin 4u

#define CoreDebug (&mock_core_debug)
#define DWT (&mock_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define __NOP() do { mock_dwt.CYCCNT++; } while (0)

void HAL_GPIO_WritePin(GPIO_TypeDef *gpio_port,
                       uint16_t gpio_pin,
                       GPIO_PinState pin_state);
void HAL_Delay(uint32_t milliseconds);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          const uint8_t *transmit_data,
                                          uint8_t *receive_data,
                                          uint16_t size,
                                          uint32_t timeout);

#endif
'''

        harness_source = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "system.h"

#define REGISTER_COUNT 7u
#define MAX_REGISTER_BYTES 4u

SPI_HandleTypeDef hspi4;
GPIO_TypeDef mock_cs_gpio;
GPIO_TypeDef mock_update_gpio;
GPIO_TypeDef mock_reset_gpio;
CoreDebug_Type mock_core_debug;
DWT_Type mock_dwt;
uint32_t SystemCoreClock = 480000000u;

static const uint8_t register_lengths[REGISTER_COUNT] = {
    1u, 3u, 2u, 3u, 4u, 2u, 3u
};
static uint8_t global_registers[REGISTER_COUNT][MAX_REGISTER_BYTES];
static uint8_t channel_registers[4][REGISTER_COUNT][MAX_REGISTER_BYTES];
static GPIO_PinState cs_state = GPIO_PIN_SET;
static uint8_t selected_channel_mask = 0xF0u;
static uint32_t complete_write_frame_count;
static uint32_t complete_read_frame_count;
static uint32_t update_rising_edge_count;
static uint8_t spi_fail_next;
static uint8_t corrupt_next_read;

static void mock_reset_device(void)
{
    uint8_t channel;

    memset(global_registers, 0, sizeof(global_registers));
    memset(channel_registers, 0, sizeof(channel_registers));
    global_registers[ad9959_register_csr][0] = 0xF0u;
    selected_channel_mask = 0xF0u;
    for (channel = 0u; channel < 4u; channel++)
    {
        channel_registers[channel][ad9959_register_cfr][0] = 0x00u;
        channel_registers[channel][ad9959_register_cfr][1] = 0x03u;
        channel_registers[channel][ad9959_register_cfr][2] = 0x02u;
    }
}

static uint8_t first_selected_channel(void)
{
    uint8_t channel;

    for (channel = 0u; channel < 4u; channel++)
    {
        if ((selected_channel_mask & (uint8_t)(0x10u << channel)) != 0u)
        {
            return channel;
        }
    }
    assert(0);
    return 0u;
}

static void mock_write_register(uint8_t address, const uint8_t *data)
{
    uint8_t channel;
    uint8_t length = register_lengths[address];

    if (address == ad9959_register_csr)
    {
        global_registers[address][0] = data[0];
        selected_channel_mask = data[0] & 0xF0u;
        return;
    }
    if ((address == ad9959_register_fr1)
        || (address == ad9959_register_fr2))
    {
        memcpy(global_registers[address], data, length);
        return;
    }

    for (channel = 0u; channel < 4u; channel++)
    {
        if ((selected_channel_mask & (uint8_t)(0x10u << channel)) != 0u)
        {
            memcpy(channel_registers[channel][address], data, length);
        }
    }
}

static const uint8_t *mock_read_register(uint8_t address)
{
    if ((address == ad9959_register_csr)
        || (address == ad9959_register_fr1)
        || (address == ad9959_register_fr2))
    {
        return global_registers[address];
    }
    return channel_registers[first_selected_channel()][address];
}

void HAL_GPIO_WritePin(GPIO_TypeDef *gpio_port,
                       uint16_t gpio_pin,
                       GPIO_PinState pin_state)
{
    (void)gpio_pin;
    if (gpio_port == &mock_cs_gpio)
    {
        cs_state = pin_state;
    }
    else if ((gpio_port == &mock_update_gpio)
             && (pin_state == GPIO_PIN_SET))
    {
        update_rising_edge_count++;
    }
    else if ((gpio_port == &mock_reset_gpio)
             && (pin_state == GPIO_PIN_SET))
    {
        mock_reset_device();
    }
}

void HAL_Delay(uint32_t milliseconds)
{
    (void)milliseconds;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout)
{
    uint8_t address;

    (void)timeout;
    assert(spi == &hspi4);
    assert(cs_state == GPIO_PIN_RESET);
    assert(size >= 2u);
    address = data[0] & 0x7Fu;
    assert(address < REGISTER_COUNT);
    assert(size == (uint16_t)(register_lengths[address] + 1u));
    complete_write_frame_count++;

    if (spi_fail_next != 0u)
    {
        spi_fail_next = 0u;
        return HAL_ERROR;
    }
    mock_write_register(address, &data[1]);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          const uint8_t *transmit_data,
                                          uint8_t *receive_data,
                                          uint16_t size,
                                          uint32_t timeout)
{
    const uint8_t *register_data;
    uint8_t address;

    (void)timeout;
    assert(spi == &hspi4);
    assert(cs_state == GPIO_PIN_RESET);
    assert((transmit_data[0] & 0x80u) != 0u);
    address = transmit_data[0] & 0x7Fu;
    assert(address < REGISTER_COUNT);
    assert(size == (uint16_t)(register_lengths[address] + 1u));
    complete_read_frame_count++;

    if (spi_fail_next != 0u)
    {
        spi_fail_next = 0u;
        return HAL_ERROR;
    }

    memset(receive_data, 0, size);
    register_data = mock_read_register(address);
    memcpy(&receive_data[1], register_data, register_lengths[address]);
    if (corrupt_next_read != 0u)
    {
        receive_data[1] ^= 0x01u;
        corrupt_next_read = 0u;
    }
    return HAL_OK;
}

int main(void)
{
    uint8_t readback[4];
    uint32_t previous_frequency_hz;
    uint32_t write_frames_before_error;
    uint32_t initial_ftw = ad9959_calculate_tuning_word(1000000u);
    uint32_t ten_mhz_ftw = ad9959_calculate_tuning_word(10000000u);

    assert(initial_ftw == 0x0083126Fu);
    assert(ad9959_calculate_phase_word(90u) == 0x1000u);
    assert(ad9959_set_frequency(ad9959_channel_0, 1000u)
           == ad9959_status_not_initialized);

    assert(ad9959_init() == ad9959_status_ok);
    assert(ad9959_diagnostics.initialized == 1u);
    assert(ad9959_diagnostics.verified == 1u);
    assert(ad9959_diagnostics.readback_mismatch_mask == 0u);
    assert(ad9959_diagnostics.frequency_hz[0] == 1000000u);
    assert(ad9959_diagnostics.frequency_hz[1] == 1000000u);
    assert(ad9959_diagnostics.fr2_readback[0] == 0x00u);
    assert(ad9959_diagnostics.fr2_readback[1] == 0x00u);
    assert(ad9959_diagnostics.cpow_readback[0][0] == 0x00u);
    assert(ad9959_diagnostics.cpow_readback[0][1] == 0x00u);
    assert(ad9959_diagnostics.cpow_readback[1][0] == 0x00u);
    assert(ad9959_diagnostics.cpow_readback[1][1] == 0x00u);
    assert(ad9959_diagnostics.acr_readback[0][0] == 0x00u);
    assert(ad9959_diagnostics.acr_readback[0][1] == 0x13u);
    assert(ad9959_diagnostics.acr_readback[0][2] == 0xFFu);
    assert(ad9959_diagnostics.acr_readback[1][0] == 0x00u);
    assert(ad9959_diagnostics.acr_readback[1][1] == 0x13u);
    assert(ad9959_diagnostics.acr_readback[1][2] == 0xFFu);
    assert(global_registers[ad9959_register_csr][0] == 0x12u);
    assert(global_registers[ad9959_register_fr1][0] == 0xD0u);
    assert(global_registers[ad9959_register_fr1][1] == 0x00u);
    assert(global_registers[ad9959_register_fr1][2] == 0x00u);
    assert(channel_registers[0][ad9959_register_cfr][2] == 0x02u);
    assert(channel_registers[1][ad9959_register_cfr][2] == 0x02u);
    assert(channel_registers[2][ad9959_register_cfr][2] == 0xC2u);
    assert(channel_registers[3][ad9959_register_cfr][2] == 0xC2u);
    assert(channel_registers[0][ad9959_register_ftw0][0] == 0x00u);
    assert(channel_registers[0][ad9959_register_ftw0][1] == 0x83u);
    assert(channel_registers[0][ad9959_register_ftw0][2] == 0x12u);
    assert(channel_registers[0][ad9959_register_ftw0][3] == 0x6Fu);
    assert(channel_registers[0][ad9959_register_acr][0] == 0x00u);
    assert(channel_registers[0][ad9959_register_acr][1] == 0x13u);
    assert(channel_registers[0][ad9959_register_acr][2] == 0xFFu);
    assert(complete_write_frame_count > 0u);
    assert(complete_read_frame_count > 0u);
    assert(update_rising_edge_count == 2u);

    assert(ad9959_set_frequency(ad9959_channel_1, 10000000u)
           == ad9959_status_ok);
    assert(ad9959_diagnostics.frequency_tuning_word[1] == ten_mhz_ftw);
    assert(ad9959_set_phase_degrees(ad9959_channel_1, 90u)
           == ad9959_status_ok);
    assert(channel_registers[1][ad9959_register_cpow0][0] == 0x10u);
    assert(channel_registers[1][ad9959_register_cpow0][1] == 0x00u);
    assert(ad9959_set_amplitude(ad9959_channel_0, 512u)
           == ad9959_status_ok);
    assert(channel_registers[0][ad9959_register_acr][1] == 0x12u);
    assert(channel_registers[0][ad9959_register_acr][2] == 0x00u);

    assert(ad9959_read_register(ad9959_channel_1,
                                ad9959_register_ftw0,
                                readback,
                                4u) == ad9959_status_ok);
    assert(memcmp(readback,
                  channel_registers[1][ad9959_register_ftw0],
                  4u) == 0);
    assert(ad9959_verify_configuration() == ad9959_status_ok);
    assert(ad9959_diagnostics.cpow_readback[1][0] == 0x10u);
    assert(ad9959_diagnostics.cpow_readback[1][1] == 0x00u);
    assert(ad9959_diagnostics.acr_readback[0][1] == 0x12u);
    assert(ad9959_diagnostics.acr_readback[0][2] == 0x00u);

    assert(ad9959_set_frequency((ad9959_channel_t)2, 1000u)
           == ad9959_status_invalid_channel);
    assert(ad9959_set_frequency(ad9959_channel_0, 0u)
           == ad9959_status_invalid_frequency);
    assert(ad9959_set_phase_degrees(ad9959_channel_0, 360u)
           == ad9959_status_invalid_phase);
    assert(ad9959_set_amplitude(ad9959_channel_0, 1024u)
           == ad9959_status_invalid_amplitude);

    previous_frequency_hz = ad9959_diagnostics.frequency_hz[0];
    corrupt_next_read = 1u;
    assert(ad9959_set_frequency(ad9959_channel_0, 2000000u)
           == ad9959_status_readback_mismatch);
    assert(ad9959_diagnostics.frequency_hz[0] == previous_frequency_hz);

    write_frames_before_error = complete_write_frame_count;
    spi_fail_next = 1u;
    assert(ad9959_set_frequency(ad9959_channel_0, 3000000u)
           == ad9959_status_spi_error);
    assert(complete_write_frame_count == write_frames_before_error + 1u);
    assert(ad9959_diagnostics.frequency_hz[0] == previous_frequency_hz);
    assert(ad9959_diagnostics.error_count >= 6u);

    corrupt_next_read = 1u;
    assert(ad9959_init() == ad9959_status_readback_mismatch);
    assert(ad9959_diagnostics.initialized == 0u);
    assert(ad9959_diagnostics.verified == 0u);
    assert((ad9959_diagnostics.readback_mismatch_mask
            & AD9959_MISMATCH_FR1) != 0u);
    return 0;
}
'''

        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            (build_directory / "ad9959.h").write_text(
                (ROOT / "Core/User/ad9959.h").read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            (build_directory / "ad9959.c").write_text(
                (ROOT / "Core/User/ad9959.c").read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            (build_directory / "system.h").write_text(
                stub_system_header,
                encoding="utf-8",
            )
            (build_directory / "harness.c").write_text(
                harness_source,
                encoding="utf-8",
            )

            executable = build_directory / "ad9959_host_test.exe"
            compile_result = subprocess.run(
                compiler
                + [
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "harness.c",
                    "ad9959.c",
                    "-o",
                    str(executable),
                ],
                cwd=build_directory,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                0,
                compile_result.returncode,
                compile_result.stdout + compile_result.stderr,
            )

            run_result = subprocess.run(
                [str(executable)],
                cwd=build_directory,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                0,
                run_result.returncode,
                run_result.stdout + run_result.stderr,
            )


if __name__ == "__main__":
    unittest.main()
