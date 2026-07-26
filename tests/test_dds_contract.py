import os
import re
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def find_host_c_compiler() -> list[str] | None:
    configured = os.environ.get("AD9834_HOST_CC")
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


class DdsContractTest(unittest.TestCase):
    def test_all_hal_driver_sources_are_included(self) -> None:
        project = ET.fromstring(read_text(".cproject"))
        source_entries = [
            entry
            for entry in project.iter("entry")
            if entry.get("name") == "Drivers/STM32H7xx_HAL_Driver/Src"
        ]

        self.assertGreater(len(source_entries), 0)
        for entry in source_entries:
            excluded = entry.get("excluding", "").split("|")
            excluded_hal = [
                name
                for name in excluded
                if name.startswith("stm32h7xx_hal_")
                and not name.endswith("_template.c")
            ]
            self.assertEqual([], excluded_hal)

    def test_ioc_matches_ad9834_spi_contract(self) -> None:
        ioc = read_text("h743_pre1.ioc")

        required_lines = (
            "PB12.GPIO_Label=DDS_FSYNC",
            "PB12.PinState=GPIO_PIN_SET",
            "PB12.Signal=GPIO_Output",
            "PB13.Signal=SPI2_SCK",
            "PB14.GPIO_Label=FS",
            "PB14.Signal=GPIO_Output",
            "PB15.Signal=SPI2_MOSI",
            "PD8.GPIO_Label=PS",
            "PD8.Signal=GPIO_Output",
            "RCC.SPI123CLockSelection=RCC_SPI123CLKSOURCE_CLKP",
            "RCC.SPI123Freq_Value=64000000",
            "SPI2.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8",
            "SPI2.CalculateBaudRate=8.0 MBits/s",
            "SPI2.CLKPolarity=SPI_POLARITY_HIGH",
            "SPI2.DataSize=SPI_DATASIZE_16BIT",
            "SPI2.Direction=SPI_DIRECTION_2LINES_TXONLY",
            "SPI2.NSSPMode=SPI_NSS_PULSE_DISABLE",
        )
        for line in required_lines:
            self.assertIn(line, ioc)

    def test_alternating_frequency_apis_use_lowercase_names(self) -> None:
        first_header = read_text("Core/User/ad9834.h")
        first_source = read_text("Core/User/ad9834.c")

        self.assertIn(
            "ad9834_status_t dds_set_frequency(uint32_t frequency_hz);",
            first_header,
        )
        self.assertIn(
            "ad9834_status_t dds_set_frequency(uint32_t frequency_hz)",
            first_source,
        )
        self.assertNotIn("DDS_SetFrequency", first_header)
        self.assertNotIn("DDS_SetFrequency", first_source)

    def test_ioc_matches_ad9959_spi4_contract(self) -> None:
        ioc = read_text("h743_pre1.ioc")
        spi_source = read_text("Core/Src/spi.c")
        main = read_text("Core/Src/main.c")

        required_lines = (
            "PD4.GPIO_Label=update9959",
            "PD5.GPIO_Label=AD9959_CS",
            "PD5.PinState=GPIO_PIN_SET",
            "PD5.GPIO_Speed=GPIO_SPEED_FREQ_HIGH",
            "PB4\\ (NJTRST).GPIO_Label=AD9959_RST",
            "PE2.Signal=SPI4_SCK",
            "PE5.Signal=SPI4_MISO",
            "PE6.Signal=SPI4_MOSI",
            "SPI4.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8",
            "SPI4.DataSize=SPI_DATASIZE_8BIT",
            "SPI4.Direction=SPI_DIRECTION_2LINES",
        )
        for line in required_lines:
            self.assertIn(line, ioc)
        self.assertIn("hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;", spi_source)
        self.assertIn("hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;", spi_source)
        self.assertIn("hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;", spi_source)
        self.assertLess(main.index("MX_SPI4_Init();"), main.index("system_init();"))

    def test_ad9959_driver_declares_public_api(self) -> None:
        header_path = ROOT / "Core/User/ad9959.h"

        self.assertTrue(header_path.is_file(), "AD9959 头文件尚未创建")
        header = header_path.read_text(encoding="utf-8")
        required = (
            "#define AD9959_MCLK_HZ 500000000u",
            "#define AD9959_MAX_OUTPUT_HZ 200000000u",
            "#define AD9959_AMPLITUDE_MAX 1023u",
            "extern volatile ad9959_diagnostics_t ad9959_diagnostics;",
            "ad9959_status_t ad9959_init(void);",
            "ad9959_status_t ad9959_set_frequency(ad9959_channel_t channel,",
            "ad9959_status_t ad9959_set_phase(ad9959_channel_t channel,",
            "ad9959_status_t ad9959_set_amplitude(ad9959_channel_t channel,",
            "ad9959_status_t ad9959_read_register(uint8_t address, uint8_t *data,",
            "uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz);",
            "ad9959_channel_0",
            "ad9959_channel_1",
            "ad9959_status_spi_error",
        )
        for text in required:
            self.assertIn(text, header)

    def test_ad9959_driver_uses_spi4_and_dedicated_gpio(self) -> None:
        source_path = ROOT / "Core/User/ad9959.c"

        self.assertTrue(source_path.is_file(), "AD9959 源文件尚未创建")
        source = source_path.read_text(encoding="utf-8")
        required = (
            "HAL_SPI_Transmit(&hspi4",
            "HAL_SPI_TransmitReceive(&hspi4",
            "AD9959_CS_GPIO_Port",
            "AD9959_CS_Pin",
            "AD9959_RST_GPIO_Port",
            "AD9959_RST_Pin",
            "update9959_GPIO_Port",
            "update9959_Pin",
            "AD9959_REG_CSR    0x00u",
            "AD9959_REG_FR1    0x01u",
            "AD9959_REG_CFTW0  0x04u",
            "AD9959_REG_CPOW0  0x05u",
            "AD9959_REG_ACR    0x06u",
            "ad9959_fr1_default[3] = { 0xD0u, 0x00u, 0x00u }",
            "ad9959_cfr_default[3] = { 0x00u, 0x03u, 0x02u }",
            "AD9959_ACR_AMPLITUDE_ENABLE 0x10u",
            "ad9959_io_update()",
            "ad9959_hardware_reset()",
        )
        for text in required:
            self.assertIn(text, source)

    def test_ad9959_guide_keeps_power_down_control_low(self) -> None:
        guide = read_text("docs/AD9959上板测试指南.md")

        self.assertIn("| GND | PDC | PWR_DWN_CTL |", guide)
        self.assertNotIn("| PDC | PWR_DWN_CTL | 浮空", guide)

    def test_ad9959_uses_500mhz_ftw(self) -> None:
        compiler = find_host_c_compiler()
        if compiler is None:
            self.skipTest("host C compiler not available")

        header = read_text("Core/User/ad9959.h")
        source = read_text("Core/User/ad9959.c")

        self.assertIn("#define AD9959_MCLK_HZ 500000000u", header)
        function = re.search(
            r"uint32_t ad9959_calculate_tuning_word\(uint32_t frequency_hz\)"
            r"\s*\{.*?\n\}",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(function)

        harness_source = (
            '#include <assert.h>\n'
            '#include "ad9959.h"\n\n'
            f"{function.group(0)}\n\n"
            "int main(void)\n"
            "{\n"
            "    assert(ad9959_calculate_tuning_word(1u) == 9u);\n"
            "    assert(ad9959_calculate_tuning_word(1000000u) == 8589935u);\n"
            "    assert(ad9959_calculate_tuning_word(10000000u) == 85899346u);\n"
            "    return 0;\n"
            "}\n"
        )

        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            (build_directory / "ad9959.h").write_text(
                header, encoding="utf-8"
            )
            (build_directory / "harness.c").write_text(
                harness_source, encoding="utf-8"
            )
            executable = build_directory / "ad9959_ftw_test.exe"
            compile_result = subprocess.run(
                compiler
                + [
                    "-std=c11",
                    "-O3",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "harness.c",
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

    def test_driver_uses_75mhz_ftw_and_manual_fsync(self) -> None:
        header = read_text("Core/User/ad9834.h")
        source = read_text("Core/User/ad9834.c")

        self.assertIn("#define AD9834_MCLK_HZ 75000000u", header)
        self.assertIn("((uint64_t)frequency_hz) << 28", source)
        self.assertIn("AD9834_CONTROL_RESET 0x2300u", source)
        self.assertIn("AD9834_CONTROL_RUN   0x2200u", source)
        self.assertIn("HAL_SPI_Transmit(&hspi2", source)

        fsync_low = source.index("DDS_FSYNC_Pin, GPIO_PIN_RESET")
        transmit = source.index("HAL_SPI_Transmit(&hspi2")
        fsync_high = source.index("DDS_FSYNC_Pin, GPIO_PIN_SET", transmit)
        self.assertLess(fsync_low, transmit)
        self.assertLess(transmit, fsync_high)

    def test_driver_controls_frequency_and_phase_select_pins(self) -> None:
        header = read_text("Core/User/ad9834.h")
        source = read_text("Core/User/ad9834.c")

        self.assertIn("ad9834_select_frequency_register", header)
        self.assertIn("ad9834_select_phase_register", header)
        self.assertIn("HAL_GPIO_WritePin(FS_GPIO_Port, FS_Pin, pin_state);", source)
        self.assertIn("HAL_GPIO_WritePin(PS_GPIO_Port, PS_Pin, pin_state);", source)
        self.assertIn(
            "ad9834_select_frequency_register(ad9834_frequency_register_0);",
            source,
        )
        self.assertIn(
            "ad9834_select_phase_register(ad9834_phase_register_0);",
            source,
        )

    def test_driver_declares_dual_frequency_and_phase_writers(self) -> None:
        header = read_text("Core/User/ad9834.h")

        self.assertIn("ad9834_status_invalid_register", header)
        self.assertIn("ad9834_status_invalid_phase", header)
        self.assertIn("ad9834_set_frequency_register_hz", header)
        self.assertIn("ad9834_set_phase_register_degrees", header)

    def test_driver_uses_all_ad9834_register_addresses(self) -> None:
        source = read_text("Core/User/ad9834.c")

        for text in (
            "AD9834_FREQ0_ADDRESS  0x4000u",
            "AD9834_FREQ1_ADDRESS  0x8000u",
            "AD9834_PHASE0_ADDRESS 0xC000u",
            "AD9834_PHASE1_ADDRESS 0xE000u",
            "phase_degrees > 359u",
            "((uint32_t)phase_degrees * 4096u) + 180u",
        ):
            self.assertIn(text, source)

    def test_driver_preserves_freq0_compatibility_and_initializes_both_banks(
        self,
    ) -> None:
        source = read_text("Core/User/ad9834.c")

        self.assertIn(
            "ad9834_set_frequency_register_hz(\n"
            "        ad9834_frequency_register_0,\n"
            "        frequency_hz)",
            source,
        )
        self.assertGreaterEqual(source.count("ad9834_frequency_register_0"), 3)
        self.assertGreaterEqual(source.count("ad9834_frequency_register_1"), 2)
        self.assertGreaterEqual(source.count("ad9834_phase_register_0"), 3)
        self.assertGreaterEqual(source.count("ad9834_phase_register_1"), 2)

    def test_driver_runtime_with_mocked_hal(self) -> None:
        compiler = find_host_c_compiler()
        if compiler is None:
            self.skipTest("host C compiler not available")

        stub_system_header = r'''
#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>
#include "ad9834.h"

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

extern SPI_HandleTypeDef hspi2;
extern GPIO_TypeDef mock_fsync_gpio;
extern GPIO_TypeDef mock_frequency_select_gpio;
extern GPIO_TypeDef mock_phase_select_gpio;

#define DDS_FSYNC_GPIO_Port (&mock_fsync_gpio)
#define DDS_FSYNC_Pin 12u
#define FS_GPIO_Port (&mock_frequency_select_gpio)
#define FS_Pin 14u
#define PS_GPIO_Port (&mock_phase_select_gpio)
#define PS_Pin 8u

void HAL_GPIO_WritePin(GPIO_TypeDef *gpio_port, uint16_t gpio_pin,
                       GPIO_PinState pin_state);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout);

#endif
'''
        harness_source = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "system.h"

SPI_HandleTypeDef hspi2;
GPIO_TypeDef mock_fsync_gpio;
GPIO_TypeDef mock_frequency_select_gpio;
GPIO_TypeDef mock_phase_select_gpio;

static uint16_t spi_words[32];
static uint32_t spi_attempt_count;
static uint32_t fail_on_attempt;
static uint32_t frequency_select_write_count;
static uint32_t phase_select_write_count;
static GPIO_PinState frequency_select_state;
static GPIO_PinState phase_select_state;

void HAL_GPIO_WritePin(GPIO_TypeDef *gpio_port, uint16_t gpio_pin,
                       GPIO_PinState pin_state)
{
    (void)gpio_pin;
    if (gpio_port == &mock_frequency_select_gpio)
    {
        frequency_select_write_count++;
        frequency_select_state = pin_state;
    }
    else if (gpio_port == &mock_phase_select_gpio)
    {
        phase_select_write_count++;
        phase_select_state = pin_state;
    }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   const uint8_t *data,
                                   uint16_t size,
                                   uint32_t timeout)
{
    uint16_t word;

    (void)spi;
    (void)timeout;
    assert(size == 1u);
    memcpy(&word, data, sizeof(word));
    assert(spi_attempt_count < 32u);
    spi_words[spi_attempt_count] = word;
    spi_attempt_count++;
    return (spi_attempt_count == fail_on_attempt) ? HAL_ERROR : HAL_OK;
}

int main(void)
{
    uint32_t tuning_word;
    uint32_t attempts_before;
    uint32_t frequency_select_writes_before;
    uint32_t phase_select_writes_before;
    uint32_t previous_frequency_hz;

    assert(ad9834_init(1000000u) == ad9834_status_ok);
    tuning_word = ad9834_calculate_tuning_word(1000000u);
    assert(spi_attempt_count == 8u);
    assert(spi_words[0] == 0x2300u);
    assert(spi_words[1] == (uint16_t)(0x4000u | (tuning_word & 0x3FFFu)));
    assert(spi_words[2] == (uint16_t)(0x4000u | ((tuning_word >> 14) & 0x3FFFu)));
    assert(spi_words[3] == (uint16_t)(0x8000u | (tuning_word & 0x3FFFu)));
    assert(spi_words[4] == (uint16_t)(0x8000u | ((tuning_word >> 14) & 0x3FFFu)));
    assert(spi_words[5] == 0xC000u);
    assert(spi_words[6] == 0xE000u);
    assert(spi_words[7] == 0x2200u);
    assert(frequency_select_state == GPIO_PIN_RESET);
    assert(phase_select_state == GPIO_PIN_RESET);
    assert(ad9834_diagnostics.selected_frequency_register == 0u);
    assert(ad9834_diagnostics.selected_phase_register == 0u);
    assert(ad9834_diagnostics.initialized == 1u);

    assert(ad9834_set_frequency_register_hz(
               ad9834_frequency_register_1, 2000000u) == ad9834_status_ok);
    assert((spi_words[8] & 0xC000u) == 0x8000u);
    assert((spi_words[9] & 0xC000u) == 0x8000u);
    assert(ad9834_diagnostics.frequency_hz[1] == 2000000u);

    assert(ad9834_set_phase_register_degrees(
               ad9834_phase_register_0, 0u) == ad9834_status_ok);
    assert(spi_words[10] == 0xC000u);
    assert(ad9834_set_phase_register_degrees(
               ad9834_phase_register_1, 359u) == ad9834_status_ok);
    assert(spi_words[11] == (uint16_t)(0xE000u | 4085u));
    assert(ad9834_diagnostics.phase_word[1] == 4085u);

    assert(ad9834_set_frequency_hz(123456u) == ad9834_status_ok);
    assert((spi_words[12] & 0xC000u) == 0x4000u);
    assert((spi_words[13] & 0xC000u) == 0x4000u);

    attempts_before = spi_attempt_count;
    frequency_select_writes_before = frequency_select_write_count;
    phase_select_writes_before = phase_select_write_count;
    assert(ad9834_set_frequency_register_hz(
               (ad9834_frequency_register_t)2, 1000u)
           == ad9834_status_invalid_register);
    assert(ad9834_set_frequency_register_hz(
               ad9834_frequency_register_0, 0u)
           == ad9834_status_invalid_frequency);
    assert(ad9834_set_phase_register_degrees(
               (ad9834_phase_register_t)2, 0u)
           == ad9834_status_invalid_register);
    assert(ad9834_set_phase_register_degrees(
               ad9834_phase_register_0, 360u)
           == ad9834_status_invalid_phase);
    ad9834_select_frequency_register((ad9834_frequency_register_t)2);
    ad9834_select_phase_register((ad9834_phase_register_t)2);
    assert(spi_attempt_count == attempts_before);
    assert(frequency_select_write_count == frequency_select_writes_before);
    assert(phase_select_write_count == phase_select_writes_before);

    previous_frequency_hz = ad9834_diagnostics.frequency_hz[0];
    fail_on_attempt = spi_attempt_count + 2u;
    assert(ad9834_set_frequency_register_hz(
               ad9834_frequency_register_0, 3000000u)
           == ad9834_status_spi_error);
    assert(ad9834_diagnostics.frequency_hz[0] == previous_frequency_hz);
    assert(ad9834_diagnostics.error_count == 1u);
    return 0;
}
'''

        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            (build_directory / "ad9834.h").write_text(
                read_text("Core/User/ad9834.h"), encoding="utf-8"
            )
            (build_directory / "ad9834.c").write_text(
                read_text("Core/User/ad9834.c"), encoding="utf-8"
            )
            (build_directory / "system.h").write_text(
                stub_system_header, encoding="utf-8"
            )
            (build_directory / "harness.c").write_text(
                harness_source, encoding="utf-8"
            )
            executable = build_directory / "ad9834_host_test.exe"
            compile_result = subprocess.run(
                compiler
                + [
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "harness.c",
                    "ad9834.c",
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

    def test_readme_documents_dual_register_programming_and_selection(self) -> None:
        readme = read_text("README.md")

        for text in (
            "ad9834_set_frequency_register_hz(ad9834_frequency_register_1, 2000000u);",
            "ad9834_set_phase_register_degrees(ad9834_phase_register_1, 90u);",
            "ad9834_select_frequency_register(ad9834_frequency_register_1);",
            "ad9834_select_phase_register(ad9834_phase_register_1);",
            "ad9834_set_frequency_hz()",
        ):
            self.assertIn(text, readme)

    def test_formal_mode_tracks_input_minus_100khz(self) -> None:
        header = read_text("Core/User/dds_control.h")
        source = read_text("Core/User/dds_control.c")

        self.assertIn("#define DDS_CONTROL_FIXED_TEST_ENABLE 0u", header)
        self.assertIn("#define DDS_CONTROL_TEST_INPUT_HZ 1000000u", header)
        self.assertIn("#define DDS_CONTROL_TEST_OUTPUT_HZ 100000u", header)
        self.assertIn("#define DDS_CONTROL_TARGET_IF_HZ 100000u", header)
        self.assertIn("return input_frequency_hz - DDS_CONTROL_TARGET_IF_HZ;", source)
        self.assertEqual(1_000_000 - 100_000, 900_000)

        expected_ftw = ((900_000 << 28) + 75_000_000 // 2) // 75_000_000
        self.assertEqual(expected_ftw, 3_221_225)

    def test_adc_compensation_api_states_and_diagnostics(self) -> None:
        header = read_text("Core/User/dds_control.h")

        required = (
            "#define DDS_CONTROL_COMPENSATION_LIMIT 10u",
            "dds_control_state_coarse",
            "dds_control_state_compensating",
            "dds_control_state_holding",
            "uint8_t compensation_count;",
            "uint32_t compensation_request_count;",
            "float adc_frequency_hz;",
            "int32_t frequency_error_hz;",
            "uint32_t last_result_sequence;",
            "void dds_control_request_compensation(void);",
        )
        for text in required:
            self.assertIn(text, header)

    def test_adc_compensation_math_direction(self) -> None:
        def compensate(current_dds_hz: int, adc_hz: float) -> int:
            error_hz = round(adc_hz - 100_000.0)
            return current_dds_hz + error_hz

        self.assertEqual(900_250, compensate(900_000, 100_250.0))
        self.assertEqual(899_750, compensate(900_000, 99_750.0))
        self.assertEqual(900_000, compensate(900_000, 100_000.0))

    def test_adc_compensation_consumes_five_new_valid_fft_results(self) -> None:
        source = read_text("Core/User/dds_control.c")

        required = (
            "measurement_result_get_snapshot(&result)",
            "MEASUREMENT_VALID_FREQUENCY",
            "result.sequence == dds_control_diagnostics.last_result_sequence",
            "dds_control_diagnostics.frequency_error_hz",
            "measurement_fft_resynchronize();",
            "dds_control_diagnostics.compensation_count++;",
            ">= DDS_CONTROL_COMPENSATION_LIMIT",
            "dds_control_state_holding",
        )
        for text in required:
            self.assertIn(text, source)

        self.assertIn("AD9834_MAX_OUTPUT_HZ", source)
        self.assertIn("isfinite(result.frequency_hz)", source)

    def test_hmi_immediate_measurement_starts_compensation(self) -> None:
        source = read_text("Core/User/hmi_tjc.c")
        command_branch = source.split(
            "else if ((command == (uint8_t)'M')", 1
        )[1].split("else", 1)[0]

        self.assertIn("frequency_measure_request_now();", command_branch)
        self.assertIn("dds_control_request_compensation();", command_branch)

    def test_system_initializes_spi_before_dds(self) -> None:
        main = read_text("Core/Src/main.c")
        system = read_text("Core/User/system.c")

        self.assertLess(main.index("MX_SPI2_Init();"), main.index("system_init();"))
        self.assertIn("dds_control_init();", system)
        self.assertIn("dds_control_process();", system)

        init_body = re.search(
            r"void system_init\(void\)\s*\{(?P<body>.*?)\n\}",
            system,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(init_body)
        self.assertLess(
            init_body.group("body").index("frequency_measure_init();"),
            init_body.group("body").index("dds_control_init();"),
        )

    def test_system_initializes_ad9959_after_spi4(self) -> None:
        main = read_text("Core/Src/main.c")
        system_header = read_text("Core/User/system.h")
        system_source = read_text("Core/User/system.c")

        self.assertLess(main.index("MX_SPI4_Init();"), main.index("system_init();"))
        self.assertIn('#include "ad9959.h"', system_header)
        self.assertIn("(void)ad9959_init();", system_source)


if __name__ == "__main__":
    unittest.main()
