import re
import shutil
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (
                crc << 1
            ) & 0xFFFF
    return crc


def function_body(source: str, function_name: str) -> str:
    start = source.index(f"void {function_name}")
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {function_name}")


class FpgaSpiHmiContractTest(unittest.TestCase):
    def setUp(self):
        self.protocol = (ROOT / "Core/User/fpga_protocol.c").read_text(
            encoding="utf-8"
        )
        self.protocol_h = (ROOT / "Core/User/fpga_protocol.h").read_text(
            encoding="utf-8"
        )
        self.link = (ROOT / "Core/User/fpga_link.c").read_text(
            encoding="utf-8"
        )
        self.link_h = (ROOT / "Core/User/fpga_link.h").read_text(
            encoding="utf-8"
        )
        self.conversion = (
            ROOT / "Core/User/measurement_conversion.c"
        ).read_text(encoding="utf-8")
        self.conversion_h = (
            ROOT / "Core/User/measurement_conversion.h"
        ).read_text(encoding="utf-8")
        self.chart = (ROOT / "Core/User/hmi_chart.c").read_text(
            encoding="utf-8"
        )
        self.chart_h = (ROOT / "Core/User/hmi_chart.h").read_text(
            encoding="utf-8"
        )
        self.ioc = (ROOT / "h743_task2_20260727.ioc").read_text(
            encoding="utf-8"
        )
        self.spi_generated = (ROOT / "Core/Src/spi.c").read_text(
            encoding="utf-8"
        )

    def test_crc_reference_vector_and_c_parameters_match_ccitt_false(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)
        self.assertIn("uint16_t crc = 0xffffu;", self.protocol)
        self.assertIn("^ 0x1021u", self.protocol)
        self.assertNotIn("crc >> 1", self.protocol)

    def test_protocol_constants_match_final_v1_contract(self):
        expected = {
            "FPGA_PROTOCOL_COMMAND_PREFIX": "0xa5u",
            "FPGA_PROTOCOL_COMMAND_GET_STATUS": "0x01u",
            "FPGA_PROTOCOL_COMMAND_READ_FRAME": "0x02u",
            "FPGA_PROTOCOL_COMMAND_ACK_FRAME": "0x03u",
            "FPGA_PROTOCOL_STATUS_BYTES": "16u",
            "FPGA_PROTOCOL_HEADER_BYTES": "128u",
            "FPGA_PROTOCOL_COMPONENT_BYTES": "16u",
            "FPGA_PROTOCOL_MIN_TIME_SAMPLES": "75u",
            "FPGA_PROTOCOL_MAX_TIME_SAMPLES": "3750u",
            "FPGA_PROTOCOL_SPECTRUM_COUNT": "1312u",
            "FPGA_PROTOCOL_MAX_FRAME_BYTES": "10254u",
            "FPGA_PROTOCOL_TIME_UV_PER_LSB": "10u",
            "FPGA_PROTOCOL_SPECTRUM_UV_PER_LSB": "10u",
        }
        for name, value in expected.items():
            self.assertRegex(
                self.protocol_h,
                rf"#define\s+{name}\s+{re.escape(value)}",
            )

    def test_header_offsets_and_little_endian_reads_are_explicit(self):
        for offset, field in (
            (8, "frame_length"),
            (12, "frame_seq"),
            (28, "time_sample_rate_hz"),
            (32, "time_count"),
            (34, "time_uv_per_lsb"),
            (42, "spectrum_count"),
            (48, "spectrum_uv_per_lsb"),
            (52, "vpp_uv"),
            (56, "vrms_uv"),
            (60, "dc_uv"),
            (64, "fundamental_mhz"),
            (116, "calibration_version"),
            (120, "dropped_frame_count"),
            (124, "reserved2"),
        ):
            self.assertRegex(
                self.protocol,
                rf"parsed\.{field}\s*=\s*fpga_protocol_read_[ui]\d+_le"
                rf"\(&frame\[{offset}\]\)",
            )
        self.assertNotIn("(fpga_protocol_frame_header_t *)", self.protocol)
        self.assertIn("component->fft_delta_q15", self.protocol)
        self.assertIn("* FPGA_PROTOCOL_COMPONENT_BYTES", self.protocol)

    def test_component_slots_are_independent_and_packed_for_hmi(self):
        self.assertIn(
            "uint8_t valid_component_count = 0u;", self.protocol
        )
        self.assertIn(
            "valid_component_count != parsed.component_count",
            self.protocol,
        )
        self.assertNotIn(
            "component_index < parsed.component_count", self.protocol
        )
        self.assertIn(
            "uint8_t output_component_index = 0u;", self.conversion
        )
        self.assertIn(
            "target->component[output_component_index] = *component;",
            self.conversion,
        )
        self.assertIn(
            "target->component_count = output_component_index;",
            self.conversion,
        )
        expected_popcounts = {
            0b000: 0,
            0b001: 1,
            0b010: 1,
            0b100: 1,
            0b011: 2,
            0b101: 2,
            0b110: 2,
            0b111: 3,
        }
        for valid_pattern, expected_count in expected_popcounts.items():
            flags = tuple(
                0x01 if valid_pattern & (1 << index) else 0x00
                for index in range(3)
            )
            self.assertEqual(
                sum(bool(value & 0x01) for value in flags),
                expected_count,
            )

    def test_all_eight_component_valid_patterns_execute_in_c_parser(self):
        compiler = shutil.which("gcc")
        if compiler is None:
            self.skipTest("host gcc is not installed")

        with tempfile.TemporaryDirectory() as temporary_directory:
            executable = (
                Path(temporary_directory) / "fpga_protocol_slots_test.exe"
            )
            build = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(ROOT / "Core/User"),
                    str(
                        ROOT
                        / "tests/fpga_protocol_component_slots_host_test.c"
                    ),
                    str(ROOT / "Core/User/fpga_protocol.c"),
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                build.returncode,
                0,
                msg=build.stdout + build.stderr,
            )
            run = subprocess.run(
                [str(executable)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(
                run.returncode,
                0,
                msg=run.stdout + run.stderr,
            )

    def test_latest_frozen_protocol_reference_vectors(self):
        sequence = 0x01020304
        frame_length = 128 + 75 * 2 + 1312 * 2 + 2
        valid_status = bytearray.fromhex(
            "5A A5 01 01 04 03 02 01 58 0B 00 00 00 00"
        )
        valid_status += struct.pack(
            "<H", crc16_ccitt_false(valid_status)
        )
        self.assertEqual(
            valid_status.hex(" ").upper(),
            "5A A5 01 01 04 03 02 01 58 0B 00 00 00 00 2E 7C",
        )

        status = bytearray.fromhex(
            "5A A5 01 11 04 03 02 01 58 0B 00 00 00 00"
        )
        status += struct.pack("<H", crc16_ccitt_false(status))
        self.assertEqual(
            status.hex(" ").upper(),
            "5A A5 01 11 04 03 02 01 58 0B 00 00 00 00 13 29",
        )

        ack = bytearray.fromhex("A5 03") + struct.pack("<I", sequence)
        ack += struct.pack("<H", crc16_ccitt_false(ack))
        self.assertEqual(
            ack.hex(" ").upper(),
            "A5 03 04 03 02 01 09 A7",
        )
        self.assertEqual(frame_length, 2904)

    def test_same_cs_immediate_response_is_used_for_status_and_frame(self):
        status_start = self.link.index("static fpga_protocol_result_t")
        status_end = self.link.index(
            "static uint8_t fpga_link_start_frame_dma", status_start
        )
        status_body = self.link[status_start:status_end]
        self.assertLess(
            status_body.index("fpga_link_cs_low();"),
            status_body.index("HAL_SPI_TransmitReceive("),
        )
        self.assertLess(
            status_body.index("HAL_SPI_TransmitReceive("),
            status_body.index("fpga_link_cs_high();"),
        )
        self.assertIn("uint8_t command_rx[2];", status_body)
        frame_start = self.link.index(
            "static uint8_t fpga_link_start_frame_dma"
        )
        frame_end = self.link.index(
            "static uint8_t fpga_link_send_ack", frame_start
        )
        frame_body = self.link[frame_start:frame_end]
        self.assertIn("HAL_SPI_TransmitReceive(", frame_body)
        self.assertIn("uint8_t command_rx[2];", frame_body)
        self.assertIn("HAL_SPI_TransmitReceive_DMA", self.link)
        self.assertIn("fpga_link_dummy_tx_cache_line", self.link)

    def test_spi3_is_generated_for_625khz_mode0_soft_nss(self):
        for setting in (
            "SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_128",
            "SPI3.CalculateBaudRate=625.0 KBits/s",
            "SPI3.DataSize=SPI_DATASIZE_8BIT",
            "SPI3.Direction=SPI_DIRECTION_2LINES",
            "SPI3.Mode=SPI_MODE_MASTER",
            "SPI3.NSSPMode=SPI_NSS_PULSE_DISABLE",
        ):
            self.assertIn(setting, self.ioc)
        for setting in (
            "hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;",
            "hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;",
            "hspi3.Init.NSS = SPI_NSS_SOFT;",
            "hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;",
            "hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;",
        ):
            self.assertIn(setting, self.spi_generated)
        self.assertIn(
            "#define FPGA_LINK_DMA_TIMEOUT_MS        200u",
            self.link,
        )

    def test_idle_status_can_use_zero_length_but_ready_status_cannot(self):
        self.assertIn(
            "parsed.state & FPGA_PROTOCOL_STATUS_FRAME_READY",
            self.protocol,
        )
        self.assertIn(
            "parsed.frame_length > FPGA_PROTOCOL_MAX_FRAME_BYTES",
            self.protocol,
        )
        status_without_frame = bytearray.fromhex(
            "5A A5 01 00 00 00 00 00 00 00 00 00 00 00"
        )
        status_without_frame += struct.pack(
            "<H", crc16_ccitt_false(status_without_frame)
        )
        self.assertEqual(len(status_without_frame), 16)
        self.assertEqual(status_without_frame[3], 0)
        self.assertEqual(struct.unpack_from("<I", status_without_frame, 8)[0], 0)

    def test_spi_dma_buffers_are_static_aligned_and_cache_maintained(self):
        self.assertIn("__attribute__((aligned(32)))", self.link)
        self.assertIn("SCB_CleanInvalidateDCache_by_Addr", self.link)
        self.assertIn("SCB_InvalidateDCache_by_Addr", self.link)
        self.assertIn("fpga_link_snapshots[2]", self.link)
        self.assertNotIn("malloc(", self.link)

    def test_crc_failure_retries_without_ack_and_valid_frame_is_acked(self):
        self.assertIn("FPGA_LINK_MAX_READ_RETRIES      3u", self.link)
        error_start = self.link.index("if (result != FPGA_PROTOCOL_OK)")
        ack_start = self.link.index(
            "if (fpga_link_send_ack(header.frame_seq) == 0u)"
        )
        self.assertLess(error_start, ack_start)
        error_block = self.link[error_start:ack_start]
        self.assertNotIn("fpga_link_send_ack", error_block)
        self.assertIn("FPGA_PROTOCOL_STATUS_RESULT_INVALID", self.link)
        self.assertIn("FPGA_PROTOCOL_HEADER_MEASUREMENT_VALID", self.link)
        self.assertIn("result_invalid_count++", self.link)

    def test_ack_waits_for_observed_data_ready_low(self):
        self.assertIn(
            "FPGA_LINK_STATE_WAIT_DATA_READY_LOW", self.link_h
        )
        self.assertIn(
            "FPGA_LINK_ACK_READY_LOW_TIMEOUT_MS 10u", self.link
        )
        self.assertIn(
            "fpga_link_observe_ready_low_fast()", self.link
        )
        self.assertIn(
            "ack_ready_low_count++", self.link
        )
        self.assertIn(
            "ack_ready_low_timeout_count++", self.link
        )
        ack_call = self.link.index(
            "if (fpga_link_send_ack(header.frame_seq) == 0u)"
        )
        wait_state = self.link.index(
            "FPGA_LINK_STATE_WAIT_DATA_READY_LOW", ack_call
        )
        self.assertLess(ack_call, wait_state)
        self.assertIn(
            "fpga_link_next_attempt_ms = now;",
            self.link[wait_state:],
        )

    def test_spi_callbacks_only_set_their_one_event_flag(self):
        expected = (
            ("HAL_GPIO_EXTI_Callback", "fpga_data_ready_flag"),
            ("HAL_SPI_TxRxCpltCallback", "fpga_spi_dma_complete_flag"),
            ("HAL_SPI_ErrorCallback", "fpga_spi_dma_error_flag"),
        )
        for function_name, flag_name in expected:
            body = function_body(self.link, function_name)
            assignments = re.findall(
                r"\b(fpga_[a-z0-9_]+)\s*=", body
            )
            self.assertEqual(assignments, [flag_name])
            self.assertNotIn("fpga_protocol_parse", body)

    def test_conversion_outputs_three_350_point_buffers(self):
        self.assertRegex(
            self.conversion_h,
            r"MEASUREMENT_DISPLAY_POINT_COUNT\s+350u",
        )
        self.assertRegex(
            self.conversion_h,
            r"MEASUREMENT_DISPLAY_Y_MIN\s+8u",
        )
        self.assertRegex(
            self.conversion_h,
            r"MEASUREMENT_DISPLAY_Y_MAX\s+201u",
        )
        for name in (
            "waveform_1cycle",
            "waveform_3cycle",
            "spectrum_display",
        ):
            self.assertIn(
                f"{name}[MEASUREMENT_DISPLAY_POINT_COUNT]",
                self.conversion_h,
            )
        self.assertIn("bucket_maximum", self.conversion)
        self.assertIn("sum += source[input_index]", self.conversion)
        self.assertNotIn("sqrt", self.conversion)

    def test_chart_buffer_covers_worst_case_350_ascii_add_commands(self):
        capacity = int(
            re.search(
                r"HMI_CHART_FRAME_MAX_BYTES\s+(\d+)u", self.chart_h
            ).group(1)
        )
        worst_case = len("cle s_spec.id,0") + 3
        worst_case += 350 * (len("add s_spec.id,0,255") + 3)
        self.assertGreaterEqual(capacity, worst_case)
        self.assertIn('"cle %s.id,0"', self.chart)
        self.assertIn('"add %s.id,0,%u"', self.chart)
        self.assertNotIn("addt", self.chart)

    def test_no_legacy_fpga_uart_protocol_remains_in_active_link(self):
        for legacy in (
            "HAL_UART",
            "AA 55",
            "fpga_link_bind_uart",
            "fpga_link_send_step_increase",
        ):
            self.assertNotIn(legacy, self.link)
            self.assertNotIn(legacy, self.link_h)


if __name__ == "__main__":
    unittest.main()
