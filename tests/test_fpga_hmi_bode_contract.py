import re
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
            "FPGA_PROTOCOL_MAX_TIME_SAMPLES": "3750u",
            "FPGA_PROTOCOL_SPECTRUM_COUNT": "1312u",
            "FPGA_PROTOCOL_MAX_FRAME_BYTES": "10254u",
        }
        for name, value in expected.items():
            self.assertRegex(
                self.protocol_h,
                rf"#define\s+{name}\s+{re.escape(value)}",
            )

    def test_header_offsets_and_little_endian_reads_are_explicit(self):
        for offset, field in (
            (8, "total_bytes"),
            (12, "frame_seq"),
            (28, "time_sample_rate_hz"),
            (32, "time_count"),
            (42, "spectrum_count"),
            (52, "vpp_uv"),
            (56, "vrms_uv"),
            (60, "fundamental_mhz"),
            (108, "dropped_frames"),
        ):
            self.assertRegex(
                self.protocol,
                rf"parsed\.{field}\s*=\s*fpga_protocol_read_[ui]\d+_le"
                rf"\(&frame\[{offset}\]\)",
            )
        self.assertNotIn("(fpga_protocol_frame_header_t *)", self.protocol)

    def test_same_cs_immediate_response_is_used_for_status_and_frame(self):
        status_start = self.link.index("static fpga_protocol_result_t")
        status_end = self.link.index(
            "static uint8_t fpga_link_start_frame_dma", status_start
        )
        status_body = self.link[status_start:status_end]
        self.assertLess(
            status_body.index("fpga_link_cs_low();"),
            status_body.index("HAL_SPI_Transmit("),
        )
        self.assertLess(
            status_body.index("HAL_SPI_Transmit("),
            status_body.index("HAL_SPI_TransmitReceive("),
        )
        self.assertLess(
            status_body.index("HAL_SPI_TransmitReceive("),
            status_body.index("fpga_link_cs_high();"),
        )
        self.assertIn("HAL_SPI_TransmitReceive_DMA", self.link)
        self.assertIn("fpga_link_dummy_tx_cache_line", self.link)

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
            "(void)fpga_link_send_ack(header.frame_seq);"
        )
        self.assertLess(error_start, ack_start)
        error_block = self.link[error_start:ack_start]
        self.assertNotIn("fpga_link_send_ack", error_block)
        self.assertIn("FPGA_PROTOCOL_STATUS_RESULT_INVALID", self.link)
        self.assertIn("result_invalid_count++", self.link)

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
