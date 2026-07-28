import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FpgaHmiBodeContractTest(unittest.TestCase):
    def setUp(self):
        self.fpga = (ROOT / "Core/User/fpga_link.c").read_text(encoding="utf-8")
        self.fpga_h = (
            ROOT / "Core/User/fpga_link.h"
        ).read_text(encoding="utf-8")
        self.chart = (ROOT / "Core/User/hmi_chart.c").read_text(encoding="utf-8")
        self.chart_h = (
            ROOT / "Core/User/hmi_chart.h"
        ).read_text(encoding="utf-8")
        self.hmi = (ROOT / "Core/User/hmi_task2.c").read_text(encoding="utf-8")
        self.system = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")
        self.result_h = (
            ROOT / "Core/User/measurement_result.h"
        ).read_text(encoding="utf-8")
        self.result_c = (
            ROOT / "Core/User/measurement_result.c"
        ).read_text(encoding="utf-8")
        self.fpga_guide = (
            ROOT / "docs/FPGA_UART_PROTOCOL_GUIDE.md"
        ).read_text(encoding="utf-8")

    def test_fpga_uses_receive_to_idle_dma_and_cache_maintenance(self):
        self.assertIn("HAL_UARTEx_ReceiveToIdle_DMA", self.fpga)
        self.assertIn("SCB_CleanInvalidateDCache_by_Addr", self.fpga)
        self.assertIn("SCB_InvalidateDCache_by_Addr", self.fpga)
        self.assertIn("__attribute__((aligned(32)))", self.fpga)

    def test_rx_event_callback_only_sets_event_size(self):
        body = self.fpga.split(
            "void HAL_UARTEx_RxEventCallback", 1
        )[1].split("\n}", 1)[0]
        assignments = re.findall(
            r"\b(fpga_link_[a-z0-9_]+)\s*=", body
        )
        self.assertEqual(assignments, ["fpga_link_rx_event_size"])
        self.assertNotIn("fpga_link_parse_frame", body)

    def test_fpga_guide_matches_current_binary_contract(self):
        for contract in (
            "AA 55  N_H N_L",
            "MAG_H MAG_L PHASE_H PHASE_L",
            "`1 ≤ N ≤ 1024`",
            "`4102` 字节",
            "`1,000,000 bit/s`",
            "`50 ms`",
        ):
            self.assertIn(contract, self.fpga_guide)

    def test_chart_uses_cle_add_without_transparent_mode(self):
        self.assertIn('"cle %s,%u"', self.chart)
        self.assertIn('"add %s,%u,%u"', self.chart)
        self.assertIn('"s0.id"', self.chart)
        self.assertIn('"s1.id"', self.chart)
        self.assertNotIn("addt", self.chart)

    def test_chart_buffer_covers_worst_case_ascii_commands(self):
        point_count_match = re.search(
            r"HMI_CHART_POINT_COUNT\s+(\d+)u",
            self.chart_h,
        )
        capacity_match = re.search(
            r"HMI_CHART_FRAME_SIZE_PER_COMPONENT\s+(\d+)u",
            self.chart_h,
        )
        self.assertIsNotNone(point_count_match)
        self.assertIsNotNone(capacity_match)
        point_count = int(point_count_match.group(1))
        capacity = int(capacity_match.group(1))
        worst_case_bytes = len("cle s0.id,0") + 3
        worst_case_bytes += point_count * (len("add s0.id,0,255") + 3)
        self.assertEqual(point_count, 256)
        self.assertGreaterEqual(capacity, worst_case_bytes)

    def test_chart_uses_static_downsample_workspace(self):
        self.assertIn(
            "static uint8_t hmi_chart_amplitude_workspace"
            "[HMI_CHART_POINT_COUNT]",
            self.chart,
        )
        self.assertIn(
            "static uint8_t hmi_chart_phase_workspace"
            "[HMI_CHART_POINT_COUNT]",
            self.chart,
        )
        self.assertNotIn(
            "uint8_t amplitude[HMI_CHART_POINT_COUNT]",
            self.chart,
        )
        self.assertNotIn(
            "uint8_t phase[HMI_CHART_POINT_COUNT]",
            self.chart,
        )

    def test_chart_downsampling_uses_bucket_means(self):
        self.assertIn(
            "magnitude_sum += bode->mag2_hi[input_index]",
            self.chart,
        )
        self.assertIn(
            "phase_sum += bode->phase[input_index]",
            self.chart,
        )
        self.assertIn(
            "mean_magnitude = (uint16_t)(magnitude_sum / sample_count)",
            self.chart,
        )
        self.assertNotIn("best_magnitude", self.chart)
        self.assertNotIn("best_index", self.chart)

    def test_chart_restores_amplitude_from_mag2_high_word(self):
        self.assertIn(
            "magnitude_root = hmi_chart_isqrt_u16(mean_magnitude)",
            self.chart,
        )
        self.assertIn(
            "amplitude[output_index] = (uint8_t)magnitude_root",
            self.chart,
        )
        self.assertIn("(I²+Q²)[63:48]", self.chart)

    def test_fpga_step_commands_use_usart2_single_byte_contract(self):
        self.assertIn(
            "#define FPGA_LINK_STEP_INCREASE_COMMAND 0x2bu",
            self.fpga,
        )
        self.assertIn(
            "#define FPGA_LINK_STEP_DECREASE_COMMAND 0x2du",
            self.fpga,
        )
        self.assertIn(
            "HAL_UART_Transmit(\n"
            "        fpga_link_uart, &command, 1u,",
            self.fpga,
        )
        self.assertIn("fpga_link_send_step_increase", self.fpga_h)
        self.assertIn("fpga_link_send_step_decrease", self.fpga_h)
        self.assertIn("command_tx_count", self.fpga_h)
        self.assertIn("command_tx_error_count", self.fpga_h)
        self.assertIn("last_tx_command", self.fpga_h)

    def test_s0_autoscales_frame_minimum_and_maximum(self):
        self.assertIn(
            "uint8_t amplitude_min = HMI_CHART_VALUE_MAX",
            self.chart,
        )
        self.assertIn("uint8_t amplitude_max = 0u", self.chart)
        self.assertIn(
            "amplitude[output_index] - amplitude_min",
            self.chart,
        )
        self.assertIn("/ amplitude_range", self.chart)
        self.assertIn("amplitude[output_index] = 0u", self.chart)

    def test_bode_transmit_is_split_at_complete_command_boundaries(self):
        self.assertIn("hmi_task2_send_next_bode_command", self.hmi)
        self.assertIn("terminator_found", self.hmi)
        self.assertIn("HMI_TASK2_COMMAND_TX_TIMEOUT_MS", self.hmi)
        self.assertNotRegex(
            self.hmi,
            r"HAL_UART_Transmit\([^;]*hmi_task2_bode_frame[^;]*"
            r"hmi_task2_bode_frame_size",
        )

    def test_power_interface_is_exposed_and_persistent(self):
        self.assertIn("MEASUREMENT_VALID_POWER", self.result_h)
        self.assertIn("float power_w;", self.result_h)
        self.assertIn("measurement_result_set_power_w", self.result_h)
        self.assertIn("measurement_result_clear_power", self.result_h)
        self.assertIn("saved_power_valid", self.result_c)

    def test_chart_self_test_uses_the_production_frame_path(self):
        self.assertIn("hmi_task2_generate_chart_self_test", self.hmi)
        self.assertIn(
            "bode->point_count = HMI_CHART_POINT_COUNT",
            self.hmi,
        )
        self.assertIn("hmi_chart_build_bode_frame", self.hmi)
        self.assertIn(
            "#define HMI_CHART_SELF_TEST_ENABLE 0u",
            self.system,
        )
        self.assertIn(
            "hmi_task2_set_chart_self_test(1u);",
            self.system,
        )


if __name__ == "__main__":
    unittest.main()
