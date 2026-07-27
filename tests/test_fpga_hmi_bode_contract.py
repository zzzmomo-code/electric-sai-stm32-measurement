import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class FpgaHmiBodeContractTest(unittest.TestCase):
    def setUp(self):
        self.fpga = (ROOT / "Core/User/fpga_link.c").read_text(encoding="utf-8")
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
        capacity_match = re.search(
            r"HMI_CHART_FRAME_SIZE_PER_COMPONENT\s+(\d+)u",
            self.chart_h,
        )
        self.assertIsNotNone(capacity_match)
        capacity = int(capacity_match.group(1))
        worst_case_bytes = len("cle s0.id,0") + 3
        worst_case_bytes += 64 * (len("add s0.id,0,255") + 3)
        self.assertGreaterEqual(capacity, worst_case_bytes)

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
