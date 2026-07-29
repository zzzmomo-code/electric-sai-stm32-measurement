import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, function_name: str) -> str:
    """Return the first brace-delimited function body for simple callback checks."""
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


class HmiRuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / "Core/User/hmi_task2.c").read_text(
            encoding="utf-8"
        )
        self.header = (ROOT / "Core/User/hmi_task2.h").read_text(
            encoding="utf-8"
        )
        self.system = (ROOT / "Core/User/system.c").read_text(
            encoding="utf-8"
        )

    def test_hmi_uses_dma_and_cache_safe_static_buffers(self):
        self.assertIn("HAL_UARTEx_ReceiveToIdle_DMA", self.source)
        self.assertIn("HAL_UART_Transmit_DMA", self.source)
        self.assertIn("__attribute__((aligned(32)))", self.source)
        self.assertIn("SCB_CleanDCache_by_Addr", self.source)
        self.assertIn("SCB_InvalidateDCache_by_Addr", self.source)
        self.assertNotIn("malloc(", self.source)

    def test_page_contract_contains_three_curves_and_parameter_texts(self):
        chart_h = (ROOT / "Core/User/hmi_chart.h").read_text(encoding="utf-8")
        for object_name in ("s_t1", "s_t3", "s_spec"):
            self.assertIn(f'"{object_name}"', chart_h)
        for text_name in ("t_vpp", "t_vrms", "t_freq", "t_status"):
            self.assertIn(text_name, self.source)
        self.assertIn('"t_comp%u.txt=', self.source)

    def test_buttons_use_a5_command_5a_and_only_change_display_mode(self):
        self.assertIn("#define HMI_TASK2_COMMAND_HEAD        0xa5u", self.source)
        self.assertIn("#define HMI_TASK2_COMMAND_TAIL        0x5au", self.source)
        self.assertIn("hmi_task2_display_requested = 1u;", self.source)
        self.assertIn("hmi_task2_visibility_pending = 1u;", self.source)
        self.assertIn("hmi_chart_build_visibility", self.source)
        self.assertNotIn("frequency_measure_request_now", self.source)
        self.assertNotIn("dac_output_set_level", self.source)

    def test_power_on_preloads_but_stays_hidden_until_a_button(self):
        self.assertIn("hmi_task2_visibility_pending = 0u;", self.source)
        self.assertIn("hmi_task2_display_requested = 0u;", self.source)
        self.assertIn(
            "if ((hmi_task2_display_requested != 0u)",
            self.source,
        )

    def test_uart_callbacks_only_set_one_shared_value(self):
        expected = (
            ("HAL_UARTEx_RxEventCallback", "hmi_uart_rx_event_size"),
            ("HAL_UART_TxCpltCallback", "hmi_uart_tx_complete_flag"),
            ("HAL_UART_ErrorCallback", "hmi_uart_error_flag"),
        )
        for function_name, variable_name in expected:
            body = function_body(self.source, function_name)
            assignments = re.findall(
                r"\b(hmi_[a-z0-9_]+)\s*=", body
            )
            self.assertEqual(assignments, [variable_name])
            self.assertNotIn("HAL_UART_", body)

    def test_preload_tracks_each_curve_sequence_and_finishes_before_refresh(self):
        self.assertIn("hmi_task2_loaded_sequence[4]", self.source)
        self.assertIn("hmi_task2_loaded_valid[4]", self.source)
        self.assertIn("hmi_task2_preload_complete()", self.source)
        self.assertIn("hmi_task2_work_snapshot", self.source)
        self.assertIn(
            "&& (hmi_task2_preload_complete() == 0u)",
            self.source,
        )

    def test_system_pipeline_is_fpga_then_conversion_then_hmi(self):
        fpga_index = self.system.index("fpga_link_process();")
        conversion_index = self.system.index(
            "measurement_conversion_update(fpga_snapshot)"
        )
        hmi_index = self.system.index("hmi_task2_process();")
        self.assertLess(fpga_index, conversion_index)
        self.assertLess(conversion_index, hmi_index)
        for legacy_process in (
            "adc_dual_process();",
            "measurement_fft_process();",
            "frequency_measure_process();",
            "dds_control_process();",
        ):
            self.assertNotIn(legacy_process, self.system)

    def test_normal_mode_keeps_chart_self_test_disabled(self):
        self.assertIn("#define HMI_CHART_SELF_TEST_ENABLE 0u", self.system)
        self.assertIn("hmi_task2_set_chart_self_test(1u);", self.system)


if __name__ == "__main__":
    unittest.main()
