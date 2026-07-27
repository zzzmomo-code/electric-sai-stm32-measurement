import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class HmiRuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / "Core/User/hmi_task2.c").read_text(encoding="utf-8")
        self.header = (ROOT / "Core/User/hmi_task2.h").read_text(encoding="utf-8")
        self.system = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")

    def test_runtime_page_only_requires_power_text(self):
        self.assertIn('"t_power.txt=\\"%s\\""', self.source)
        for name in (
            "t_timer_freq", "t_adc_freq", "t_adc_amp", "t_real_freq",
            "t_real_amp", "t_wave", "t_dds_freq", "t_vga",
            "t_status", "t_overflow",
        ):
            self.assertNotIn(f'"{name}"', self.source)

    def test_buttons_map_to_vga_and_forced_measurement(self):
        self.assertIn("dac_output_set_level", self.source)
        self.assertIn("frequency_measure_request_now();", self.source)
        self.assertRegex(self.source, r"command\s*>=\s*\(uint8_t\)'0'")
        self.assertRegex(self.source, r"command\s*<=\s*\(uint8_t\)'5'")
        self.assertIn("command == (uint8_t)'M'", self.source)

    def test_uart_callbacks_only_set_one_flag(self):
        expected = (
            ("HAL_UART_RxCpltCallback", "hmi_task2_rx_flag"),
            ("HAL_UART_ErrorCallback", "hmi_task2_error_flag"),
        )
        for function_name, flag_name in expected:
            body = self.source.split(f"void {function_name}", 1)[1].split("\n}", 1)[0]
            assignments = re.findall(r"\b([a-z0-9_]+_flag)\s*=", body)
            self.assertEqual(assignments, [flag_name])
            self.assertNotIn("HAL_UART_Receive_IT", body)

    def test_runtime_process_is_called_every_loop(self):
        self.assertIn("hmi_task2_process();", self.system)
        self.assertIn("fpga_link_process();", self.system)
        self.assertLess(
            self.system.index("fpga_link_process();"),
            self.system.index("hmi_task2_process();"),
        )
        self.assertNotIn(
            "if (measurement_fft_hmi_refresh_allowed() != 0u)\n    {\n"
            "        hmi_task2_process();",
            self.system,
        )

    def test_conversion_isolated_for_future_calibration(self):
        conversion = (ROOT / "Core/User/measurement_conversion.c").read_text(
            encoding="utf-8"
        )
        self.assertIn("dds_frequency_hz + adc_frequency_hz", conversion)
        self.assertIn("return adc_amplitude_vpp;", conversion)


if __name__ == "__main__":
    unittest.main()
