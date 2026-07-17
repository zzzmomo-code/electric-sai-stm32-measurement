from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class TimerConfigurationTest(unittest.TestCase):
    def test_tim3_generates_one_hz_and_has_low_priority(self):
        timer = (ROOT / "Core/Src/tim.c").read_text(encoding="utf-8")
        self.assertIn("htim3.Init.Prescaler = 23999;", timer)
        self.assertIn("htim3.Init.Period = 9999;", timer)
        self.assertIn("HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);", timer)
        self.assertEqual(240_000_000 // (23_999 + 1) // (9_999 + 1), 1)

    def test_tim5_counts_pa0_rising_edges_without_filter(self):
        timer = (ROOT / "Core/Src/tim.c").read_text(encoding="utf-8")
        self.assertIn("htim5.Init.Prescaler = 0;", timer)
        self.assertIn("htim5.Init.Period = 4294967295;", timer)
        self.assertIn("TIM_SLAVEMODE_EXTERNAL1", timer)
        self.assertIn("TIM_TS_TI1FP1", timer)
        self.assertIn("TIM_TRIGGERPOLARITY_RISING", timer)
        self.assertIn("sSlaveConfig.TriggerFilter = 0;", timer)
        self.assertIn("GPIO_PIN_0", timer)
        self.assertIn("GPIO_AF2_TIM5", timer)

    def test_generated_main_initializes_timers_before_system(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        order = [
            main.index("MX_TIM3_Init();"),
            main.index("MX_TIM5_Init();"),
            main.index("system_init();"),
        ]
        self.assertEqual(order, sorted(order))


class FrequencyModuleTest(unittest.TestCase):
    def test_unsigned_delta_math_handles_wrap_and_30mhz(self):
        previous_counter = 0xFFF0_0000
        counter_delta = 30_000_000
        current_counter = (previous_counter + counter_delta) & 0xFFFF_FFFF
        measured_delta = (current_counter - previous_counter) & 0xFFFF_FFFF
        cycle_delta = 480_000_000
        frequency_hz = measured_delta * 480_000_000.0 / cycle_delta
        self.assertEqual(measured_delta, counter_delta)
        self.assertAlmostEqual(frequency_hz, 30_000_000.0)

    def test_module_exposes_independent_result_and_process_api(self):
        header_path = ROOT / "Core/User/frequency_measure.h"
        self.assertTrue(header_path.exists(), "frequency_measure.h must exist")
        header = header_path.read_text(encoding="utf-8")
        self.assertIn("void frequency_measure_init(void);", header)
        self.assertIn("void frequency_measure_process(void);", header)
        self.assertIn("extern volatile uint8_t frequency_measure_flag;", header)
        self.assertIn("extern volatile float frequency_measure_hz;", header)

    def test_tim3_callback_only_assigns_its_flag(self):
        source_path = ROOT / "Core/User/frequency_measure.c"
        self.assertTrue(source_path.exists(), "frequency_measure.c must exist")
        source = source_path.read_text(encoding="utf-8")
        body = source.split("void HAL_TIM_PeriodElapsedCallback", 1)[1].split("\n}", 1)[0]
        assigned_flags = re.findall(r"\b([a-z0-9_]+_flag)\s*=", body)
        self.assertEqual(assigned_flags, ["frequency_measure_flag"])
        self.assertNotIn("frequency_measure_process();", body)
        self.assertNotIn("__HAL_TIM_GET_COUNTER", body)

    def test_runtime_uses_counter_and_dwt_deltas_without_resetting_dwt(self):
        source_path = ROOT / "Core/User/frequency_measure.c"
        self.assertTrue(source_path.exists(), "frequency_measure.c must exist")
        source = source_path.read_text(encoding="utf-8")
        self.assertIn("__HAL_TIM_GET_COUNTER(&htim5)", source)
        self.assertIn("DWT->CYCCNT", source)
        self.assertIn("SystemCoreClock", source)
        self.assertNotIn("DWT->CYCCNT = 0", source)


class SystemIntegrationTest(unittest.TestCase):
    def test_system_header_is_the_unified_entry(self):
        header = (ROOT / "Core/User/system.h").read_text(encoding="utf-8")
        self.assertIn('#include "frequency_measure.h"', header)

    def test_system_initializes_and_processes_frequency_module(self):
        source = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")
        init_body = source.split("void system_init(void)", 1)[1].split("\n}", 1)[0]
        process_body = source.split("void system_process(void)", 1)[1].split("\n}", 1)[0]
        self.assertIn("frequency_measure_init();", init_body)
        self.assertIn("frequency_measure_process();", process_body)

    def test_external_frequency_does_not_replace_fft_result(self):
        source = (ROOT / "Core/User/frequency_measure.c").read_text(encoding="utf-8")
        self.assertNotIn("measurement_result_publish", source)
        self.assertNotIn("measurement_result_t", source)


if __name__ == "__main__":
    unittest.main()
