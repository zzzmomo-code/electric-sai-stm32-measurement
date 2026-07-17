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


if __name__ == "__main__":
    unittest.main()
