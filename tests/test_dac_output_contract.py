import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


class DacOutputContractTest(unittest.TestCase):
    def test_cube_configuration_routes_dac_to_opamp_pc4(self) -> None:
        ioc = read_text("h743_pre1.ioc")
        for line in (
            "DAC1.DAC_Channel-DAC_OUT1_Int=DAC_CHANNEL_1",
            "PC4.Mode=Follower-DAC_OUT1-INP",
            "PC4.Signal=OPAMP1_VOUT",
            "VP_DAC1_VS_DACI1.Mode=DAC_OUT1_Int",
        ):
            self.assertIn(line, ioc)

    def test_hse_pll_keeps_480mhz_system_clock(self) -> None:
        main = read_text("Core/Src/main.c")
        hal_conf = read_text("Core/Inc/stm32h7xx_hal_conf.h")
        self.assertIn("#define HSE_VALUE    (25000000UL)", hal_conf)
        self.assertIn(
            "RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;", main
        )
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 5;", main)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 192;", main)
        self.assertIn("RCC_OscInitStruct.PLL.PLLP = 2;", main)
        self.assertIn("FLASH_LATENCY_4", main)

    def test_driver_exposes_levels_calibration_and_gain_api(self) -> None:
        header = read_text("Core/User/dac_output.h")
        for token in (
            "DAC_OUTPUT_LEVEL_COUNT              6u",
            "DAC_OUTPUT_REFERENCE_MV             3300u",
            "DAC_OUTPUT_RF_OHM                   10000.0f",
            "DAC_OUTPUT_RG_OHM                   10000.0f",
            "DAC_OUTPUT_GAIN_CALIBRATION         1.0f",
            "DAC_OUTPUT_OFFSET_CALIBRATION_V     0.0f",
            "dac_output_init(void)",
            "dac_output_set_level(uint8_t level)",
            "dac_output_get_voltage(uint8_t level, float *voltage_v)",
            "dac_output_get_vg(uint8_t level, float *vg)",
            "dac_output_get_vga_gain(uint8_t level, float *gain)",
        ):
            self.assertIn(token, header)

    def test_driver_starts_opamp_and_dac_and_system_initializes_it(self) -> None:
        source = read_text("Core/User/dac_output.c")
        system_h = read_text("Core/User/system.h")
        system_c = read_text("Core/User/system.c")
        self.assertIn("HAL_OPAMP_Start(&hopamp1)", source)
        self.assertIn("HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1", source)
        self.assertIn("HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)", source)
        self.assertIn('#include "dac_output.h"', system_h)
        self.assertIn("dac_output_init();", system_c)

    def test_six_default_levels_have_expected_codes_vg_and_gain(self) -> None:
        millivolts = (0, 660, 1320, 1980, 2640, 3300)
        expected_codes = (0, 819, 1638, 2457, 3276, 4095)
        for index, mv in enumerate(millivolts):
            code = (mv * 4095 + 1650) // 3300
            vg = (20.0 / 33.0) * (mv / 1000.0) - 1.0
            gain = (1.0 + vg) * (10000.0 / 10000.0)
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(index * 0.4 - 1.0, vg, places=6)
            self.assertAlmostEqual(index * 0.4, gain, places=6)

    def test_set_level_validates_before_writing_hal(self) -> None:
        source = read_text("Core/User/dac_output.c")
        validation = source.index("level >= DAC_OUTPUT_LEVEL_COUNT")
        write = source.index("HAL_DAC_SetValue")
        self.assertLess(validation, write)


if __name__ == "__main__":
    unittest.main()
