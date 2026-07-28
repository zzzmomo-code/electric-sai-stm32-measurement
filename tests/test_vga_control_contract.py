from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class VgaControlContractTest(unittest.TestCase):
    def test_module_exposes_six_vca821_voltage_levels_and_limits(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
        command_voltages = (
            "0.617f",
            "0.717f",
            "0.817f",
            "0.917f",
            "1.017f",
            "1.117f",
        )
        for index, voltage in enumerate(command_voltages):
            self.assertIn(
                f"#define VGA_CONTROL_LEVEL_{index}_VOLTAGE_V {voltage}",
                header,
            )
            self.assertIn(
                f"#define VGA_CONTROL_LEVEL_{index}_MEASURED_VOLTAGE_V "
                f"{voltage}",
                header,
            )
        self.assertIn(
            "#define VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V 3.30f", header
        )
        self.assertIn("#define VGA_CONTROL_VCA821_CONTROL_MIN_V 0.617f", header)
        self.assertIn("#define VGA_CONTROL_VCA821_CONTROL_MAX_V 1.220f", header)
        self.assertIn("#define VGA_CONTROL_VCA821_GAIN_MIN_DB 0.0f", header)
        self.assertIn("#define VGA_CONTROL_VCA821_GAIN_MAX_DB 20.0f", header)
        self.assertNotRegex(header, r"LEVEL_[0-5]_CODE")

    def test_dac_codes_and_vca821_piecewise_gain_model(self):
        command_voltages = (0.617, 0.717, 0.817, 0.917, 1.017, 1.117)
        expected_codes = (766, 890, 1014, 1138, 1262, 1386)
        expected_gain_db = (
            0.664207,
            6.918506,
            13.172806,
            16.408623,
            18.659508,
            19.525459,
        )
        expected_gain = (
            1.079469,
            2.217815,
            4.556594,
            6.613497,
            8.569893,
            9.468320,
        )

        for index, command_voltage in enumerate(command_voltages):
            code = int(command_voltage / 3.30 * ((1 << 12) - 1) + 0.5)
            control_mv = command_voltage * 1000.0
            if index <= 2:
                gain_db = (control_mv - 606.38) / 15.989
            elif index == 3:
                gain_db = (control_mv - 247.2) / 40.82
            else:
                gain_db = (control_mv + 1137.8) / 115.48
            gain_db = min(20.0, max(0.0, gain_db))
            gain = 10.0 ** (gain_db / 20.0)
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(expected_gain_db[index], gain_db, places=6)
            self.assertAlmostEqual(expected_gain[index], gain, places=6)

    def test_dac_write_and_gain_model_use_calibrated_control_voltage(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        set_body = source.split(
            "vga_control_status_t vga_control_set_level", 1
        )[1].split("vga_control_status_t vga_control_gain_from_level", 1)[0]
        gain_body = source.split(
            "vga_control_status_t vga_control_gain_from_level", 1
        )[1]

        self.assertIn(
            "vga_control_voltage_to_dac_code(dac_voltage_v)", set_body
        )
        self.assertIn(
            "vga_control_gain_db_from_voltage(level, measured_voltage_v)",
            set_body,
        )
        self.assertIn(
            "vga_control_diagnostics.measured_voltage_v = measured_voltage_v;",
            set_body,
        )
        self.assertIn(
            "vga_control_gain_db_from_voltage(level, measured_voltage_v)",
            gain_body,
        )
        for level in range(6):
            measured_macro = (
                f"VGA_CONTROL_LEVEL_{level}_MEASURED_VOLTAGE_V"
            )
            self.assertIn(measured_macro, set_body)
            self.assertIn(measured_macro, gain_body)

    def test_both_level_functions_use_switch_and_default(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        self.assertGreaterEqual(source.count("switch (level)"), 2)
        for level in range(6):
            self.assertGreaterEqual(source.count(f"case {level}u:"), 2)
        self.assertGreaterEqual(source.count("default:"), 2)

    def test_on_chip_dac_uses_automatic_12bit_right_aligned_code(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        self.assertIn("(1u << 12u) - 1u", source)
        self.assertIn("DAC_ALIGN_12B_R", source)
        self.assertIn("HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)", source)
        self.assertIn(
            "HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code)",
            source,
        )

    def test_invalid_set_level_returns_before_hal_write(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        body = source.split("vga_control_status_t vga_control_set_level", 1)[1]
        switch_body = body.split("HAL_DAC_SetValue", 1)[0]
        default_body = switch_body.rsplit("default:", 1)[1]
        self.assertIn(
            "vga_control_diagnostics.last_status = "
            "vga_control_status_invalid_level;",
            default_body,
        )
        self.assertIn("return vga_control_status_invalid_level;", default_body)

    def test_init_preloads_zero_before_starting_dac(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        init_body = source.split("void vga_control_init(void)", 1)[1].split(
            "vga_control_status_t vga_control_set_level", 1
        )[0]
        preload = init_body.index(
            "HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0u)"
        )
        start = init_body.index("HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)")
        self.assertLess(preload, start)

    def test_public_api_and_diagnostics_are_declared(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
        self.assertIn("float measured_voltage_v;", header)
        self.assertIn("float gain_db;", header)
        self.assertIn("void vga_control_init(void);", header)
        self.assertIn(
            "vga_control_status_t vga_control_set_level(uint8_t level);", header
        )
        self.assertIn(
            "vga_control_status_t vga_control_gain_from_level(uint8_t level, "
            "float *gain);",
            header,
        )
        self.assertIn(
            "extern vga_control_diagnostics_t vga_control_diagnostics;", header
        )

    def test_system_header_is_the_unified_dac_and_vga_entry(self):
        header = (ROOT / "Core/User/system.h").read_text(encoding="utf-8")
        self.assertIn('#include "dac.h"', header)
        self.assertIn('#include "vga_control.h"', header)

    def test_system_initializes_vga_after_cubemx_initializes_dac(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        system = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")
        self.assertLess(main.index("MX_DAC1_Init();"), main.index("system_init();"))
        init_body = re.search(
            r"void system_init\(void\)\s*\{(?P<body>.*?)\n\}",
            system,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(init_body)
        self.assertIn("vga_control_init();", init_body.group("body"))

    def test_cubemx_dac_dependency_is_present(self):
        dac_header = ROOT / "Core/Inc/dac.h"
        dac_source = ROOT / "Core/Src/dac.c"
        self.assertTrue(dac_header.exists())
        self.assertTrue(dac_source.exists())

        ioc = (ROOT / "h743_pre1.ioc").read_text(encoding="utf-8")
        hal_config = (ROOT / "Core/Inc/stm32h7xx_hal_conf.h").read_text(
            encoding="utf-8"
        )
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        self.assertIn("DAC1.DAC_Channel-DAC_OUT1=DAC_CHANNEL_1", ioc)
        self.assertIn("PA4.Signal=COMP_DAC11_group", ioc)
        self.assertIn("#define HAL_DAC_MODULE_ENABLED", hal_config)
        self.assertIn('#include "dac.h"', main)
        self.assertLess(main.index("MX_DAC1_Init();"), main.index("system_init();"))

    def test_main_user_regions_remain_thin(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        init_region = main.split("/* USER CODE BEGIN 2 */", 1)[1].split(
            "/* USER CODE END 2 */", 1
        )[0]
        loop_region = main.split("/* USER CODE BEGIN 3 */", 1)[1].split(
            "/* USER CODE END 3 */", 1
        )[0]
        init_statements = [
            line.strip()
            for line in init_region.splitlines()
            if line.strip().endswith(";")
        ]
        loop_statements = [
            line.strip()
            for line in loop_region.splitlines()
            if line.strip().endswith(";")
        ]
        self.assertEqual(
            ["system_init();"],
            init_statements,
        )
        self.assertEqual(
            ["system_process();"],
            loop_statements,
        )

    def test_readme_documents_dac_vga_control(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        required = (
            "PA4 / DAC1_OUT1",
            "PC4 / ADC1_INP4",
            "PB1 / ADC2_INP5",
            "DAC_TRIGGER_NONE",
            "DAC_OUTPUTBUFFER_ENABLE",
            "vga_control_set_level",
            "vga_control_gain_from_level",
            "VCA821",
            "0.617、0.717、0.817、0.917、1.017、1.117 V",
            "PA4 实测电压",
            "VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V",
            "measured_voltage_v",
            "gain_db",
        )
        for text in required:
            self.assertIn(text, readme)


if __name__ == "__main__":
    unittest.main()
