from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class VgaControlContractTest(unittest.TestCase):
    def test_module_exposes_six_voltage_levels_and_formulas(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
        for index, voltage in enumerate(
            ("0.00f", "0.66f", "1.32f", "1.98f", "2.64f", "3.30f")
        ):
            self.assertIn(
                f"#define VGA_CONTROL_LEVEL_{index}_VOLTAGE_V {voltage}",
                header,
            )
        self.assertIn(
            "#define VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V 3.30f", header
        )
        self.assertIn("#define VGA_CONTROL_VG_SCALE (20.0f / 33.0f)", header)
        self.assertIn("#define VGA_CONTROL_VG_OFFSET_V (-1.0f)", header)
        self.assertIn("#define VGA_CONTROL_RF 1.0f", header)
        self.assertIn("#define VGA_CONTROL_RG 1.0f", header)
        self.assertIn("VGA_CONTROL_VG_FROM_DAC_VOLTAGE", header)
        self.assertIn("VGA_CONTROL_GAIN_FROM_VG", header)
        self.assertIn("VGA_CONTROL_VOUT_FROM_INPUTS", header)
        self.assertNotRegex(header, r"LEVEL_[0-5]_CODE")

    def test_six_level_math_matches_the_design(self):
        voltages = (0.00, 0.66, 1.32, 1.98, 2.64, 3.30)
        expected_codes = (0, 819, 1638, 2457, 3276, 4095)
        expected_vg = (-1.0, -0.6, -0.2, 0.2, 0.6, 1.0)
        expected_gain = (0.0, 0.4, 0.8, 1.2, 1.6, 2.0)

        for index, voltage in enumerate(voltages):
            code = int(voltage / 3.30 * ((1 << 12) - 1) + 0.5)
            vg = (20.0 / 33.0) * voltage - 1.0
            gain = (1.0 + vg) * 1.0 / 1.0
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(expected_vg[index], vg, places=6)
            self.assertAlmostEqual(expected_gain[index], gain, places=6)

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
        self.assertIn("return vga_control_status_invalid_level;", default_body)

    def test_public_api_and_diagnostics_are_declared(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
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


if __name__ == "__main__":
    unittest.main()
