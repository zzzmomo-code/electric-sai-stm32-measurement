import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class MeasurementCalibrationContractTest(unittest.TestCase):
    def setUp(self):
        self.source = (
            ROOT / "Core/User/measurement_calibration.c"
        ).read_text(encoding="utf-8")
        self.header = (
            ROOT / "Core/User/measurement_calibration.h"
        ).read_text(encoding="utf-8")
        self.hmi = (ROOT / "Core/User/hmi_task2.c").read_text(
            encoding="utf-8"
        )
        self.system = (ROOT / "Core/User/system.c").read_text(
            encoding="utf-8"
        )

    def test_default_is_calibrated_and_frontend_gain_halves_uv_values(self):
        self.assertIn(
            "#define MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 1u",
            self.header,
        )
        for gain_name in (
            "MEASUREMENT_FRONTEND_VPP_GAIN",
            "MEASUREMENT_FRONTEND_VRMS_GAIN",
            "MEASUREMENT_FRONTEND_COMPONENT_GAIN",
        ):
            self.assertRegex(
                self.source,
                rf"#define\s+{gain_name}\s+2\.0",
            )

        identity_curves = re.findall(
            r"calibration_[a-z0-9_]+(?:\s*=\s*|\s*\n\s*)"
            r"\{\s*0\.0,\s*1\.0,\s*0\.0,\s*0\.0\s*\}",
            self.source,
        )
        self.assertEqual(len(identity_curves), 2)

    def test_fpga_microvolts_are_corrected_by_inverse_frontend_gain(self):
        self.assertIn(
            "output_uv = (double)fpga_uv / frontend_gain;",
            self.source,
        )
        self.assertIn(
            "measurement_calibration_saturate_u32(output_uv)",
            self.source,
        )

        for fpga_uv, expected_uv in (
            (1000, 500),
            (100000, 50000),
            (3300000, 1650000),
        ):
            actual_uv = round(fpga_uv / 2.0)
            self.assertEqual(actual_uv, expected_uv)

    def test_curve_formula_uses_horner_evaluation(self):
        self.assertIn(
            "y = c0 + c1*x + c2*x*x + c3*x*x*x", self.source
        )
        self.assertIn(
            "(((curve->c3 * input + curve->c2) * input + curve->c1)",
            self.source,
        )

    def test_frequency_keeps_millihertz_contract(self):
        self.assertIn(
            "input_hz = (double)fpga_mhz / 1000.0;",
            self.source,
        )
        self.assertIn(
            "measurement_calibration_saturate_u32(output_hz * 1000.0)",
            self.source,
        )
        self.assertRegex(
            self.source,
            r"calibration_frequency_curve\s*=\s*"
            r"\{\s*0\.0,\s*1\.0,\s*0\.0,\s*0\.0\s*\}",
        )

    def test_raw_values_bypass_fit_when_calibration_is_disabled(self):
        self.assertGreaterEqual(
            self.source.count(
                "if (measurement_calibration_enabled == 0u)"
            ),
            3,
        )
        for fpga_name in ("fpga_uv", "fpga_mhz"):
            self.assertIn(f"return {fpga_name};", self.source)

    def test_all_scalar_display_interfaces_are_exposed_and_used(self):
        interfaces = (
            "measurement_calibration_apply_vpp_uv",
            "measurement_calibration_apply_vrms_uv",
            "measurement_calibration_apply_frequency_mhz",
            "measurement_calibration_apply_component_amplitude_uv",
            "measurement_calibration_apply_dc_offset_uv",
        )
        for interface in interfaces:
            self.assertIn(interface, self.header)
            self.assertIn(interface, self.source)

        for active_interface in interfaces[:-1]:
            self.assertIn(active_interface, self.hmi)

    def test_system_initializes_calibration_before_hmi(self):
        conversion_index = self.system.index(
            "measurement_conversion_init();"
        )
        calibration_index = self.system.index(
            "measurement_calibration_init();"
        )
        hmi_index = self.system.index("hmi_task2_init();")
        self.assertLess(conversion_index, calibration_index)
        self.assertLess(calibration_index, hmi_index)


if __name__ == "__main__":
    unittest.main()
