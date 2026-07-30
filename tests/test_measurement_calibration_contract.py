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

    def test_default_is_calibrated_and_all_initial_curves_are_identity(self):
        self.assertIn(
            "#define MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 1u",
            self.header,
        )
        curve_initializers = re.findall(
            r"calibration_[a-z0-9_]+(?:\s*=\s*|\s*\n\s*)"
            r"\{\s*0\.0,\s*1\.0,\s*0\.0,\s*0\.0\s*\}",
            self.source,
        )
        self.assertEqual(len(curve_initializers), 5)

    def test_curve_formula_uses_horner_evaluation(self):
        self.assertIn(
            "y = c0 + c1*x + c2*x*x + c3*x*x*x", self.source
        )
        self.assertIn(
            "(((curve->c3 * input + curve->c2) * input + curve->c1)",
            self.source,
        )

    def test_raw_values_bypass_fit_when_calibration_is_disabled(self):
        self.assertGreaterEqual(
            self.source.count(
                "if (measurement_calibration_enabled == 0u)"
            ),
            3,
        )
        for raw_name in ("raw_uv", "raw_mhz"):
            self.assertIn(f"return {raw_name};", self.source)

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
