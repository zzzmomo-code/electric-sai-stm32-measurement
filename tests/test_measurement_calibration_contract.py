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

    def test_default_is_calibrated_and_voltage_scales_use_6400_raw_per_mv(self):
        self.assertIn(
            "#define MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 1u",
            self.header,
        )
        for scale_name in (
            "MEASUREMENT_CALIBRATION_VPP_RAW_PER_MV",
            "MEASUREMENT_CALIBRATION_VRMS_RAW_PER_MV",
            "MEASUREMENT_CALIBRATION_COMPONENT_RAW_PER_MV",
        ):
            self.assertRegex(
                self.source,
                rf"#define\s+{scale_name}\s+6400\.0",
            )

        identity_curves = re.findall(
            r"calibration_[a-z0-9_]+(?:\s*=\s*|\s*\n\s*)"
            r"\{\s*0\.0,\s*1\.0,\s*0\.0,\s*0\.0\s*\}",
            self.source,
        )
        self.assertEqual(len(identity_curves), 2)

    def test_voltage_raw_codes_are_converted_to_microvolts(self):
        self.assertIn(
            "output_uv = ((double)raw_value * 1000.0) / raw_per_mv;",
            self.source,
        )
        self.assertIn(
            "measurement_calibration_saturate_u32(output_uv)",
            self.source,
        )

        # 6400 raw/mV：6400、640000、21120000 分别对应
        # 1 mV、100 mV、3.3 V，返回接口单位统一为 uV。
        for raw_value, expected_uv in (
            (6400, 1000),
            (640000, 100000),
            (21120000, 3300000),
        ):
            actual_uv = round(raw_value * 1000.0 / 6400.0)
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
            "input_hz = (double)raw_mhz / 1000.0;",
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
