from pathlib import Path
import math
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SamplingDesignTest(unittest.TestCase):
    def test_timer_generates_600ksps(self):
        timer = (ROOT / "Core/Src/tim.c").read_text(encoding="utf-8")
        self.assertIn("htim2.Init.Prescaler = 0;", timer)
        self.assertIn("htim2.Init.Period = 399;", timer)
        self.assertIn("TIM_TRGO_UPDATE", timer)

    def test_frame_math(self):
        sample_rate = 600_000.0
        length = 65_536
        self.assertAlmostEqual(length / sample_rate, 0.1092266667, places=9)
        self.assertAlmostEqual(sample_rate / length, 9.1552734375, places=9)
        self.assertEqual(length * 4, 256 * 1024)


class SourceContractTest(unittest.TestCase):
    def test_fft_module_exists_and_exposes_forward_api(self):
        header = (ROOT / "Core/User/fft_f32_65536.h").read_text(encoding="utf-8")
        source = (ROOT / "Core/User/fft_f32_65536.c").read_text(encoding="utf-8")
        self.assertIn("fft_f32_65536_forward", header)
        self.assertIn("fft_f32_65536_forward", source)
        self.assertIn("FFT_F32_65536_STATUS_NONFINITE", header)

    def test_real_fft_mirror_twiddle_sign_is_preserved(self):
        source = (ROOT / "Core/User/fft_f32_65536.c").read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"-twiddle_real,\s*twiddle_imaginary,\s*&mirror_output_real",
        )

    def test_measurement_constants_are_updated(self):
        header = (ROOT / "Core/User/measurement_fft.h").read_text(encoding="utf-8")
        self.assertIn("#define MEASUREMENT_FFT_LENGTH 65536u", header)
        self.assertIn("#define MEASUREMENT_FFT_SAMPLE_RATE_HZ 600000.0f", header)
        self.assertIn("#define MEASUREMENT_FFT_SPECTRUM_MAX_HZ 120000.0f", header)

    def test_sample_pair_and_memory_sections_exist(self):
        adc_header = (ROOT / "Core/User/adc_dual.h").read_text(encoding="utf-8")
        fft_source = (ROOT / "Core/User/measurement_fft.c").read_text(encoding="utf-8")
        linker = (ROOT / "STM32H743VITX_FLASH.ld").read_text(encoding="utf-8")
        self.assertIn("adc_dual_sample_pair_t", adc_header)
        self.assertIn('section(".adc_sample_pairs")', fft_source)
        self.assertIn('section(".fft_f32_work")', fft_source)
        self.assertIn(".adc_sample_pairs", linker)
        self.assertIn(".fft_f32_work", linker)

    def test_existing_diagnostics_fields_remain(self):
        header = (ROOT / "Core/User/measurement_fft.h").read_text(encoding="utf-8")
        required = {
            "init_status", "window_count", "discarded_sample_count", "fft_count",
            "publish_count", "last_fft_cycles", "raw_sample_rate_hz",
            "effective_sample_rate_hz", "bin_width_hz", "decimation_factor",
            "capture_resync_count", "ch1_frame_min_code", "ch1_frame_max_code",
            "ch1_frame_mean_code", "ch2_frame_min_code", "ch2_frame_max_code",
            "ch2_frame_mean_code", "peak_bin", "secondary_peak_bin",
            "peak_offset_bins", "raw_peak_frequency_hz", "peak_frequency_hz",
            "secondary_raw_peak_frequency_hz", "secondary_peak_frequency_hz",
            "amplitude_vpp", "secondary_amplitude_vpp", "dc_voltage",
            "secondary_dc_voltage", "rms_voltage", "secondary_rms_voltage",
            "thd_percent", "secondary_thd_percent", "raw_phase_deg", "phase_deg",
            "clipping_mask", "voltage_calibrated_mask", "voltage_estimated_mask",
            "quality", "result_valid",
        }
        for field in required:
            self.assertRegex(header, rf"\b{re.escape(field)}\b")

    def test_fft_frequency_calibration_formula_is_exposed(self):
        header = (ROOT / "Core/User/measurement_fft.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "Core/User/measurement_fft.c").read_text(
            encoding="utf-8"
        )
        required_header = (
            "#define MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ 40000.0f",
            "#define MEASUREMENT_FFT_LOW_FREQUENCY_GAIN 0.9999807f",
            "#define MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ (-0.2414f)",
            "#define MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN 0.99995854f",
            "#define MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ (-0.3226f)",
            "float measurement_fft_calibrate_frequency(float raw_frequency_hz);",
        )
        for text in required_header:
            self.assertIn(text, header)

        self.assertIn(
            "float measurement_fft_calibrate_frequency(float raw_frequency_hz)",
            source,
        )
        self.assertIn(
            "raw_frequency_hz <= MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ",
            source,
        )
        self.assertIn("MEASUREMENT_FFT_LOW_FREQUENCY_GAIN", source)
        self.assertIn("MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN", source)

    def test_fft_frequency_calibration_math_and_boundary(self):
        def calibrate(raw_frequency_hz):
            if raw_frequency_hz <= 0.0:
                return 0.0
            if raw_frequency_hz <= 40000.0:
                calibrated = 0.9999807 * raw_frequency_hz - 0.2414
            else:
                calibrated = 0.99995854 * raw_frequency_hz - 0.3226
            return max(calibrated, 0.0)

        self.assertEqual(0.0, calibrate(0.0))
        self.assertEqual(0.0, calibrate(-1.0))
        self.assertAlmostEqual(999.7393, calibrate(1000.0), places=4)
        self.assertAlmostEqual(39998.9866, calibrate(40000.0), places=4)
        self.assertAlmostEqual(39999.01895854, calibrate(40001.0), places=5)

    def test_both_fft_channels_preserve_raw_and_publish_calibrated_frequency(self):
        source = (ROOT / "Core/User/measurement_fft.c").read_text(
            encoding="utf-8"
        )
        required_source = (
            "measurement_fft_diagnostics.raw_peak_frequency_hz =",
            "measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =",
            "measurement_fft_calibrate_frequency(\n"
            "                measurement_fft_diagnostics.raw_peak_frequency_hz)",
            "measurement_fft_calibrate_frequency(\n"
            "                measurement_fft_diagnostics.secondary_raw_peak_frequency_hz)",
            "result.frequency_hz = measurement_fft_diagnostics.peak_frequency_hz;",
            "measurement_fft_diagnostics.secondary_peak_frequency_hz;",
        )
        for text in required_source:
            self.assertIn(text, source)

        self.assertGreaterEqual(
            source.count("raw_peak_frequency_hz = 0.0f;"), 2
        )
        self.assertGreaterEqual(
            source.count("peak_frequency_hz = 0.0f;"), 4
        )

    def test_hmi_voltage_and_vpp_do_not_read_adc_stats(self):
        source = (ROOT / "Core/User/hmi_tjc.c").read_text(encoding="utf-8")
        self.assertIn("result->dc_voltage", source)
        self.assertIn("result->secondary_dc_voltage", source)
        self.assertIn("result->amplitude_vpp", source)
        self.assertIn("result->secondary_amplitude_vpp", source)
        self.assertNotIn("adc_dual_get_stats", source)
        self.assertNotIn("adc_dual_stats_t", source)

    def test_adc_callbacks_only_set_their_flag(self):
        source = (ROOT / "Core/User/adc_dual.c").read_text(encoding="utf-8")
        expected = (
            ("HAL_ADC_ConvHalfCpltCallback", "adc_dual_dma_half_flag"),
            ("HAL_ADC_ConvCpltCallback", "adc_dual_dma_full_flag"),
            ("HAL_ADC_ErrorCallback", "adc_dual_error_flag"),
        )
        for function_name, flag_name in expected:
            body = source.split(f"void {function_name}", 1)[1].split("\n}", 1)[0]
            assigned_flags = re.findall(r"\b([a-z0-9_]+_flag)\s*=", body)
            self.assertEqual(assigned_flags, [flag_name])


if __name__ == "__main__":
    unittest.main()
