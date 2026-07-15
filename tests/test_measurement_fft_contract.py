"""检查双通道 FFT、时域统计、频谱和结果发布之间的源码契约。"""

from pathlib import Path
import re
import unittest


project_root = Path(__file__).resolve().parents[1]
user_dir = project_root / "Core" / "User"


class MeasurementFftContractTest(unittest.TestCase):
    """防止 CubeMX 重新生成或后续重构破坏已经确认的数据边界。"""

    def test_q15_fft_output_buffer_has_required_double_length(self):
        """Q15 复数 FFT 的交错实虚部输出必须为输入长度的两倍。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        self.assertRegex(
            header,
            r"#define\s+MEASUREMENT_FFT_LENGTH\s+8192u",
        )
        self.assertRegex(
            header,
            (
                r"#define\s+MEASUREMENT_FFT_OUTPUT_LENGTH\s+"
                r"\(\s*2u\s*\*\s*MEASUREMENT_FFT_LENGTH\s*\)"
            ),
        )
        self.assertRegex(
            source,
            (
                r"measurement_fft_output\s*"
                r"\[\s*MEASUREMENT_FFT_CHANNEL_COUNT\s*\]\s*"
                r"\[\s*MEASUREMENT_FFT_OUTPUT_LENGTH\s*\]"
            ),
        )

    def test_q15_fft_uses_runtime_tables_for_128k_flash(self):
        """8192 点 FFT 必须在 RAM 生成旋转因子，不得依赖超容量静态表。"""
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        self.assertIn("measurement_fft_initialize_twiddle();", source)
        self.assertIn("measurement_fft_execute_q15", source)
        self.assertNotIn("arm_rfft_init_q15", source)
        self.assertNotIn("arm_rfft_q15(&measurement_fft_instance", source)

    def test_fft_publishes_only_through_measurement_result(self):
        """FFT 模块只接收同步样本对并通过结果快照发布。"""
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        self.assertIn("measurement_result_publish(&result);", source)
        self.assertIn("measurement_fft_ingest_pair", source)
        self.assertNotIn("HAL_UART_", source)
        self.assertNotIn("hmi_tjc_", source)
        self.assertNotIn("ads8688_dma_rx", source)
        self.assertNotIn("ads8688_get_channel_range", source)
        self.assertNotIn("ads8688_convert_raw_to_voltage", source)
        self.assertNotIn("MEASUREMENT_FFT_INTERCHANNEL_DELAY_S", source)

    def test_fft_keeps_quality_and_debug_observability(self):
        """上板时需要的直流、幅频、THD、相位和采样率诊断字段必须保留。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        required_names = (
            "MEASUREMENT_FFT_QUALITY_SIGNAL_TOO_SMALL",
            "MEASUREMENT_FFT_QUALITY_CLIPPED",
            "MEASUREMENT_FFT_QUALITY_CHANNEL_MISMATCH",
            "MEASUREMENT_FFT_QUALITY_DC_INPUT",
            "raw_sample_rate_hz",
            "effective_sample_rate_hz",
            "bin_width_hz",
            "decimation_factor",
            "capture_resync_count",
            "ch1_frame_min_code",
            "ch1_frame_max_code",
            "ch1_frame_mean_code",
            "ch2_frame_min_code",
            "ch2_frame_max_code",
            "ch2_frame_mean_code",
            "peak_frequency_hz",
            "secondary_peak_frequency_hz",
            "amplitude_vpp",
            "secondary_amplitude_vpp",
            "dc_voltage",
            "rms_voltage",
            "thd_percent",
            "thd_harmonic_count",
            "raw_phase_deg",
            "phase_deg",
            "harmonic_ratio_3",
            "harmonic_ratio_5",
            "clipping_mask",
            "result_valid",
            "voltage_calibrated_mask",
        )

        for name in required_names:
            with self.subTest(name=name):
                self.assertIn(name, header)

    def test_fft_exposes_capture_resynchronization_entrypoint(self):
        """DMA 顺序失去可信度时应能放弃当前窗口并重新同步。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        self.assertIn("void measurement_fft_resynchronize(void);", header)
        self.assertIn("void measurement_fft_resynchronize(void)", source)
        self.assertIn("measurement_fft_diagnostics.capture_resync_count++", source)

    def test_fixed_80ksps_and_spectrum_contract_is_declared(self):
        """固定80 kSPS应达到10 Hz内频点间隔，并保留64点频谱接口。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        for token in (
            "MEASUREMENT_FFT_RAW_SAMPLE_RATE_HZ 80000.0f",
            "MEASUREMENT_FFT_DECIMATION_FACTOR 1u",
            "measurement_fft_update_timing_diagnostics",
        ):
            with self.subTest(token=token):
                self.assertIn(token, source)

        self.assertNotIn("measurement_fft_select_decimation", source)
        self.assertNotIn("measurement_fft_decimation_count", source)

        self.assertRegex(
            header,
            r"#define\s+MEASUREMENT_FFT_SPECTRUM_POINT_COUNT\s+64u",
        )
        self.assertIn("measurement_fft_spectrum_t", header)
        self.assertIn("measurement_fft_get_spectrum", header)
        self.assertIn("measurement_fft_build_spectrum", source)

    def test_measurement_result_supports_partial_valid_fields(self):
        """AIN1 缺失时 AIN0 结果仍须通过字段有效位发布。"""
        header = (user_dir / "measurement_result.h").read_text(encoding="utf-8")
        fft_source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")
        hmi_source = (user_dir / "hmi_tjc.c").read_text(encoding="utf-8")

        for token in (
            "MEASUREMENT_VALID_DC_VOLTAGE",
            "MEASUREMENT_VALID_AMPLITUDE",
            "MEASUREMENT_VALID_RMS",
            "MEASUREMENT_VALID_FREQUENCY",
            "MEASUREMENT_VALID_THD",
            "MEASUREMENT_VALID_WAVE_TYPE",
            "MEASUREMENT_VALID_PHASE",
            "MEASUREMENT_VALID_SPECTRUM",
            "uint16_t valid_mask",
        ):
            with self.subTest(token=token):
                self.assertIn(token, header)

        self.assertIn("valid_mask |= MEASUREMENT_VALID_PHASE", fft_source)
        self.assertIn("result->valid_mask", hmi_source)
        self.assertIn('wave_text = "DC";', hmi_source)

    def test_ads8688_range_and_voltage_apis_are_public(self):
        """FFT 与最新值存储必须共用同一套量程和电压换算接口。"""
        header = (user_dir / "ads8688.h").read_text(encoding="utf-8")
        storage = (user_dir / "ads8688_storage.c").read_text(encoding="utf-8")

        normalized_header = re.sub(r"\s+", " ", header)
        self.assertIn(
            (
                "ads8688_status_t ads8688_get_channel_range("
                "uint8_t channel, ads8688_range_t *range);"
            ),
            normalized_header,
        )
        self.assertIn(
            (
                "ads8688_status_t ads8688_convert_raw_to_voltage("
                "uint16_t raw_code, ads8688_range_t range, float *voltage);"
            ),
            normalized_header,
        )
        self.assertNotIn("ads8688_storage_convert_voltage", storage)

    def test_hardware_self_test_is_disabled_for_real_measurement(self):
        """固定 HMI 自检值不得覆盖真实 FFT 发布结果。"""
        source = (user_dir / "system.c").read_text(encoding="utf-8")

        self.assertRegex(
            source,
            r"#define\s+HMI_TJC_SELF_TEST_ENABLE\s+0u",
        )

    def test_measurement_calibration_has_public_validated_boundary(self):
        """每通道校准必须通过受检接口配置，不能散落为算法魔数。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        for token in (
            "measurement_fft_calibration_t",
            "measurement_fft_set_calibration",
            "measurement_fft_get_calibration",
            "calibration->valid > 1u",
            "calibration->volts_per_code == 0.0f",
            "!isfinite(calibration->offset_v)",
        ):
            with self.subTest(token=token):
                self.assertIn(token, header + source)

        self.assertIn("calibration->volts_per_code", source)
        self.assertIn("measurement_fft_calibration[channel]", source)
        self.assertIn("voltage_status[0] != 0u", source)

    def test_synchronized_pair_sample_rate_contract(self):
        """片上双 ADC 必须以80 kSPS同步样本对进入FFT。"""
        header = (user_dir / "measurement_fft.h").read_text(encoding="utf-8")
        source = (user_dir / "measurement_fft.c").read_text(encoding="utf-8")

        self.assertIn("measurement_fft_ingest_pair(uint16_t ch1_raw_code", header)
        self.assertIn("MEASUREMENT_FFT_RAW_SAMPLE_RATE_HZ 80000.0f", source)
        self.assertIn("measurement_fft_sampling_required", header + source)

    def test_hmi_detail_and_spectrum_builders_are_available_but_not_automatic(self):
        """扩展控件和频谱应可构帧，但未知控件 ID 时不得自动发送。"""
        header = (user_dir / "hmi_tjc.h").read_text(encoding="utf-8")
        source = (user_dir / "hmi_tjc.c").read_text(encoding="utf-8")

        for token in (
            "hmi_tjc_build_detail_frame",
            "hmi_tjc_build_spectrum_frame",
            "HMI_TJC_SPECTRUM_COMPONENT_DISABLED",
            '"t_dc"',
            '"t_rms"',
            '"t_thd"',
            '"cle %u,%u"',
            '"add %u,%u,%u"',
        ):
            with self.subTest(token=token):
                self.assertIn(token, header + source)

        self.assertEqual(source.count("hmi_tjc_build_spectrum_frame("), 1)
        self.assertEqual(
            source.count("frame[offset++] = HMI_TJC_TERMINATOR_BYTE;"),
            3,
        )


if __name__ == "__main__":
    unittest.main()
