"""用合成信号验证固件采用的时域、FFT、THD 与频谱数学模型。"""

import math
import unittest

try:
    import numpy as np
except ImportError:  # pragma: no cover - 无 NumPy 环境仍可运行其他契约测试
    np = None


FFT_LENGTH = 8192
RAW_SAMPLE_RATE_HZ = 500_000.0
INITIAL_DECIMATION = 4
LOW_BAND_DECIMATION = 8
SPECTRUM_POINT_COUNT = 64
SPECTRUM_MAX_HZ = 20_000.0


def _wrap_phase(phase_deg):
    """按固件规则将角度折返到 -180 至 180 度。"""
    while phase_deg > 180.0:
        phase_deg -= 360.0
    while phase_deg <= -180.0:
        phase_deg += 360.0
    return phase_deg


def _analyze(signal, sample_rate_hz):
    """按固件的 Hann 窗和三点对数抛物线方法估计主频。"""
    windowed = (signal - signal.mean()) * np.hanning(FFT_LENGTH)
    spectrum = np.fft.rfft(windowed)
    power = spectrum.real * spectrum.real + spectrum.imag * spectrum.imag
    peak_bin = int(np.argmax(power[1:-1]) + 1)
    left, center, right = np.log(power[peak_bin - 1 : peak_bin + 2])
    denominator = left - 2.0 * center + right
    offset = 0.5 * (left - right) / denominator
    offset = max(-0.5, min(0.5, float(offset)))
    frequency_hz = (peak_bin + offset) * sample_rate_hz / FFT_LENGTH
    return spectrum, power, peak_bin, offset, frequency_hz


def _harmonic_ratio(power, peak_bin, peak_offset, harmonic_order):
    """按固件规则读取目标谐波左右一个频点内的最大幅值比。"""
    target_bin = (peak_bin + peak_offset) * harmonic_order
    center_bin = int(target_bin + 0.5)
    harmonic_power = max(power[center_bin - 1 : center_bin + 2])
    return math.sqrt(float(harmonic_power / power[peak_bin]))


def _classify(harmonic_ratio_3, harmonic_ratio_5):
    """复现固件当前可上板调参的谐波分类阈值。"""
    if harmonic_ratio_3 >= 0.200 and harmonic_ratio_5 >= 0.080:
        return "SQUARE"
    if 0.055 <= harmonic_ratio_3 < 0.200 and harmonic_ratio_5 <= 0.080:
        return "TRIANGLE"
    if harmonic_ratio_3 < 0.055 and harmonic_ratio_5 < 0.040:
        return "SINE"
    return "UNKNOWN"


def _band_power(power, center_bin, radius=2):
    """复现固件对 Hann 主瓣左右两个频点的能量求和。"""
    if center_bin <= radius or center_bin + radius >= FFT_LENGTH // 2:
        return 0.0
    return float(power[center_bin - radius : center_bin + radius + 1].sum())


def _thd_percent(power, fundamental_bin, max_harmonic=10):
    """按固件的可观测谐波范围计算 THD 百分比。"""
    fundamental_center = int(fundamental_bin + 0.5)
    fundamental_power = _band_power(power, fundamental_center)
    harmonic_power = 0.0
    harmonic_count = 0
    for order in range(2, max_harmonic + 1):
        target_bin = fundamental_bin * order
        if target_bin >= FFT_LENGTH // 2 - 2:
            break
        center_bin = int(target_bin + 0.5)
        if center_bin <= fundamental_center + 4:
            continue
        band_power = _band_power(power, center_bin)
        if band_power > 0.0:
            harmonic_power += band_power
            harmonic_count += 1
    if harmonic_count == 0 or fundamental_power <= 0.0:
        return 0.0, harmonic_count
    return 100.0 * math.sqrt(harmonic_power / fundamental_power), harmonic_count


def _compress_spectrum(power, sample_rate_hz):
    """把 0 至 20 kHz 频谱压缩为固件公开的 64 点相对 dB 数据。"""
    point_width_hz = SPECTRUM_MAX_HZ / SPECTRUM_POINT_COUNT
    bin_width_hz = sample_rate_hz / FFT_LENGTH
    nyquist_hz = sample_rate_hz / 2.0
    reference_power = float(power[1 : FFT_LENGTH // 2].max())
    compressed = []
    for point in range(SPECTRUM_POINT_COUNT):
        start_hz = point * point_width_hz
        end_hz = start_hz + point_width_hz
        if start_hz >= nyquist_hz or reference_power <= 0.0:
            compressed.append(-80.0)
            continue
        start_bin = max(1, int(start_hz / bin_width_hz))
        end_bin = min(FFT_LENGTH // 2 - 1, int(end_hz / bin_width_hz))
        largest_power = float(power[start_bin : end_bin + 1].max())
        relative_db = 10.0 * math.log10(largest_power / reference_power)
        compressed.append(max(-80.0, min(0.0, relative_db)))
    return compressed


@unittest.skipIf(np is None, "需要 NumPy 执行离线合成信号模型测试")
class MeasurementFftModelTest(unittest.TestCase):
    """验证离线算法基线，实板误差仍需信号源校准。"""

    def setUp(self):
        self.frequency_hz = 12_345.0
        self.sample_rate_hz = RAW_SAMPLE_RATE_HZ / INITIAL_DECIMATION
        self.time_s = np.arange(FFT_LENGTH) / self.sample_rate_hz
        self.base_phase = 2.0 * np.pi * self.frequency_hz * self.time_s

    def test_off_bin_frequency_interpolation_is_sub_hertz(self):
        """8192 点 Hann 窗对数插值应把已知非整频点误差控制在 1 Hz 内。"""
        signal = np.sin(self.base_phase)
        _, _, _, _, measured_hz = _analyze(signal, self.sample_rate_hz)

        self.assertLess(abs(measured_hz - self.frequency_hz), 1.0)

    def test_synchronous_channels_recover_phase_without_delay_compensation(self):
        """同一 TIM2 触发时刻的双 ADC 样本应直接恢复真实相位差。"""
        expected_phase_deg = 67.0
        first_signal = np.sin(self.base_phase)
        second_signal = np.sin(self.base_phase + np.deg2rad(expected_phase_deg))
        first_spectrum, _, first_bin, _, _ = _analyze(
            first_signal, self.sample_rate_hz
        )
        second_spectrum, _, second_bin, _, _ = _analyze(
            second_signal, self.sample_rate_hz
        )

        self.assertEqual(first_bin, second_bin)
        cross_spectrum = second_spectrum[first_bin] * np.conj(
            first_spectrum[first_bin]
        )
        raw_phase_deg = math.degrees(float(np.angle(cross_spectrum)))
        corrected_phase_deg = _wrap_phase(raw_phase_deg)

        self.assertLess(abs(corrected_phase_deg - expected_phase_deg), 0.1)

    def test_harmonic_thresholds_separate_three_basic_waveforms(self):
        """当前阈值应能区分理想正弦波、方波和三角波。"""
        frequency_hz = 1_000.0
        sample_rate_hz = RAW_SAMPLE_RATE_HZ / LOW_BAND_DECIMATION
        time_s = np.arange(FFT_LENGTH) / sample_rate_hz
        base_phase = 2.0 * np.pi * frequency_hz * time_s
        signals = {
            "SINE": np.sin(base_phase),
            "SQUARE": np.sign(np.sin(base_phase)),
            "TRIANGLE": 2.0 / np.pi * np.arcsin(np.sin(base_phase)),
        }

        for expected_wave, signal in signals.items():
            with self.subTest(wave=expected_wave):
                _, power, peak_bin, peak_offset, _ = _analyze(
                    signal, sample_rate_hz
                )
                ratio_3 = _harmonic_ratio(power, peak_bin, peak_offset, 3)
                ratio_5 = _harmonic_ratio(power, peak_bin, peak_offset, 5)
                self.assertEqual(_classify(ratio_3, ratio_5), expected_wave)

    def test_low_band_decimation_meets_20_hz_frequency_requirement(self):
        """低频抽取后 20 Hz 合成信号的插值误差应明显小于题目 10 Hz。"""
        frequency_hz = 20.0
        sample_rate_hz = RAW_SAMPLE_RATE_HZ / LOW_BAND_DECIMATION
        time_s = np.arange(FFT_LENGTH) / sample_rate_hz
        signal = np.sin(2.0 * np.pi * frequency_hz * time_s)

        _, _, _, _, measured_hz = _analyze(signal, sample_rate_hz)

        self.assertLess(abs(measured_hz - frequency_hz), 1.0)
        self.assertLess(sample_rate_hz / FFT_LENGTH, 10.0)

    def test_known_harmonics_produce_expected_thd(self):
        """10% 三次和 5% 五次谐波应得到约 11.18% THD。"""
        frequency_hz = 1_000.0
        sample_rate_hz = RAW_SAMPLE_RATE_HZ / LOW_BAND_DECIMATION
        time_s = np.arange(FFT_LENGTH) / sample_rate_hz
        phase = 2.0 * np.pi * frequency_hz * time_s
        signal = np.sin(phase) + 0.10 * np.sin(3.0 * phase) + 0.05 * np.sin(
            5.0 * phase
        )
        _, power, peak_bin, peak_offset, _ = _analyze(signal, sample_rate_hz)

        measured_thd, harmonic_count = _thd_percent(
            power, peak_bin + peak_offset
        )

        self.assertGreaterEqual(harmonic_count, 4)
        self.assertAlmostEqual(measured_thd, math.sqrt(0.10**2 + 0.05**2) * 100.0, delta=0.35)

    def test_time_domain_dc_rms_and_sine_vpp_conversion(self):
        """已知 1 V 偏置、0.5 V 峰值正弦应恢复 DC、RMS 和 Vpp。"""
        frequency_hz = 1_000.0
        sample_rate_hz = RAW_SAMPLE_RATE_HZ / LOW_BAND_DECIMATION
        time_s = np.arange(FFT_LENGTH) / sample_rate_hz
        voltage = 1.0 + 0.5 * np.sin(2.0 * np.pi * frequency_hz * time_s)
        volts_per_code = 5.0 / 65536.0
        offset_v = -2.5
        raw_code = np.rint((voltage - offset_v) / volts_per_code)
        mean_raw = raw_code.mean()
        rms_raw = np.sqrt(np.mean((raw_code - mean_raw) ** 2))
        dc_voltage = offset_v + mean_raw * volts_per_code
        rms_voltage = rms_raw * volts_per_code
        amplitude_vpp = 2.0 * math.sqrt(2.0) * rms_voltage

        self.assertAlmostEqual(dc_voltage, 1.0, delta=0.001)
        self.assertAlmostEqual(rms_voltage, 0.5 / math.sqrt(2.0), delta=0.001)
        self.assertAlmostEqual(amplitude_vpp, 1.0, delta=0.003)

    def test_compressed_spectrum_marks_fundamental_bucket(self):
        """64 点压缩频谱的最高点应落入已知基波所在频段。"""
        frequency_hz = 1_000.0
        sample_rate_hz = RAW_SAMPLE_RATE_HZ / LOW_BAND_DECIMATION
        time_s = np.arange(FFT_LENGTH) / sample_rate_hz
        signal = np.sin(2.0 * np.pi * frequency_hz * time_s)
        _, power, _, _, _ = _analyze(signal, sample_rate_hz)

        compressed = _compress_spectrum(power, sample_rate_hz)
        peak_point = int(np.argmax(compressed))
        expected_point = int(frequency_hz / (SPECTRUM_MAX_HZ / SPECTRUM_POINT_COUNT))

        self.assertEqual(peak_point, expected_point)
        self.assertAlmostEqual(compressed[peak_point], 0.0, places=6)

    def test_linear_front_end_calibration_matches_known_codes(self):
        """前级比例和偏置确认后应按统一线性校准参数换算原始码。"""
        volts_per_code = 5.0 / 65536.0
        offset_v = -2.5

        def convert(raw_code):
            return raw_code * volts_per_code + offset_v

        self.assertAlmostEqual(convert(32768), 0.0, places=6)
        self.assertAlmostEqual(convert(65535), 2.5 - volts_per_code, places=6)


if __name__ == "__main__":
    unittest.main()
