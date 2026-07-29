"""无 FPGA 测量数学的主机参考测试。

用“较弱基波 + 更强三次谐波 + 五次谐波 + 1 MHz 以上单频干扰”
验证倍频约束、联合正余弦最小二乘、有效值和重建峰峰值的设计目标。
该脚本验证算法数学，不代替 STM32 实板 ADC、缓存和执行时间验证。
"""

from __future__ import annotations

import math

import numpy as np


SAMPLE_RATE_HZ = 3_200_000.0
SAMPLE_COUNT = 8192
BIN_SPACING_HZ = SAMPLE_RATE_HZ / SAMPLE_COUNT


def refine_peak(power: np.ndarray, bin_index: int) -> float:
    """用对数功率抛物线细化一个局部谱峰。"""
    left, center, right = np.log(
        power[bin_index - 1 : bin_index + 2] + np.finfo(float).tiny
    )
    denominator = left - 2.0 * center + right
    delta = 0.0 if abs(denominator) < 1e-20 else 0.5 * (left - right) / denominator
    return bin_index + float(np.clip(delta, -0.5, 0.5))


def local_peaks(
    power: np.ndarray, minimum_hz: float, maximum_hz: float
) -> list[tuple[float, float]]:
    """返回指定频带内按功率降序排列的局部峰。"""
    first = max(2, math.ceil(minimum_hz / BIN_SPACING_HZ))
    last = min(SAMPLE_COUNT // 2 - 2, math.floor(maximum_hz / BIN_SPACING_HZ))
    found: list[tuple[float, float]] = []
    for index in range(first, last + 1):
        if power[index] > power[index - 1] and power[index] >= power[index + 1]:
            refined = refine_peak(power, index)
            found.append((refined * BIN_SPACING_HZ, float(power[index])))
    found.sort(key=lambda item: item[1], reverse=True)
    return found[:12]


def estimate_fundamental(peaks: list[tuple[float, float]]) -> float:
    """复现固件的“基波必须存在 + 其余峰为整数倍”约束。"""
    tolerance = 1.75 * BIN_SPACING_HZ
    best_frequency = 0.0
    best_key = (-1, -1.0)
    for peak_hz, _ in peaks:
        for divisor in range(1, 51):
            candidate = peak_hz / divisor
            if not 8_000.0 <= candidate <= 550_000.0:
                continue
            if min(abs(item[0] - candidate) for item in peaks) > tolerance:
                continue
            weighted_sum = 0.0
            weight_sum = 0.0
            score = 0.0
            matches = 0
            for frequency_hz, power_value in peaks:
                order = round(frequency_hz / candidate)
                if not 1 <= order <= 50:
                    continue
                error = abs(frequency_hz - candidate * order)
                if error <= tolerance:
                    amplitude_weight = math.sqrt(power_value)
                    frequency_weight = amplitude_weight * order * order
                    weighted_sum += frequency_hz / order * frequency_weight
                    weight_sum += frequency_weight
                    score += amplitude_weight * (1.0 - 0.4 * error / tolerance)
                    matches += 1
            if matches >= 2 and (matches, score) > best_key:
                best_key = (matches, score)
                best_frequency = weighted_sum / weight_sum
    if best_frequency == 0.0:
        raise AssertionError("没有找到满足倍频关系的基波")
    return best_frequency


def fit_tones(samples: np.ndarray, frequencies_hz: list[float]) -> tuple[np.ndarray, float]:
    """对多个频率同时做正余弦最小二乘。"""
    index = np.arange(samples.size, dtype=np.float64)
    columns = []
    for frequency_hz in frequencies_hz:
        angle = 2.0 * np.pi * frequency_hz * index / SAMPLE_RATE_HZ
        columns.extend((np.cos(angle), np.sin(angle)))
    basis = np.column_stack(columns)
    coefficients, _, _, _ = np.linalg.lstsq(basis, samples, rcond=None)
    residual = samples - basis @ coefficients
    return coefficients, float(residual @ residual)


def main() -> None:
    """运行完整参考场景并检查误差。"""
    fundamental_hz = 37_250.0
    harmonic_orders = [1, 3, 5]
    amplitudes_v = np.array([0.018, 0.040, 0.022])
    phases_rad = np.array([0.35, -1.10, 2.05])
    interference_hz = 1_123_400.0
    interference_peak_v = 0.100
    interference_phase = -0.47
    sample_index = np.arange(SAMPLE_COUNT, dtype=np.float64)

    useful = np.zeros(SAMPLE_COUNT, dtype=np.float64)
    for order, amplitude, phase in zip(
        harmonic_orders, amplitudes_v, phases_rad, strict=True
    ):
        useful += amplitude * np.cos(
            2.0
            * np.pi
            * fundamental_hz
            * order
            * sample_index
            / SAMPLE_RATE_HZ
            + phase
        )
    acquired = useful + interference_peak_v * np.cos(
        2.0 * np.pi * interference_hz * sample_index / SAMPLE_RATE_HZ
        + interference_phase
    )
    adc_codes = np.clip(
        np.rint((1.65 + acquired) * 4095.0 / 3.3), 0, 4095
    )
    centered_v = (adc_codes - np.mean(adc_codes)) * 3.3 / 4095.0

    windowed = centered_v * np.hanning(SAMPLE_COUNT)
    spectrum = np.fft.rfft(windowed)
    power = spectrum.real * spectrum.real + spectrum.imag * spectrum.imag
    signal_peaks = local_peaks(power, 8_000.0, 550_000.0)
    interference_peaks = local_peaks(power, 650_000.0, 1_550_000.0)
    estimated_hz = estimate_fundamental(signal_peaks)

    selected_orders = []
    for frequency_hz, _ in signal_peaks:
        order = round(frequency_hz / estimated_hz)
        if (
            1 <= order <= 50
            and abs(frequency_hz - order * estimated_hz)
            <= 1.75 * BIN_SPACING_HZ
            and order not in selected_orders
        ):
            selected_orders.append(order)
    assert 1 in selected_orders
    selected_orders = [1] + [
        order for order in selected_orders if order != 1
    ][:2]
    selected_orders.sort()

    interference_seed_hz = interference_peaks[0][0]
    grid = estimated_hz + np.arange(-2, 3) * BIN_SPACING_HZ * 0.25
    residuals = []
    for trial_hz in grid:
        trial_frequencies = [
            trial_hz * order for order in selected_orders
        ] + [interference_seed_hz]
        _, residual = fit_tones(centered_v, trial_frequencies)
        residuals.append(residual)
    best_index = int(np.argmin(residuals))
    refined_hz = float(grid[best_index])
    if 0 < best_index < len(grid) - 1:
        left, center, right = residuals[best_index - 1 : best_index + 2]
        denominator = left - 2.0 * center + right
        if abs(denominator) > 1e-20:
            refined_hz += float(
                np.clip(0.5 * (left - right) / denominator, -1.0, 1.0)
                * BIN_SPACING_HZ
                * 0.25
            )

    fitted_frequencies = [
        refined_hz * order for order in selected_orders
    ] + [interference_seed_hz]
    coefficients, _ = fit_tones(centered_v, fitted_frequencies)
    fitted_amplitudes_v = np.hypot(
        coefficients[0 : 2 * len(selected_orders) : 2],
        coefficients[1 : 2 * len(selected_orders) : 2],
    )
    fitted_rms_v = math.sqrt(float(np.sum(fitted_amplitudes_v**2) / 2.0))

    phase = np.linspace(0.0, 2.0 * np.pi, 4096, endpoint=False)
    reconstruction = np.zeros_like(phase)
    for component, order in enumerate(selected_orders):
        reconstruction += (
            coefficients[2 * component] * np.cos(order * phase)
            + coefficients[2 * component + 1] * np.sin(order * phase)
        )
    fitted_vpp_v = float(np.ptp(reconstruction))
    true_rms_v = math.sqrt(float(np.sum(amplitudes_v**2) / 2.0))
    true_vpp_v = float(
        np.ptp(
            sum(
                amplitude * np.cos(order * phase + phase_offset)
                for order, amplitude, phase_offset in zip(
                    harmonic_orders, amplitudes_v, phases_rad, strict=True
                )
            )
        )
    )

    assert selected_orders == harmonic_orders
    assert abs(refined_hz - fundamental_hz) < 20.0
    assert np.max(np.abs(fitted_amplitudes_v - amplitudes_v)) < 0.0005
    assert abs(fitted_rms_v - true_rms_v) < 0.0005
    assert abs(fitted_vpp_v - true_vpp_v) < 0.001
    print(
        "Measurement math passed: "
        f"f0_error={refined_hz - fundamental_hz:+.2f} Hz, "
        f"rms_error={(fitted_rms_v - true_rms_v) * 1000:+.3f} mV, "
        f"vpp_error={(fitted_vpp_v - true_vpp_v) * 1000:+.3f} mV"
    )


if __name__ == "__main__":
    main()
