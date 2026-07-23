from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


class Ads8688HardwareContract(unittest.TestCase):
    def test_spi3_and_dma_match_ads8688(self):
        ioc = read("h743_pre1.ioc")
        required = (
            "PA15\\ (JTDI).Signal=SPI3_NSS",
            "PC10.Signal=SPI3_SCK",
            "PC11.Signal=SPI3_MISO",
            "PC12.Signal=SPI3_MOSI",
            "PD0.GPIO_Label=ADS8688_DAISY",
            "PD1.GPIO_Label=ADS8688_RST",
            "SPI3.DataSize=SPI_DATASIZE_32BIT",
            "SPI3.CLKPhase=SPI_PHASE_2EDGE",
            "SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4",
            "SPI3.MasterInterDataIdleness=SPI_MASTER_INTERDATA_IDLENESS_02CYCLE",
            "Dma.SPI3_RX.1.MemInc=DMA_MINC_ENABLE",
            "Dma.SPI3_TX.2.MemInc=DMA_MINC_DISABLE",
        )
        for line in required:
            self.assertIn(line, ioc)

    def test_ads8688_public_lifecycle_and_channel_api(self):
        header = read("Core/User/ads8688.h")
        for name in (
            "ads8688_start",
            "ads8688_stop",
            "ads8688_set_single_channel",
            "ads8688_set_dual_channel",
            "ads8688_get_effective_sample_rate_hz",
        ):
            self.assertIn(name, header)

    def test_callbacks_only_set_one_ads8688_flag(self):
        source = read("Core/User/ads8688.c")
        expected = (
            ("HAL_SPI_TxRxHalfCpltCallback", "ads8688_dma_half_flag"),
            ("HAL_SPI_TxRxCpltCallback", "ads8688_dma_full_flag"),
            ("HAL_SPI_ErrorCallback", "ads8688_error_flag"),
        )
        for function, flag in expected:
            self.assertIn(f"void {function}", source)
            body = source.split(f"void {function}", 1)[1].split("\n}", 1)[0]
            assigned = re.findall(r"\b([a-z0-9_]+_flag)\s*=", body)
            self.assertEqual(assigned, [flag])

    def test_ads8688_uses_generated_spi3_and_gpio_names(self):
        source = read("Core/User/ads8688.c")
        self.assertIn("&hspi3", source)
        self.assertIn("ADS8688_DAISY_GPIO_Port", source)
        self.assertIn("ADS8688_RST_GPIO_Port", source)
        self.assertNotIn("&hspi2", source)
        self.assertNotIn("GPIO_PIN_8", source)
        self.assertNotIn("GPIO_PIN_9", source)

    def test_driver_recovers_backlog_and_uses_enabled_channel_count(self):
        source = read("Core/User/ads8688.c")
        self.assertIn("if (pending_flags == 3u)", source)
        self.assertIn("measurement_fft_resynchronize();", source)
        public_stop = source.split(
            "ads8688_status_t ads8688_stop(void)", 1
        )[1].split("\n}", 1)[0]
        self.assertIn("ads8688_recovery_pending = 1u;", public_stop)
        self.assertIn("ads8688_initialized = 0u;", public_stop)
        self.assertIn("enabled_channel_count++", source)
        self.assertIn(
            "frame_rate_hz / (float)enabled_channel_count",
            source,
        )

    def test_driver_restores_saved_state_if_dma_restart_fails(self):
        source = read("Core/User/ads8688.c")
        for assignment in (
            "ads8688_mode = previous_mode;",
            "ads8688_channel_mask = previous_channel_mask;",
            "ads8688_channel_ranges[channel] = previous_range;",
        ):
            self.assertIn(assignment, source)

    def test_power_down_wakeup_precedes_configuration(self):
        source = read("Core/User/ads8688.c")
        self.assertIn(
            "#define ADS8688_SPI_FRAME_CYCLES           34.0f",
            source,
        )
        body = source.split(
            "static ads8688_status_t ads8688_initialize_attempt(void)", 1
        )[1].split("\n}", 1)[0]
        power_down_delay = body.index("HAL_Delay(1u);")
        wake_command = body.index(
            "status = ads8688_send_command(ADS8688_COMMAND_AUTO_RST);"
        )
        reference_delay = body.index("HAL_Delay(15u);")
        first_configuration = body.index(
            "status = ads8688_write_and_verify_register("
        )
        final_auto_reset = body.rindex(
            "ads8688_send_command(ADS8688_COMMAND_AUTO_RST)"
        )
        self.assertLess(power_down_delay, wake_command)
        self.assertLess(wake_command, reference_delay)
        self.assertLess(reference_delay, first_configuration)
        self.assertLess(first_configuration, final_auto_reset)


class MeasurementInputContract(unittest.TestCase):
    def test_fft_profile_and_single_ingest_exist(self):
        header = read("Core/User/measurement_fft.h")
        for name in (
            "measurement_fft_input_profile_t",
            "measurement_fft_configure_input",
            "measurement_fft_ingest_single",
        ):
            self.assertIn(name, header)

    def test_runtime_rate_math(self):
        self.assertAlmostEqual(16_000_000.0 / 34.0, 470_588.2353, places=3)
        self.assertAlmostEqual(16_000_000.0 / 68.0, 235_294.1176, places=3)
        self.assertAlmostEqual(34.0 / 16_000_000.0, 2.125e-6, places=12)

    def test_fft_uses_runtime_rate_and_phase_delay(self):
        source = read("Core/User/measurement_fft.c")
        self.assertIn("measurement_fft_input_profile.sample_rate_hz", source)
        self.assertIn("measurement_fft_input_profile.ch2_delay_seconds", source)
        self.assertIn("measurement_fft_ingest_single", source)
        self.assertIn("MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL", source)
        self.assertIn("secondary_valid_mask = 0u", source)

    def test_internal_adc_has_explicit_lifecycle(self):
        header = read("Core/User/adc_dual.h")
        source = read("Core/User/adc_dual.c")
        self.assertIn("adc_dual_status_t", header)
        self.assertIn("adc_dual_start(void)", header)
        self.assertIn("adc_dual_stop(void)", header)
        self.assertIn("HAL_TIM_Base_Stop(&htim2)", source)

    def test_manager_and_system_integration_exist(self):
        header = read("Core/User/measurement_input.h")
        manager = read("Core/User/measurement_input.c")
        source = read("Core/User/system.c")
        for name in (
            "measurement_input_select",
            "measurement_input_set_ads8688_single_channel",
            "measurement_input_set_ads8688_dual_channel",
            "measurement_input_get_diagnostics",
        ):
            self.assertIn(name, header)
        self.assertIn("measurement_input_init();", source)
        self.assertIn("measurement_input_process();", source)
        self.assertNotIn("adc_dual_init();", source)
        self.assertIn("measurement_input_restore_ads", manager)
        self.assertIn("validation_calibration", manager)
        self.assertIn("status == ADS8688_STATUS_NOT_INITIALIZED", manager)
        self.assertIn("status = ads8688_init();", manager)

    def test_ads8688_is_in_unified_header_and_active_build(self):
        header = read("Core/User/system.h")
        project = read(".cproject")
        self.assertIn('#include "ads8688.h"', header)
        self.assertIn('#include "measurement_input.h"', header)
        self.assertNotIn("User/ads8688.c|User/ads8688_storage.c", project)


if __name__ == "__main__":
    unittest.main()
