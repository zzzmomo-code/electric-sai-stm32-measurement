import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


class DdsContractTest(unittest.TestCase):
    def test_ioc_matches_ad9834_spi_contract(self) -> None:
        ioc = read_text("h743_pre1.ioc")

        required_lines = (
            "PB12.GPIO_Label=DDS_FSYNC",
            "PB12.PinState=GPIO_PIN_SET",
            "PB12.Signal=GPIO_Output",
            "PB13.Signal=SPI2_SCK",
            "PB15.Signal=SPI2_MOSI",
            "RCC.SPI123CLockSelection=RCC_SPI123CLKSOURCE_CLKP",
            "RCC.SPI123Freq_Value=64000000",
            "SPI2.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8",
            "SPI2.CalculateBaudRate=8.0 MBits/s",
            "SPI2.CLKPolarity=SPI_POLARITY_HIGH",
            "SPI2.DataSize=SPI_DATASIZE_16BIT",
            "SPI2.Direction=SPI_DIRECTION_2LINES_TXONLY",
            "SPI2.NSSPMode=SPI_NSS_PULSE_DISABLE",
        )
        for line in required_lines:
            self.assertIn(line, ioc)

    def test_driver_uses_75mhz_ftw_and_manual_fsync(self) -> None:
        header = read_text("Core/User/ad9834.h")
        source = read_text("Core/User/ad9834.c")

        self.assertIn("#define AD9834_MCLK_HZ 75000000u", header)
        self.assertIn("((uint64_t)frequency_hz) << 28", source)
        self.assertIn("AD9834_CONTROL_RESET 0x2100u", source)
        self.assertIn("AD9834_CONTROL_RUN   0x2000u", source)
        self.assertIn("HAL_SPI_Transmit(&hspi2", source)

        fsync_low = source.index("DDS_FSYNC_Pin, GPIO_PIN_RESET")
        transmit = source.index("HAL_SPI_Transmit(&hspi2")
        fsync_high = source.index("DDS_FSYNC_Pin, GPIO_PIN_SET", transmit)
        self.assertLess(fsync_low, transmit)
        self.assertLess(transmit, fsync_high)

    def test_fixed_board_test_outputs_900khz(self) -> None:
        header = read_text("Core/User/dds_control.h")
        source = read_text("Core/User/dds_control.c")

        self.assertIn("#define DDS_CONTROL_FIXED_TEST_ENABLE 1u", header)
        self.assertIn("#define DDS_CONTROL_TEST_INPUT_HZ 1000000u", header)
        self.assertIn("#define DDS_CONTROL_TARGET_IF_HZ 100000u", header)
        self.assertIn("return input_frequency_hz - DDS_CONTROL_TARGET_IF_HZ;", source)
        self.assertEqual(1_000_000 - 100_000, 900_000)

        expected_ftw = ((900_000 << 28) + 75_000_000 // 2) // 75_000_000
        self.assertEqual(expected_ftw, 3_221_225)

    def test_system_initializes_spi_before_dds(self) -> None:
        main = read_text("Core/Src/main.c")
        system = read_text("Core/User/system.c")

        self.assertLess(main.index("MX_SPI2_Init();"), main.index("system_init();"))
        self.assertIn("dds_control_init();", system)
        self.assertIn("dds_control_process();", system)

        init_body = re.search(
            r"void system_init\(void\)\s*\{(?P<body>.*?)\n\}",
            system,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(init_body)
        self.assertLess(
            init_body.group("body").index("frequency_measure_init();"),
            init_body.group("body").index("dds_control_init();"),
        )


if __name__ == "__main__":
    unittest.main()
