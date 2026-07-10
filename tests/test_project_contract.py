"""ADS8688 用户模块公共接口契约测试。"""

import re
import unittest
from pathlib import Path


project_root = Path(__file__).resolve().parents[1]
user_dir = project_root / "Core" / "User"


def has_function_prototype(source, function_name):
    """判断源码文本中是否出现指定函数原型。"""
    source_without_comments = re.sub(
        r"/\*.*?\*/|//[^\r\n]*",
        "",
        source,
        flags=re.DOTALL,
    )
    prototype_pattern = (
        rf"(?m)^[ \t]*(?:[A-Za-z_]\w*[ \t]+)+"
        rf"(?:\*[ \t]*)?{re.escape(function_name)}[ \t]*"
        rf"\([^;{{}}]*\)[ \t]*;"
    )
    return re.search(prototype_pattern, source_without_comments) is not None


def strip_c_comments(source):
    """移除 C 源码注释，避免仅在注释中提及名称就通过契约测试。"""
    return re.sub(
        r"/\*.*?\*/|//[^\r\n]*",
        " ",
        source,
        flags=re.DOTALL,
    )


def normalize_c_declarations(source):
    """去除注释并规范化声明中的空白，避免格式差异影响契约匹配。"""
    source_without_comments = strip_c_comments(source)
    source_without_directives = re.sub(
        r"(?m)^[ \t]*#[^\r\n]*",
        "",
        source_without_comments,
    )
    normalized = re.sub(r"\s+", " ", source_without_directives).strip()
    normalized = re.sub(r"\s*([(),;*])\s*", r"\1", normalized)
    return normalized


def has_exact_declaration(source, declaration):
    """判断源码是否包含返回类型、参数类型和参数顺序均匹配的完整声明。"""
    normalized_source = normalize_c_declarations(source)
    normalized_declaration = normalize_c_declarations(declaration)
    source_statements = {
        f"{statement.strip()};"
        for statement in normalized_source.split(";")
        if statement.strip()
    }
    return normalized_declaration in source_statements


def get_function_body(source, function_name):
    """提取简单 C 函数的函数体，供中断回调契约检查使用。"""
    source_without_comments = strip_c_comments(source)
    function_match = re.search(
        rf"\b{re.escape(function_name)}\s*\([^)]*\)\s*\{{",
        source_without_comments,
    )
    if function_match is None:
        return None

    body_start = function_match.end()
    depth = 1
    for index in range(body_start, len(source_without_comments)):
        if source_without_comments[index] == "{":
            depth += 1
        elif source_without_comments[index] == "}":
            depth -= 1
            if depth == 0:
                return source_without_comments[body_start:index]
    return None


def get_user_code_section(source, section_name):
    """提取 CubeMX USER CODE 区域正文，确保 main.c 只放置允许的用户调用。"""
    pattern = (
        rf"/\* USER CODE BEGIN {re.escape(section_name)} \*/"
        rf"(?P<body>.*?)"
        rf"/\* USER CODE END {re.escape(section_name)} \*/"
    )
    match = re.search(pattern, source, flags=re.DOTALL)
    if match is None:
        return None
    return match.group("body").strip()


class ProjectContractTest(unittest.TestCase):
    """验证 ADS8688 用户模块对外公开的头文件与接口契约。"""

    def test_required_headers_exist(self):
        """用户公共头文件必须位于 Core/User 目录。"""
        for header_name in (
            "ads8688.h",
            "ads8688_storage.h",
            "measurement_result.h",
            "hmi_tjc.h",
            "system.h",
        ):
            with self.subTest(header=header_name):
                self.assertTrue(
                    (user_dir / header_name).is_file(),
                    f"缺少必需头文件 Core/User/{header_name}",
                )

    def test_main_c_uses_only_system_entry_points(self):
        """main.c 的用户区只能包含统一头文件、初始化入口和功能处理调用。"""
        main = (project_root / "Core" / "Src" / "main.c").read_text(
            encoding="utf-8"
        )

        self.assertEqual(
            get_user_code_section(main, "Includes"),
            '#include "system.h"',
        )
        self.assertRegex(
            get_user_code_section(main, "2"),
            (
                r"^system_init\(\);\s*"
                r"#if defined\(HAL_UART_MODULE_ENABLED\)\s*"
                r"hmi_tjc_bind_uart\(\s*&huart1\s*\);\s*#endif$"
            ),
        )
        self.assertRegex(
            get_user_code_section(main, "3"),
            r"^ads8688_process\(\);\s*hmi_tjc_process\(\);\s*\}$",
        )

    def test_system_c_initializes_ads8688_through_unified_header(self):
        """system.c 必须只包含 system.h，并通过 system_init() 启动 ADS8688。"""
        source = (user_dir / "system.c").read_text(encoding="utf-8")
        quoted_includes = re.findall(
            r'^\s*#include\s+"([^"]+)"',
            source,
            re.MULTILINE,
        )
        body = get_function_body(source, "system_init")

        self.assertEqual(quoted_includes, ["system.h"])
        self.assertIsNotNone(body)
        self.assertRegex(
            body,
            (
                r"measurement_result_init\s*\(\s*\)\s*;[\s\S]*?"
                r"hmi_tjc_init\s*\(\s*\)\s*;[\s\S]*?"
                r"\(void\)\s*ads8688_init\s*\(\s*\)\s*;"
            ),
        )

    def test_user_modules_have_required_chinese_headers(self):
        """所有用户 C/H 文件必须具备中文模块说明和调用说明。"""
        required_phrases = (
            "模块用途",
            "GPIO 引脚映射",
            "依赖的外设和 CubeIDE 配置",
            "初始化方法",
            "调用方法",
        )

        for path in sorted(user_dir.glob("*.[ch]")):
            text = path.read_text(encoding="utf-8")
            with self.subTest(file=path.name):
                for phrase in required_phrases:
                    self.assertIn(phrase, text)

    def test_readme_documents_ads8688_integration(self):
        """README 必须说明硬件连接、CubeIDE 配置、使用方法和未上板验证项。"""
        readme = (project_root / "README.md").read_text(encoding="utf-8")
        required_phrases = (
            "STM32H750VBT6",
            "STM32CubeIDE 1.19.0",
            "PB12",
            "PB13",
            "PB14",
            "PB15",
            "PD8",
            "PD9",
            "自动循环扫描",
            "单通道采集",
            "内部 4.096 V 基准",
            "16.125 MHz",
            "约 393 kSPS",
            "4096",
            "200 kSPS",
            "编译",
            "烧录",
            "运行",
            "验证",
            "已知限制",
            "尚未完成实板验证",
            "TJC4827T143_011R_I_P20",
            "USART1",
            "PA9",
            "PA10",
            "115200",
            "measurement_result_publish",
            "hmi_tjc_process",
        )

        for phrase in required_phrases:
            with self.subTest(phrase=phrase):
                self.assertIn(phrase, readme)

    def test_measurement_result_contract_is_declared(self):
        """算法与显示之间必须只通过测量结果快照接口传递数据。"""
        header = (user_dir / "measurement_result.h").read_text(encoding="utf-8")
        result_match = re.search(
            r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*measurement_result_t\s*;",
            header,
            flags=re.DOTALL,
        )

        self.assertIsNotNone(result_match, "缺少 measurement_result_t")
        result_body = result_match.group("body")
        required_fields = {
            "amplitude_vpp": "float",
            "frequency_hz": "float",
            "phase_deg": "float",
            "wave_type": "measurement_wave_type_t",
            "valid": "uint8_t",
            "sequence": "uint32_t",
        }
        for field_name, field_type in required_fields.items():
            with self.subTest(field=field_name):
                self.assertRegex(
                    result_body,
                    rf"\b{field_type}\s+{field_name}\s*;",
                )

        for declaration in (
            "void measurement_result_init(void);",
            "void measurement_result_publish(const measurement_result_t *result);",
            "uint8_t measurement_result_get_snapshot(measurement_result_t *result);",
        ):
            with self.subTest(declaration=declaration):
                self.assertTrue(has_exact_declaration(header, declaration))

    def test_hmi_tjc_frame_contract_is_preserved(self):
        """串口屏帧必须固定更新五个文本控件并使用原始三字节结束符。"""
        header = strip_c_comments(
            (user_dir / "hmi_tjc.h").read_text(encoding="utf-8")
        )
        source = strip_c_comments(
            (user_dir / "hmi_tjc.c").read_text(encoding="utf-8")
        )

        self.assertRegex(
            header,
            r"(?m)^[ \t]*#define[ \t]+HMI_TJC_REFRESH_MS[ \t]+100u[ \t]*$",
        )
        self.assertRegex(
            header,
            r"(?m)^[ \t]*#define[ \t]+HMI_TJC_TX_BUFFER_SIZE[ \t]+192u[ \t]*$",
        )
        for control_name in ("t_amp", "t_freq", "t_phase", "t_wave", "t_status"):
            with self.subTest(control=control_name):
                self.assertIn(f'"{control_name}"', source)

        self.assertEqual(
            source.count("frame[offset++] = HMI_TJC_TERMINATOR_BYTE;"),
            3,
        )
        self.assertIn("HAL_UART_Transmit_DMA", source)
        self.assertIn("HAL_UART_TxCpltCallback", source)
        self.assertIn("snprintf", source)
        self.assertNotIn("sprintf(", source)

    def test_hmi_callbacks_only_mark_hmi_state(self):
        """UART 回调不得格式化文本、读取 ADS 数据或重启传输。"""
        source = (user_dir / "hmi_tjc.c").read_text(encoding="utf-8")

        for callback_name, flag_name in (
            ("HAL_UART_TxCpltCallback", "hmi_tjc_tx_complete_flag"),
            ("HAL_UART_ErrorCallback", "hmi_tjc_uart_error_flag"),
        ):
            with self.subTest(callback=callback_name):
                body = get_function_body(source, callback_name)
                self.assertIsNotNone(body)
                self.assertRegex(body, rf"\b{flag_name}\s*=\s*1u\s*;")
                for forbidden in (
                    "snprintf",
                    "HAL_UART_Transmit_DMA",
                    "HAL_UART_AbortTransmit",
                    "measurement_result",
                    "ads8688",
                ):
                    self.assertNotIn(forbidden, body)

    def test_hmi_page_document_exists(self):
        """HMI 页面工程必须有可交接的控件和串口配置说明。"""
        hmi_readme_path = project_root / "hmi" / "README.md"
        self.assertTrue(hmi_readme_path.is_file())
        hmi_readme = hmi_readme_path.read_text(encoding="utf-8")

        for phrase in (
            "TJC4827T143_011R_I_P20",
            "480 x 272",
            "115200",
            "vscope",
            "txt_maxl",
            "t_amp",
            "t_freq",
            "t_phase",
            "t_wave",
            "t_status",
            "FF FF FF",
            "USART1",
            "PA9",
            "PA10",
            "DMA1 Stream0",
            "DMA1 Stream1",
            "DMA1 Stream2",
            "待硬件验证",
        ):
            with self.subTest(phrase=phrase):
                self.assertIn(phrase, hmi_readme)

    def test_ads8688_public_apis_are_declared(self):
        """ads8688.h 必须声明全部公共 ADS8688 API。"""
        header = (user_dir / "ads8688.h").read_text(encoding="utf-8")
        required_apis = (
            "ads8688_init",
            "ads8688_process",
            "ads8688_set_auto_mode",
            "ads8688_set_manual_mode",
            "ads8688_set_channel_range",
            "ads8688_get_latest",
            "ads8688_read_history",
            "ads8688_clear_history",
            "ads8688_get_diagnostics",
        )

        for api_name in required_apis:
            with self.subTest(api=api_name):
                self.assertTrue(
                    has_function_prototype(header, api_name),
                    f"ads8688.h 未声明 {api_name}()",
                )

    def test_ads8688_dma_contract_is_declared(self):
        """DMA 缓冲长度、回调标志及统一头文件 extern 声明必须完整。"""
        source = strip_c_comments(
            (user_dir / "ads8688.c").read_text(encoding="utf-8")
        )
        system_header = strip_c_comments(
            (user_dir / "system.h").read_text(encoding="utf-8")
        )

        self.assertRegex(
            source,
            r"(?m)^[ \t]*#define[ \t]+ADS8688_DMA_WORD_COUNT[ \t]+1024u[ \t]*$",
        )
        for flag_name in (
            "ads8688_dma_half_flag",
            "ads8688_dma_full_flag",
            "ads8688_error_flag",
        ):
            with self.subTest(flag=flag_name):
                self.assertRegex(
                    source,
                    rf"\bvolatile\s+uint8_t\s+{flag_name}\s*;",
                )
                self.assertRegex(
                    system_header,
                    rf"\bextern\s+volatile\s+uint8_t\s+{flag_name}\s*;",
                )

    def test_ads8688_get_diagnostics_has_full_prototype(self):
        """诊断接口的返回值和输出参数类型必须与设计一致。"""
        header = (user_dir / "ads8688.h").read_text(encoding="utf-8")
        self.assertTrue(
            has_exact_declaration(
                header,
                (
                    "ads8688_status_t ads8688_get_diagnostics("
                    "ads8688_diagnostics_t *diagnostics);"
                ),
            )
        )

    def test_ads8688_callbacks_only_set_their_flags(self):
        """三个 HAL 回调只能处理未使用参数并置位各自的单一标志。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")
        callbacks = {
            "HAL_SPI_TxRxHalfCpltCallback": "ads8688_dma_half_flag",
            "HAL_SPI_TxRxCpltCallback": "ads8688_dma_full_flag",
            "HAL_SPI_ErrorCallback": "ads8688_error_flag",
        }
        forbidden_calls = (
            "for",
            "while",
            "HAL_Delay",
            "printf",
            "ads8688_storage",
            "ads8688_process",
            "recover",
        )

        for callback_name, flag_name in callbacks.items():
            with self.subTest(callback=callback_name):
                body = get_function_body(source, callback_name)
                self.assertIsNotNone(body, f"缺少回调 {callback_name}()")
                self.assertRegex(body, rf"\b{flag_name}\s*=\s*1u\s*;")
                assignments = re.findall(r"(?<![=!<>])=(?!=)", body)
                self.assertEqual(assignments, ["="], "回调中存在额外赋值")
                for forbidden in forbidden_calls:
                    self.assertNotRegex(
                        body,
                        rf"\b{re.escape(forbidden)}",
                        f"回调中禁止出现 {forbidden}",
                    )

    def test_range_change_resynchronizes_auto_channel_tracking(self):
        """量程修改发送 AUTO_RST 后必须重置自动模式的软件通道跟踪。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")
        body = get_function_body(source, "ads8688_set_channel_range")

        self.assertIsNotNone(body, "缺少 ads8688_set_channel_range()")
        self.assertRegex(
            body,
            (
                r"ads8688_mode\s*==\s*ADS8688_MODE_AUTO[\s\S]*?"
                r"ads8688_current_channel\s*=\s*"
                r"ads8688_find_first_channel\s*\(\s*ads8688_channel_mask\s*\)"
            ),
        )

    def test_dma_stop_uses_checked_blocking_abort(self):
        """STM32H7 必须使用受检查的阻塞 Abort，禁止调用不支持的 DMAStop。"""
        source = strip_c_comments(
            (user_dir / "ads8688.c").read_text(encoding="utf-8")
        )
        body = get_function_body(source, "ads8688_stop_dma")

        self.assertNotIn("HAL_SPI_DMAStop", source)
        self.assertIsNotNone(body, "缺少 ads8688_stop_dma()")
        self.assertRegex(body, r"HAL_SPI_Abort\s*\(\s*&hspi2\s*\)")
        self.assertRegex(
            body,
            (
                r"HAL_SPI_Abort\s*\(\s*&hspi2\s*\)\s*!=\s*HAL_OK"
                r"[\s\S]*?return\s+ADS8688_STATUS_HAL_ERROR\s*;"
            ),
        )
        self.assertRegex(body, r"return\s+ADS8688_STATUS_OK\s*;")

    def test_dma_flags_are_claimed_before_half_processing(self):
        """主循环必须先在领取函数中清 flag，再处理耗时的 512 个样本。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")
        body = get_function_body(source, "ads8688_process")

        self.assertIsNotNone(body, "缺少 ads8688_process()")
        for flag_name, half_index in (
            ("ads8688_dma_half_flag", "0u"),
            ("ads8688_dma_full_flag", "1u"),
        ):
            with self.subTest(flag=flag_name):
                claim_match = re.search(
                    (
                        r"ads8688_claim_flag\s*\(\s*&"
                        rf"{flag_name}\s*\)"
                    ),
                    body,
                )
                process_match = re.search(
                    rf"ads8688_process_dma_half\s*\(\s*{half_index}\s*\)",
                    body,
                )
                self.assertIsNotNone(claim_match, f"未先领取 {flag_name}")
                self.assertIsNotNone(process_match, f"未处理 DMA 半区 {half_index}")
                self.assertLess(claim_match.start(), process_match.start())

    def test_recovery_failure_remains_pending_for_process_retry(self):
        """恢复失败必须保留 pending，并允许后续 process 在未初始化状态下重试。"""
        source = strip_c_comments(
            (user_dir / "ads8688.c").read_text(encoding="utf-8")
        )
        process_body = get_function_body(source, "ads8688_process")
        recover_body = get_function_body(source, "ads8688_recover")

        self.assertRegex(
            source,
            r"\bstatic\s+uint8_t\s+ads8688_recovery_pending\s*;",
        )
        self.assertIsNotNone(process_body)
        self.assertLess(
            process_body.find("ads8688_recovery_pending"),
            process_body.find("ads8688_initialized"),
            "process 必须先处理 pending，再判断初始化状态",
        )
        self.assertRegex(
            process_body,
            (
                r"ads8688_recovery_pending[\s\S]*?"
                r"ads8688_recover\s*\(\s*\)[\s\S]*?return\s*;"
            ),
        )
        self.assertIsNotNone(recover_body)
        self.assertRegex(
            recover_body,
            r"ads8688_recovery_pending\s*=\s*0u\s*;",
        )

    def test_configuration_restart_failure_rolls_back_old_acquisition(self):
        """三个配置 API 的 DMA restart 失败均必须进入旧配置回滚路径。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")

        for function_name in (
            "ads8688_set_auto_mode",
            "ads8688_set_manual_mode",
            "ads8688_set_channel_range",
        ):
            with self.subTest(function=function_name):
                body = get_function_body(source, function_name)
                self.assertIsNotNone(body)
                self.assertRegex(
                    body,
                    (
                        r"status\s*=\s*ads8688_start_dma\s*\(\s*\)\s*;"
                        r"[\s\S]*?if\s*\(\s*status\s*!=\s*"
                        r"ADS8688_STATUS_OK\s*\)"
                        r"[\s\S]*?ads8688_rollback_after_failure"
                    ),
                )

    def test_all_dma_stop_callers_check_failure_before_transfer(self):
        """配置与恢复路径必须确认 Abort 成功后才进行阻塞传输或复位。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")

        for function_name in (
            "ads8688_recover",
            "ads8688_set_auto_mode",
            "ads8688_set_manual_mode",
            "ads8688_set_channel_range",
        ):
            with self.subTest(function=function_name):
                body = get_function_body(source, function_name)
                self.assertIsNotNone(body)
                self.assertRegex(
                    body,
                    (
                        r"status\s*=\s*ads8688_stop_dma\s*\(\s*\)\s*;"
                        r"[\s\S]*?if\s*\(\s*status\s*!=\s*"
                        r"ADS8688_STATUS_OK\s*\)[\s\S]*?return"
                    ),
                )

    def test_reinitialization_stops_active_or_faulted_dma_before_reset(self):
        """重复初始化或故障后初始化必须确认 Abort 成功，再执行硬件初始化助手。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")
        body = get_function_body(source, "ads8688_init")

        self.assertIsNotNone(body)
        stop_match = re.search(
            r"status\s*=\s*ads8688_stop_dma\s*\(\s*\)\s*;",
            body,
        )
        initialize_match = re.search(
            r"status\s*=\s*ads8688_initialize_attempt\s*\(\s*\)\s*;",
            body,
        )
        self.assertIsNotNone(stop_match, "重复初始化路径未停止 DMA")
        self.assertIsNotNone(initialize_match)
        self.assertLess(stop_match.start(), initialize_match.start())
        self.assertRegex(
            body,
            (
                r"status\s*=\s*ads8688_stop_dma\s*\(\s*\)\s*;"
                r"[\s\S]*?if\s*\(\s*status\s*!=\s*"
                r"ADS8688_STATUS_OK\s*\)[\s\S]*?return\s+status\s*;"
            ),
        )

    def test_configuration_stop_failure_enters_recovery_pending_state(self):
        """三个配置 API 的 Abort 失败必须转入未初始化且待恢复状态。"""
        source = (user_dir / "ads8688.c").read_text(encoding="utf-8")

        for function_name in (
            "ads8688_set_auto_mode",
            "ads8688_set_manual_mode",
            "ads8688_set_channel_range",
        ):
            with self.subTest(function=function_name):
                body = get_function_body(source, function_name)
                self.assertIsNotNone(body)
                self.assertRegex(
                    body,
                    (
                        r"status\s*=\s*ads8688_stop_dma\s*\(\s*\)\s*;"
                        r"[\s\S]*?if\s*\(\s*status\s*!=\s*"
                        r"ADS8688_STATUS_OK\s*\)\s*\{"
                        r"[\s\S]*?ads8688_initialized\s*=\s*0u\s*;"
                        r"[\s\S]*?ads8688_recovery_pending\s*=\s*1u\s*;"
                        r"[\s\S]*?return\s+status\s*;"
                        r"[\s\S]*?\}"
                    ),
                )

    def test_api_name_in_comment_is_not_a_prototype(self):
        """仅在 C 注释中出现的 API 名称不能满足函数原型契约。"""
        comment_only_header = """
        /* 初始化时调用 ads8688_init()。 */
        // 主循环调用 ads8688_process()
        """

        self.assertFalse(
            has_function_prototype(comment_only_header, "ads8688_init")
        )
        self.assertFalse(
            has_function_prototype(comment_only_header, "ads8688_process")
        )

    def test_storage_api_with_wrong_signature_is_rejected(self):
        """存储 API 的返回类型、参数类型与顺序必须完全匹配。"""
        required_declaration = (
            "uint32_t ads8688_storage_read("
            "ads8688_sample_t *samples, uint32_t max_count);"
        )
        wrong_declarations = (
            "int ads8688_storage_read(ads8688_sample_t *samples, uint32_t max_count);",
            "uint32_t ads8688_storage_read(void);",
            "uint32_t ads8688_storage_read(uint32_t max_count, ads8688_sample_t *samples);",
        )

        for wrong_declaration in wrong_declarations:
            with self.subTest(declaration=wrong_declaration):
                self.assertFalse(
                    has_exact_declaration(
                        wrong_declaration,
                        required_declaration,
                    )
                )

    def test_ads8688_sample_contains_required_fields(self):
        """ads8688_sample_t 必须包含设计规定的四个字段。"""
        header = (user_dir / "ads8688.h").read_text(encoding="utf-8")
        sample_match = re.search(
            r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*ads8688_sample_t\s*;",
            header,
            flags=re.DOTALL,
        )

        self.assertIsNotNone(sample_match, "ads8688.h 未定义 ads8688_sample_t")
        sample_body = sample_match.group("body")
        required_fields = {
            "sample_index": "uint32_t",
            "raw_code": "uint16_t",
            "channel": "uint8_t",
            "reserved": "uint8_t",
        }
        for field_name, field_type in required_fields.items():
            with self.subTest(field=field_name):
                self.assertRegex(
                    sample_body,
                    rf"\b{field_type}\s+{field_name}\s*;",
                    f"ads8688_sample_t 缺少字段 {field_type} {field_name}",
                )


    def test_ads8688_storage_constants_are_declared(self):
        """ads8688_storage.h 必须声明规定的容量和通道数量常量。"""
        header = strip_c_comments(
            (user_dir / "ads8688_storage.h").read_text(encoding="utf-8")
        )

        self.assertRegex(
            header,
            r"(?m)^[ \t]*#define[ \t]+ADS8688_HISTORY_CAPACITY[ \t]+4096u[ \t]*$",
        )
        self.assertRegex(
            header,
            r"(?m)^[ \t]*#define[ \t]+ADS8688_CHANNEL_COUNT[ \t]+8u[ \t]*$",
        )

    def test_ads8688_storage_apis_are_declared(self):
        """ads8688_storage.h 必须声明全部内部存储 API。"""
        header = (user_dir / "ads8688_storage.h").read_text(encoding="utf-8")
        required_declarations = (
            "void ads8688_storage_init(void);",
            (
                "void ads8688_storage_push(uint8_t channel, "
                "uint16_t raw_code, ads8688_range_t range, "
                "uint32_t sample_index);"
            ),
            (
                "ads8688_status_t ads8688_storage_get_latest("
                "uint8_t channel, ads8688_latest_t *latest);"
            ),
            (
                "uint32_t ads8688_storage_read("
                "ads8688_sample_t *samples, uint32_t max_count);"
            ),
            "void ads8688_storage_clear(void);",
            "uint32_t ads8688_storage_get_overwrite_count(void);",
        )

        for declaration in required_declarations:
            with self.subTest(declaration=declaration):
                self.assertTrue(
                    has_exact_declaration(header, declaration),
                    f"ads8688_storage.h 未声明完整接口：{declaration}",
                )


    def test_ads8688_transport_constants_are_defined(self):
        """ads8688.c 必须定义寄存器传输所需的精确协议常量。"""
        source_path = user_dir / "ads8688.c"
        self.assertTrue(source_path.is_file(), "缺少 Core/User/ads8688.c")
        source = strip_c_comments(source_path.read_text(encoding="utf-8"))
        required_constants = {
            "ADS8688_COMMAND_NO_OP": "0x0000u",
            "ADS8688_COMMAND_AUTO_RST": "0xa000u",
            "ADS8688_REGISTER_AUTO_SEQUENCE": "0x01u",
            "ADS8688_REGISTER_FEATURE_SELECT": "0x03u",
            "ADS8688_REGISTER_RANGE_CH0": "0x05u",
            "ADS8688_REGISTER_RANGE_CH7": "0x0cu",
        }

        for constant_name, constant_value in required_constants.items():
            with self.subTest(constant=constant_name):
                self.assertRegex(
                    source,
                    (
                        rf"(?m)^[ \t]*#define[ \t]+{constant_name}"
                        rf"[ \t]+{constant_value}[ \t]*$"
                    ),
                )


if __name__ == "__main__":
    unittest.main()
