import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, function_name: str) -> str:
    """Return the first brace-delimited function body for simple callback checks."""
    start = source.index(f"void {function_name}")
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {function_name}")


class HmiRuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / "Core/User/hmi_task2.c").read_text(
            encoding="utf-8"
        )
        self.header = (ROOT / "Core/User/hmi_task2.h").read_text(
            encoding="utf-8"
        )
        self.chart = (ROOT / "Core/User/hmi_chart.c").read_text(
            encoding="utf-8"
        )
        self.system = (ROOT / "Core/User/system.c").read_text(
            encoding="utf-8"
        )

    def test_hmi_uses_dma_and_cache_safe_static_buffers(self):
        self.assertIn("HAL_UARTEx_ReceiveToIdle_DMA", self.source)
        self.assertIn("HAL_UART_Transmit_DMA", self.source)
        self.assertIn("__attribute__((aligned(32)))", self.source)
        self.assertIn("SCB_CleanDCache_by_Addr", self.source)
        self.assertIn("SCB_InvalidateDCache_by_Addr", self.source)
        self.assertNotIn("malloc(", self.source)

    def test_page_contract_contains_three_curves_and_parameter_texts(self):
        chart_h = (ROOT / "Core/User/hmi_chart.h").read_text(encoding="utf-8")
        for object_name in ("s_t1", "s_t3", "s_spec"):
            self.assertIn(f'"{object_name}"', chart_h)
        for text_name in (
            "t_vpp",
            "t_vrms",
            "t_freq",
            "t_status",
            "t_nihe",
        ):
            self.assertIn(text_name, self.source)
        self.assertIn('"t_comp%u.txt=', self.source)
        self.assertIn("hmi_chart_append_visibility(", self.chart)

    def test_buttons_use_a5_command_5a_without_controlling_fpga(self):
        self.assertIn("#define HMI_TASK2_COMMAND_HEAD        0xa5u", self.source)
        self.assertIn("#define HMI_TASK2_COMMAND_TAIL        0x5au", self.source)
        self.assertIn("#define HMI_TASK2_COMMAND_START       0x04u", self.source)
        self.assertIn(
            "#define HMI_TASK2_COMMAND_MODE_UNUSED 0x10u", self.source
        )
        self.assertIn(
            "#define HMI_TASK2_COMMAND_CALIBRATION 0x20u", self.source
        )
        self.assertIn("hmi_task2_visibility_pending = 1u;", self.source)
        self.assertIn("hmi_task2_invalidate_all_charts();", self.source)
        self.assertIn("hmi_chart_build_visibility", self.source)
        self.assertNotIn("frequency_measure_request_now", self.source)
        self.assertNotIn("dac_output_set_level", self.source)
        self.assertNotIn("fpga_link_process", self.source)

    def test_calibration_button_only_refreshes_scalar_text_and_status(self):
        parse_start = self.source.index(
            "else if (hmi_task2_command_candidate\n"
            "                         == HMI_TASK2_COMMAND_CALIBRATION)"
        )
        parse_end = self.source.index(
            "else if (hmi_task2_command_candidate\n"
            "                    == HMI_TASK2_COMMAND_START)",
            parse_start,
        )
        calibration_branch = self.source[parse_start:parse_end]
        self.assertIn("measurement_calibration_toggle()", calibration_branch)
        self.assertIn("hmi_task2_calibration_pending = 1u;", calibration_branch)
        self.assertIn("hmi_task2_text_valid = 0u;", calibration_branch)
        self.assertNotIn(
            "hmi_task2_waveform_redraw_pending", calibration_branch
        )
        self.assertNotIn("hmi_task2_loaded_valid", calibration_branch)
        self.assertNotIn("fpga_link", calibration_branch)

    def test_calibration_state_uses_t_nihe_gb2312_commands(self):
        self.assertIn(
            r'"t_nihe.txt=\"\xd2\xd1\xd0\xa3\xd7\xbc\""',
            self.source,
        )
        self.assertIn(
            r'"t_nihe.txt=\"\xce\xb4\xd0\xa3\xd7\xbc\""',
            self.source,
        )
        self.assertIn("hmi_task2_append_calibration_state(", self.source)
        select_start = self.source.index(
            "static hmi_tx_action_t hmi_task2_select_action"
        )
        select_end = self.source.index(
            "static void hmi_task2_complete_action", select_start
        )
        select_body = self.source[select_start:select_end]
        self.assertLess(
            select_body.index("HMI_TX_ACTION_CALIBRATION"),
            select_body.index("hmi_task2_work_valid == 0u"),
        )

    def test_numeric_text_uses_calibration_but_charts_do_not(self):
        text_start = self.source.index(
            "static uint8_t hmi_task2_build_text"
        )
        text_end = self.source.index(
            "static uint8_t hmi_task2_generate_triangle_point", text_start
        )
        text_body = self.source[text_start:text_end]
        for apply_function in (
            "measurement_calibration_apply_vpp_uv",
            "measurement_calibration_apply_vrms_uv",
            "measurement_calibration_apply_frequency_mhz",
            "measurement_calibration_apply_component_amplitude_uv",
        ):
            self.assertIn(apply_function, text_body)

        chart_start = self.source.index(
            "static uint8_t hmi_task2_build_action"
        )
        chart_end = self.source.index(
            "static hmi_tx_action_t hmi_task2_select_action", chart_start
        )
        chart_body = self.source[chart_start:chart_end]
        self.assertNotIn("measurement_calibration_apply_", chart_body)

    def test_unused_mode_button_is_recognized_but_has_no_action(self):
        parse_start = self.source.index(
            "static void hmi_task2_parse_commands"
        )
        parse_end = self.source.index(
            "static uint8_t hmi_task2_build_action", parse_start
        )
        parse_body = self.source[parse_start:parse_end]
        self.assertIn(
            "byte == HMI_TASK2_COMMAND_MODE_UNUSED", parse_body
        )
        self.assertIn(
            "== HMI_TASK2_COMMAND_MODE_UNUSED", parse_body
        )
        self.assertIn("静默忽略", parse_body)

    def test_first_snapshot_invalidates_all_three_charts(self):
        first_snapshot = self.source.index(
            "if (hmi_task2_work_valid == 0u)"
        )
        stability_gate = self.source.index(
            "hmi_task2_snapshots_are_stable(", first_snapshot
        )
        self.assertLess(first_snapshot, stability_gate)
        self.assertIn(
            "hmi_task2_work_snapshot, latest",
            self.source[first_snapshot:stability_gate],
        )
        self.assertIn(
            "hmi_task2_invalidate_all_charts();",
            self.source[first_snapshot:stability_gate],
        )
        invalidate_start = self.source.index(
            "static void hmi_task2_invalidate_all_charts"
        )
        replay_start = self.source.index(
            "static void hmi_task2_request_display_replay",
            invalidate_start,
        )
        self.assertIn(
            "hmi_task2_visibility_pending = 1u;",
            self.source[invalidate_start:replay_start],
        )

    def test_chart_is_chunked_and_committed_only_after_final_point(self):
        chart_h = (ROOT / "Core/User/hmi_chart.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("#define HMI_CHART_POINTS_PER_CHUNK 32u", chart_h)
        self.assertIn("hmi_chart_build_waveform_chunk(", self.chart)
        self.assertIn("hmi_task2_chart_next_point", self.source)
        self.assertIn("HMI_TASK2_CHART_GAP_MS", self.source)

        complete_start = self.source.index(
            "static void hmi_task2_complete_action"
        )
        complete_end = self.source.index(
            "static uint8_t hmi_task2_start_tx", complete_start
        )
        complete_body = self.source[complete_start:complete_end]
        final_point_check = complete_body.index(
            ">= MEASUREMENT_DISPLAY_POINT_COUNT"
        )
        loaded_commit = complete_body.index(
            "hmi_task2_loaded_valid[mode] = 1u;"
        )
        self.assertLess(final_point_check, loaded_commit)

    def test_initialization_reveals_both_waveforms_with_one_cycle_in_front(self):
        initialize_start = self.source.index(
            "static uint8_t hmi_task2_build_initialize"
        )
        initialize_end = self.source.index(
            "static uint8_t hmi_task2_build_text", initialize_start
        )
        initialize_body = self.source[initialize_start:initialize_end]
        self.assertIn("hmi_chart_build_visibility(", initialize_body)
        self.assertIn("HMI_CHART_MODE_ONE_CYCLE", initialize_body)
        self.assertNotIn("hmi_chart_build_hide_all(", initialize_body)
        visibility_start = self.chart.index(
            "hmi_chart_status_t hmi_chart_build_visibility"
        )
        visibility_body = self.chart[visibility_start:]
        self.assertIn(
            "? HMI_CHART_T3_OBJECT : HMI_CHART_T1_OBJECT",
            visibility_body,
        )
        self.assertIn(
            "? HMI_CHART_T1_OBJECT : HMI_CHART_T3_OBJECT",
            visibility_body,
        )
        self.assertIn(
            "#define HMI_CHART_HIDE_BACKGROUND_FALLBACK 0u",
            (ROOT / "Core/User/hmi_chart.h").read_text(encoding="utf-8"),
        )
        self.assertIn("background_object,", visibility_body)
        self.assertIn("foreground_object, 0u", visibility_body)
        self.assertIn("foreground_object, 1u", visibility_body)
        self.assertIn("HMI_CHART_SPECTRUM_OBJECT, 1u", visibility_body)

    def test_screen_reconnect_probe_replays_cached_display(self):
        self.assertIn('"sendme"', self.source)
        self.assertIn("HMI_TASK2_PAGE_REPLY_HEAD", self.source)
        self.assertIn("hmi_task2_parse_page_reply_byte(byte);", self.source)
        self.assertIn("hmi_task2_request_display_replay();", self.source)
        self.assertIn("hmi_task2_initialize_done = 0u;", self.source)
        self.assertIn(
            "memset(hmi_task2_loaded_valid, 0, "
            "sizeof(hmi_task2_loaded_valid));",
            self.source,
        )
        self.assertIn("hmi_task2_visibility_pending = 1u;", self.source)
        for field in (
            "probe_count",
            "probe_reply_count",
            "reconnect_count",
            "screen_online",
            "current_page",
        ):
            self.assertIn(field, self.header)

    def test_reconnect_replay_does_not_touch_fpga_transport(self):
        replay_start = self.source.index(
            "static void hmi_task2_request_display_replay"
        )
        replay_end = self.source.index(
            "static void hmi_task2_mark_screen_alive", replay_start
        )
        replay_body = self.source[replay_start:replay_end]
        for forbidden in (
            "fpga_link",
            "HAL_SPI",
            "FPGA_CS",
            "DATA_READY",
        ):
            self.assertNotIn(forbidden, replay_body)

    def test_uart_callbacks_only_set_one_shared_value(self):
        expected = (
            ("HAL_UARTEx_RxEventCallback", "hmi_uart_rx_event_size"),
            ("HAL_UART_TxCpltCallback", "hmi_uart_tx_complete_flag"),
            ("HAL_UART_ErrorCallback", "hmi_uart_error_flag"),
        )
        for function_name, variable_name in expected:
            body = function_body(self.source, function_name)
            assignments = re.findall(
                r"\b(hmi_[a-z0-9_]+)\s*=", body
            )
            self.assertEqual(assignments, [variable_name])
            self.assertNotIn("HAL_UART_", body)

    def test_stable_latch_updates_text_and_all_three_charts(self):
        self.assertIn("hmi_task2_loaded_sequence[4]", self.source)
        self.assertIn("hmi_task2_loaded_valid[4]", self.source)
        self.assertIn("hmi_task2_preload_complete()", self.source)
        self.assertIn("hmi_task2_work_snapshot", self.source)
        self.assertIn("#define HMI_TASK2_STABLE_FRAME_COUNT  3u", self.source)
        self.assertIn("hmi_task2_snapshots_are_stable", self.source)
        self.assertIn("hmi_task2_text_valid = 0u;", self.source)
        stable_refresh = self.source.index(
            "新的稳定输入同时刷新数字、一周期、三周期和频谱"
        )
        next_function = self.source.index(
            "static void hmi_task2_parse_commands", stable_refresh
        )
        self.assertIn(
            "hmi_task2_invalidate_all_charts();",
            self.source[stable_refresh:next_function],
        )
        self.assertIn(
            "&& (hmi_task2_preload_complete() == 0u)",
            self.source,
        )
        self.assertIn("matched_right_mask", self.source)
        self.assertIn("found_match", self.source)
        self.assertIn("无序匹配", self.source)

    def test_period_buttons_only_switch_foreground_and_start_redraws_all(self):
        select_start = self.source.index(
            "static hmi_tx_action_t hmi_task2_select_action"
        )
        select_end = self.source.index(
            "static void hmi_task2_complete_action", select_start
        )
        select_body = self.source[select_start:select_end]
        self.assertLess(
            select_body.index("HMI_TX_ACTION_ONE_CYCLE"),
            select_body.index("return HMI_TX_ACTION_VISIBILITY;"),
        )
        parse_start = self.source.index(
            "static void hmi_task2_parse_commands"
        )
        parse_end = self.source.index(
            "static uint8_t hmi_task2_build_action", parse_start
        )
        parse_body = self.source[parse_start:parse_end]
        self.assertEqual(
            parse_body.count("hmi_task2_invalidate_all_charts();"),
            1,
        )
        self.assertIn(
            "== HMI_TASK2_COMMAND_START",
            parse_body,
        )
        period_branch = parse_body[parse_body.index(
            "hmi_task2_diagnostics.requested_mode ="
        ):]
        self.assertIn("hmi_task2_visibility_pending = 1u;", period_branch)
        self.assertNotIn("hmi_task2_invalidate_all_charts();", period_branch)

    def test_chart_chunks_do_not_change_overlap_layering(self):
        chunk_start = self.chart.index(
            "hmi_chart_status_t hmi_chart_build_waveform_chunk"
        )
        chunk_end = self.chart.index(
            "hmi_chart_status_t hmi_chart_build_visibility", chunk_start
        )
        chunk_body = self.chart[chunk_start:chunk_end]
        self.assertNotIn("hmi_chart_append_visibility(", chunk_body)

    def test_system_pipeline_is_fpga_then_conversion_then_hmi(self):
        fpga_index = self.system.index("fpga_link_process();")
        conversion_index = self.system.index(
            "measurement_conversion_update(fpga_snapshot)"
        )
        hmi_index = self.system.index("hmi_task2_process();")
        self.assertLess(fpga_index, conversion_index)
        self.assertLess(conversion_index, hmi_index)
        for legacy_process in (
            "adc_dual_process();",
            "measurement_fft_process();",
            "frequency_measure_process();",
            "dds_control_process();",
        ):
            self.assertNotIn(legacy_process, self.system)

    def test_normal_mode_keeps_chart_self_test_disabled(self):
        self.assertIn("#define HMI_CHART_SELF_TEST_ENABLE 0u", self.system)
        self.assertIn("hmi_task2_set_chart_self_test(1u);", self.system)

    def test_spectrum_array_is_reversed_for_tjc_scroll_direction(self):
        conversion = (
            ROOT / "Core/User/measurement_conversion.c"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "MEASUREMENT_DISPLAY_POINT_COUNT - 1u - output_index",
            conversion,
        )
        self.assertIn("output[display_index]", conversion)

    def test_non_contiguous_component_slots_follow_valid_flags(self):
        self.assertIn(
            "left_index < FPGA_PROTOCOL_COMPONENT_MAX",
            self.source,
        )
        self.assertIn(
            "right_index < FPGA_PROTOCOL_COMPONENT_MAX",
            self.source,
        )
        self.assertNotIn(
            "index < hmi_task2_work_snapshot.component_count",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
