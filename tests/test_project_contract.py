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
        "",
        source,
        flags=re.DOTALL,
    )


class ProjectContractTest(unittest.TestCase):
    """验证 ADS8688 用户模块对外公开的头文件与接口契约。"""

    def test_required_headers_exist(self):
        """三个用户公共头文件必须位于 Core/User 目录。"""
        for header_name in ("ads8688.h", "ads8688_storage.h", "system.h"):
            with self.subTest(header=header_name):
                self.assertTrue(
                    (user_dir / header_name).is_file(),
                    f"缺少必需头文件 Core/User/{header_name}",
                )

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
        )

        for api_name in required_apis:
            with self.subTest(api=api_name):
                self.assertTrue(
                    has_function_prototype(header, api_name),
                    f"ads8688.h 未声明 {api_name}()",
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
        required_apis = (
            "ads8688_storage_init",
            "ads8688_storage_push",
            "ads8688_storage_get_latest",
            "ads8688_storage_read",
            "ads8688_storage_clear",
            "ads8688_storage_get_overwrite_count",
        )

        for api_name in required_apis:
            with self.subTest(api=api_name):
                self.assertTrue(
                    has_function_prototype(header, api_name),
                    f"ads8688_storage.h 未声明 {api_name}()",
                )


if __name__ == "__main__":
    unittest.main()
