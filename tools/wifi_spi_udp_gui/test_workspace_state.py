"""工作区持久化与十进制步长回归测试，不启动 GUI、不连接实车。"""
from decimal import Decimal
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from slider_protocol import encode_slider_packet, snap_value, validate_range
from workspace_state import WorkspaceStore


class WorkspaceTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.store = WorkspaceStore(Path(self.directory.name) / "workspace.json")

    def test_full_state_round_trip_and_empty_list(self):
        state = {"version": 1, "sliders": [{"name": "位置参数", "value": "2.501",
                 "minimum": "0", "maximum": "10", "step": "0.001"}],
                 "controls": {"bind_port": 8081}, "rules": [{"name": "roll"}]}
        self.store.save(state)
        self.assertEqual(self.store.load(), (state, ""))
        empty = {"version": 1, "sliders": []}
        self.store.save(empty)
        self.assertEqual(self.store.load(), (empty, ""))

    def test_backup_recovers_corrupt_main(self):
        old = {"version": 1, "sliders": [{"name": "kp", "value": "2.5"}]}
        self.store.save(old)
        self.store.save({"version": 1, "sliders": []})
        # 故障注入仅针对临时目录中的测试文件。
        self.store.path.write_bytes(b"broken")
        state, warning = self.store.load()
        self.assertEqual(state, old)
        self.assertIn("备份", warning)
        self.store.save(old)
        self.assertEqual(json.loads(self.store.backup.read_text(encoding="utf-8")), old)

    def test_replace_failure_preserves_original(self):
        old = {"version": 1, "sliders": []}
        self.store.save(old)
        with patch("workspace_state.os.replace", side_effect=OSError("模拟写入失败")):
            with self.assertRaises(OSError):
                self.store.save({"version": 1, "sliders": [{"name": "new"}]})
        self.assertEqual(self.store.load()[0], old)
        self.assertEqual(list(self.store.path.parent.glob("*.tmp")), [])

    def test_missing_and_invalid_state(self):
        self.assertEqual(self.store.load(), (None, ""))
        for state in ({"version": 2, "sliders": []}, {"version": 1, "sliders": "bad"}):
            self.store.save(state)
            self.assertIsNone(self.store.load()[0])

    def test_exact_steps_and_packet(self):
        limits = validate_range(0, 10, .001)
        value = Decimal("2.5")
        for _ in range(1000):
            value = snap_value(value + limits[2], *limits)
        self.assertEqual(value, Decimal("3.5"))
        self.assertEqual(encode_slider_packet("pos_xy_kp", value), b"[slider,pos_xy_kp,3.5]")
        self.assertEqual(snap_value(11, *limits), Decimal("10"))
        self.assertEqual(snap_value(-1, *limits), Decimal("0"))
        with self.assertRaises(ValueError):
            validate_range(0, 1, .0001)


if __name__ == "__main__":
    unittest.main()
