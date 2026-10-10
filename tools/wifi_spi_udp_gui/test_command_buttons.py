"""命令按钮状态机回归：边沿、失败重试、协议格式和固定抬起目标。"""
import unittest
from button_protocol import ButtonConfig, ButtonState, command_bytes


class CommandButtonTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.send = lambda packet, endpoint: self.sent.append((packet, endpoint))
        self.endpoint = ("127.0.0.1", 5001)

    def test_momentary_edges_only(self):
        state = ButtonState()
        config = ButtonConfig(press_command="STREAM 1", release_command="STREAM 0")
        state.press(config, self.endpoint, self.send)
        state.press(config, self.endpoint, self.send)
        self.assertTrue(state.pressed)
        self.assertEqual(self.sent, [(b"STREAM 1\n", self.endpoint)])
        state.release(self.send)
        state.release(self.send)
        self.assertEqual(self.sent[-1], (b"STREAM 0\n", self.endpoint))
        self.assertEqual(len(self.sent), 2)

    def test_latch_first_second_third_press(self):
        state = ButtonState()
        config = ButtonConfig(mode="latch", press_command="ON", release_command="OFF", newline=False)
        for expected in (True, False, True):
            state.press(config, self.endpoint, self.send)
            self.assertEqual(state.pressed, expected)
        self.assertEqual([packet for packet, _ in self.sent], [b"ON", b"OFF", b"ON"])

    def test_release_uses_original_config_and_address(self):
        state = ButtonState()
        config = ButtonConfig(press_command="ON", release_command="OFF")
        state.press(config, self.endpoint, self.send)
        config.release_command = "CHANGED"
        state.release(self.send)
        self.assertEqual(self.sent[-1], (b"OFF\n", self.endpoint))

    def test_network_failure_preserves_state_for_retry(self):
        state = ButtonState()
        config = ButtonConfig(press_command="ON", release_command="OFF")
        def broken(packet, endpoint):
            raise OSError("模拟发送失败")
        with self.assertRaises(OSError):
            state.press(config, self.endpoint, broken)
        self.assertFalse(state.pressed)
        state.press(config, self.endpoint, self.send)
        with self.assertRaises(OSError):
            state.release(broken)
        self.assertTrue(state.pressed)
        state.release(self.send)
        self.assertFalse(state.pressed)

    def test_hex_newline_and_empty_edge(self):
        self.assertEqual(command_bytes("AA 55 00 FF", "hex", True), b"\xaa\x55\0\xff")
        self.assertEqual(command_bytes("开始", "text", False), "开始".encode())
        self.assertEqual(command_bytes("GET?\r\n", "text", True), b"GET?\n")
        state = ButtonState()
        state.press(ButtonConfig(press_command="ON", release_command=""), self.endpoint, self.send)
        state.release(self.send)
        self.assertEqual(len(self.sent), 1)
        with self.assertRaises(ValueError):
            ButtonState().press(ButtonConfig(), self.endpoint, self.send)

    def test_invalid_release_rejected_before_press(self):
        with self.assertRaises(ValueError):
            ButtonState().press(ButtonConfig(press_command="AA", release_command="XX", encoding="hex"),
                                self.endpoint, self.send)
        self.assertEqual(self.sent, [])
        with self.assertRaises(ValueError):
            ButtonConfig.from_dict({"mode": "bad"})


if __name__ == "__main__":
    unittest.main()
