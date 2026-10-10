"""python -m unittest test_mcar_telemetry.py；无需 GUI/硬件。"""
import math
from pathlib import Path
import re
import socket
import struct
import unittest

from mcar_telemetry import PRESETS, Subscription, TAIL, control_reply, parse_names, validate_frame
from udp_receiver import UdpReceiver


class ProtocolTest(unittest.TestCase):
    def test_bound_socket_round_trip(self):
        receiver = UdpReceiver()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as module:
            module.bind(("127.0.0.1", 0))
            module.settimeout(1)
            receiver.start("127.0.0.1", 0)
            try:
                listening_port = receiver._socket.getsockname()[1]
                receiver.send_bound(b"GET?\n", "127.0.0.1", module.getsockname()[1])
                request, peer = module.recvfrom(1024)
                self.assertEqual(request, b"GET?\n")
                self.assertEqual(peer[1], listening_port)
                module.sendto(b"MCAR STREAM 1\n", peer)
                kind, payload = receiver.events.get(timeout=1)
                if kind == "status":
                    kind, payload = receiver.events.get(timeout=1)
                self.assertEqual(kind, "packet")
                self.assertEqual(payload[1], b"MCAR STREAM 1\n")
            finally:
                receiver.stop()
        with self.assertRaises(OSError):
            receiver.send_bound(b"GET?\n", "127.0.0.1", 5001)

    def test_fields_and_firmware_presets(self):
        self.assertEqual(parse_names(" x_cm, y_cm "), ["x_cm", "y_cm"])
        for text in ("", "x_cm,", "x_cm,x_cm", "x_cm;RUN 1", ",".join(f"x{i}" for i in range(41))):
            with self.assertRaises(ValueError):
                parse_names(text)
        firmware = Path(__file__).resolve().parents[2] / "project/code/wifispi.c"
        names = set(re.findall(r"X\(([a-z][a-z0-9_]*),", firmware.read_text(encoding="utf-8")))
        self.assertGreater(len(names), 40)
        for preset in PRESETS.values():
            self.assertTrue(set(preset) <= names)

    def test_frames_and_replies_are_distinct(self):
        data = struct.pack("<fff", 25, -50, 40) + TAIL
        validate_frame(data, 3)
        self.assertIsNone(control_reply(data))
        self.assertIsNone(control_reply(b"MCAR abc" + TAIL))  # 浮点字节碰巧与文本前缀相同。
        self.assertEqual(control_reply(b"MCAR SUB x_cm,y_cm\n"), ("SUB", "x_cm,y_cm"))
        self.assertEqual(control_reply(b"MCAR SLIDER pos_xy_kp 2.501\n"), ("SLIDER", "pos_xy_kp 2.501"))
        for invalid in (b"MCAR SLIDER kp nan\n", b"MCAR SLIDER kp\n"):
            with self.assertRaises(ValueError):
                control_reply(invalid)
        for packet in (data[:-4], data, b"MCAR SUB x_cm\n", struct.pack("<ff", math.nan, 1) + TAIL):
            with self.assertRaises(ValueError):
                validate_frame(packet, 2)
        with self.assertRaises(ValueError):
            control_reply(b"MCAR BAD thing\n")
        with self.assertRaises(ValueError):
            validate_frame(TAIL, 0)

    def test_confirmed_subscription_and_retries(self):
        state = Subscription()
        self.assertEqual(state.begin("x_cm,y_cm", 20, 0), "STREAM 0\n")
        self.assertIsNone(state.acknowledge("SUB", "x_cm,y_cm", 0.1))  # 乱序
        self.assertEqual(state.poll(1), "STREAM 0\n")
        self.assertEqual(state.acknowledge("STREAM", "0", 1.1), "SUB x_cm,y_cm\n")
        self.assertIsNone(state.acknowledge("STREAM", "0", 1.2))  # 重复
        self.assertEqual(state.acknowledge("SUB", "x_cm,y_cm", 1.3), "RATE 20\n")
        self.assertEqual(state.acknowledge("RATE", "20", 1.4), "STREAM 1\n")
        self.assertIsNone(state.acknowledge("STREAM", "1", 1.5))
        self.assertFalse(state.pending)
        state.begin("x_cm", 10, 2)
        with self.assertRaises(ValueError):
            state.begin("y_cm", 10, 2)
        state.acknowledge("ERR", "unknown variable", 2.1)
        self.assertFalse(state.pending)
        self.assertIn("unknown variable", state.error)
        state.begin("x_cm", 10, 3)
        self.assertEqual(state.poll(4), "STREAM 0\n")
        self.assertEqual(state.poll(5), "STREAM 0\n")
        self.assertIsNone(state.poll(6))
        self.assertFalse(state.pending)
        self.assertIn("超时", state.error)


if __name__ == "__main__":
    unittest.main()
