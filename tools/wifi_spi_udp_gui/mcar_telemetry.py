"""麦轮固件遥测协议；不依赖 GUI，便于测试和复用。

控制消息是以 MCAR 开头的 ASCII UDP 数据报；数据是 JustFloat。
订阅通过暂停 -> SUB -> RATE -> 恢复逐步确认，超时有限重试。
"""
from __future__ import annotations

import math
import re
import struct

TAIL = b"\x00\x00\x80\x7f"
PRESETS = {
    "IMU": ["roll_deg", "pitch_deg", "yaw_deg"],
    "定位": ["x_cm", "y_cm", "nav_yaw_deg", "goal_x_cm", "goal_y_cm", "goal_yaw_deg",
             "cmd_vx_cmps", "cmd_vy_cmps", "cmd_omega_radps", "distance_cm", "pos_status", "nav_valid"],
    "编码器": ["ul_raw", "ur_raw", "dl_raw", "dr_raw", "ul_cmps", "ur_cmps", "dl_cmps", "dr_cmps"],
    "轮速环": ["ul_target", "ur_target", "dl_target", "dr_target", "ul_filt", "ur_filt", "dl_filt",
              "dr_filt", "ul_pwm", "ur_pwm", "dl_pwm", "dr_pwm"],
}


def parse_names(text: str) -> list[str]:
    names = [name.strip() for name in text.split(",")]
    if not 1 <= len(names) <= 40 or any(not re.fullmatch(r"[a-z][a-z0-9_]*", name) for name in names):
        raise ValueError("上传变量需为 1～40 个名称，以英文逗号分隔")
    if len(set(names)) != len(names):
        raise ValueError("上传变量不能重复")
    return names


def control_reply(packet: bytes) -> tuple[str, str] | None:
    if packet.endswith(TAIL) or not packet.startswith(b"MCAR "):
        return None
    if not packet.endswith(b"\n"):
        raise ValueError("MCAR 应答缺少换行")
    text = packet.decode("ascii").strip()
    parts = text.split(" ", 2)
    if len(parts) != 3 or parts[1] not in {"SUB", "LIST", "RATE", "STREAM", "ERR", "SLIDER"}:
        raise ValueError("无效的 MCAR 应答")
    if parts[1] == "SLIDER":
        parameter = parts[2].split()
        if len(parameter) != 2 or not math.isfinite(float(parameter[1])):
            raise ValueError("无效的调参确认")
    return parts[1], parts[2]


def validate_frame(packet: bytes, channel_count: int) -> None:
    if not 1 <= channel_count <= 40 or len(packet) != 4 * channel_count + 4 or not packet.endswith(TAIL):
        raise ValueError("JustFloat 长度/帧尾与当前订阅不符，丢弃数据，请读取当前配置")
    if not all(math.isfinite(x) for x in struct.unpack("<" + "f" * channel_count, packet[:-4])):
        raise ValueError("遥测包含非有限数值，丢弃此帧")


class Subscription:
    """主线程驱动状态机；pending 时丢弃数据，仅在对应应答确认后推进。

    所有命令均幂等。任何阶段失败都停止推进，不自动恢复上传。
    UDP 无顺序保证；协议没有帧序号，极端延迟旧包无法完全识别。
    """
    def __init__(self) -> None:
        self.pending = False
        self.error = ""
        self.commands: list[str] = []
        self.index = 0
        self.attempts = 0
        self.deadline = 0.0

    def begin(self, text: str, period: int, now: float) -> str:
        if self.pending:
            raise ValueError("正在等待设备确认，请稍候")
        names = parse_names(text)
        if not 2 <= period <= 1000:
            raise ValueError("周期范围为 2～1000 ms")
        self.commands = ["STREAM 0", "SUB " + ",".join(names), f"RATE {period}", "STREAM 1"]
        self.index, self.error, self.pending = 0, "", True
        return self._send(now)

    def _send(self, now: float) -> str:
        self.deadline, self.attempts = now + 1.0, 1
        return self.commands[self.index] + "\n"

    def abort(self, message: str) -> None:
        self.error, self.pending = message, False

    def acknowledge(self, kind: str, value: str, now: float) -> str | None:
        if not self.pending:
            return None
        if kind == "ERR":
            self.abort("设备拒绝配置：" + value)
            return None
        if f"{kind} {value}" != self.commands[self.index]:
            return None  # 重复/乱序应答不能推进后续阶段。
        self.index += 1
        if self.index == len(self.commands):
            self.pending = False
            return None
        return self._send(now)

    def poll(self, now: float) -> str | None:
        if not self.pending or now < self.deadline:
            return None
        if self.attempts >= 3:
            self.abort("等待设备应答超时，上传可能已暂停；检查模块 IP 后重新应用")
            return None
        self.attempts += 1
        self.deadline = now + 1.0
        return self.commands[self.index] + "\n"
