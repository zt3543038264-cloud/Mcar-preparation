"""可配置命令按钮：只在状态边沿发送，按下时冻结抬起命令与目标地址。"""
from dataclasses import dataclass, asdict
from typing import Callable


@dataclass
class ButtonConfig:
    name: str = "新按键"
    press_command: str = ""
    release_command: str = ""
    mode: str = "momentary"
    encoding: str = "text"
    newline: bool = True

    @classmethod
    def from_dict(cls, data: dict):
        if not isinstance(data, dict):
            raise ValueError("按键配置必须是对象")
        result = cls(**{key: data[key] for key in cls.__dataclass_fields__ if key in data})
        if any(not isinstance(value, str) for value in
               (result.name, result.press_command, result.release_command)):
            raise ValueError("按键名称与命令必须为文本")
        if result.mode not in ("momentary", "latch") or result.encoding not in ("text", "hex"):
            raise ValueError("按键模式或格式无效")
        if not isinstance(result.newline, bool):
            raise ValueError("换行配置无效")
        return result

    def to_dict(self):
        return asdict(self)


def command_bytes(command: str, encoding: str, newline: bool) -> bytes:
    """空命令表示此边沿不发包；文本可选追加 LF，HEX 永远逐字节发送。"""
    if not command.strip():
        return b""
    if encoding == "hex":
        packet = bytes.fromhex(command)
    elif encoding == "text":
        packet = (command.rstrip("\r\n") + "\n" if newline else command).encode("utf-8")
    else:
        raise ValueError("未知命令格式")
    if len(packet) > 65507:
        raise ValueError("命令超过 UDP 数据报最大长度")
    return packet


class ButtonState:
    def __init__(self):
        self.pressed = False
        self.release_packet = b""
        self.endpoint = None

    def press(self, config: ButtonConfig, endpoint: tuple[str, int], send: Callable) -> bool:
        if self.pressed:
            return self.release(send) if config.mode == "latch" else False
        # 两个边沿都提前校验；无效的抬起命令不能在启动之后才发现。
        press = command_bytes(config.press_command, config.encoding, config.newline)
        release = command_bytes(config.release_command, config.encoding, config.newline)
        if not press and not release:
            raise ValueError("请至少填写一条按下或抬起命令")
        if not endpoint[0] or not 1 <= endpoint[1] <= 65535:
            raise ValueError("请设置正确的模块 IP 和端口")
        if press:
            send(press, endpoint)
        # 仅发送成功后切换本机状态；UDP 是否由设备执行仍需设备回执。
        self.release_packet, self.endpoint = release, endpoint
        self.pressed = True
        return True

    def release(self, send: Callable) -> bool:
        if not self.pressed:
            return False
        if self.release_packet:
            send(self.release_packet, self.endpoint)
        # 发送异常时保留按下状态，允许用“抬起”按钮重试。
        self.pressed = False
        self.release_packet, self.endpoint = b"", None
        return True
