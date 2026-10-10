"""调参滑杆的步长计算与 UDP 文本协议；不依赖 GUI，便于单独测试。"""

from __future__ import annotations

from decimal import Decimal, InvalidOperation, ROUND_FLOOR, ROUND_HALF_UP


MIN_STEP = Decimal("0.001")


def _number(value: object, label: str) -> Decimal:
    """将界面数值转成有限十进制数，避免 float 二进制误差进入协议。"""
    try:
        result = Decimal(str(value))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{label}不是有效数字") from exc
    if not result.is_finite():
        raise ValueError(f"{label}必须是有限数字")
    return result


def validate_range(minimum: object, maximum: object, step: object) -> tuple[Decimal, Decimal, Decimal]:
    """步长最小 0.001；范围必须严格递增。"""
    low, high, increment = (_number(minimum, "最小值"), _number(maximum, "最大值"),
                            _number(step, "步长"))
    if high <= low:
        raise ValueError("最大值必须大于最小值")
    if increment < MIN_STEP:
        raise ValueError("步长不能小于 0.001")
    if increment > high - low:
        raise ValueError("步长不能大于滑杆范围")
    return low, high, increment


def snap_value(value: object, minimum: object, maximum: object, step: object) -> Decimal:
    """以最小值为网格起点，四舍五入到最近步长并限制在范围内。"""
    low, high, increment = validate_range(minimum, maximum, step)
    raw = max(low, min(high, _number(value, "当前值")))
    index = ((raw - low) / increment).to_integral_value(rounding=ROUND_HALF_UP)
    snapped = low + index * increment
    if snapped > high:
        last_index = ((high - low) / increment).to_integral_value(rounding=ROUND_FLOOR)
        snapped = low + last_index * increment
    return snapped


def number_text(value: object) -> str:
    """固定小数表示，不发科学计数法，也不保留无意义的尾零。"""
    number = _number(value, "数值")
    return "0" if number == 0 else format(number.normalize(), "f")


def encode_slider_packet(name: str, value: object) -> bytes:
    """一个滑杆更新对应一个 UTF-8 UDP 报文：[slider,参数名,数值]。"""
    field = name.strip()
    if not field or any(char in field for char in ",[]\r\n\x00") or any(ord(char) < 32 for char in field):
        raise ValueError("参数名不能为空，也不能包含逗号、方括号或控制字符")
    if len(field.encode("utf-8")) > 48:
        raise ValueError("参数名不能超过 48 个 UTF-8 字节")
    return f"[slider,{field},{number_text(value)}]".encode("utf-8")
