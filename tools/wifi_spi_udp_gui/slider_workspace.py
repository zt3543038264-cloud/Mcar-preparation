"""DearPyGui 调参滑杆、单步键盘输入和可恢复的工作区。"""
from __future__ import annotations

from decimal import Decimal
import json
import math
from pathlib import Path
import sys
import tempfile
import time

import dearpygui.dearpygui as dpg
from slider_protocol import encode_slider_packet, number_text, snap_value, validate_range
from workspace_state import WorkspaceStore


class SliderWorkspace:
    def initialize_workspace(self, profile: str, app_file: str, state_path: Path | None = None) -> None:
        self.slider_rows = []
        self.slider_ranges = {}
        self.slider_values = {}
        self.slider_last_sent = {}
        self.selected_slider = None
        self._slider_keys_held = set()
        self.slider_tx_count = 0
        self._last_slider_log = 0.0
        self._last_slider_error = ""
        self._last_slider_error_log = 0.0
        self._last_workspace_poll = 0.0
        self._workspace_signature = None
        self._workspace_ready = False
        self._workspace_temp = None
        if state_path is None:
            if "--self-test" in sys.argv:
                self._workspace_temp = tempfile.TemporaryDirectory(prefix="wifispi-selftest-")
                base = Path(self._workspace_temp.name)
            else:
                base = Path(sys.executable if getattr(sys, "frozen", False) else app_file).resolve().parent
            state_path = base / f"WiFiSPI_UDP_{profile}.workspace.json"
        self.workspace_store = WorkspaceStore(state_path)
        self._saved_workspace, self._workspace_warning = self.workspace_store.load()

    @staticmethod
    def _slider_format(numbers) -> str:
        places = max(3, min(9, max(-number.as_tuple().exponent for number in numbers)))
        return f"%.{places}f"

    def build_slider_ui(self) -> None:
        px = self.px
        with dpg.child_window(height=px(320), width=-1, border=True, tag="shared_slider_panel"):
            dpg.add_text("实时调参滑杆", color=(98, 185, 255))
            dpg.add_text("点击滑杆后：↑ 加一步，↓ 减一步；拖动与按键都会发送，最小步长 0.001。")
            with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp,
                           tag="sliders_table"):
                for label, weight in (("参数名", 1.2), ("最小", 0.9), ("最大", 0.9), ("步长", 0.9),
                                      ("滑杆", 2.5), ("当前值", 0.8), ("操作", 1.8)):
                    dpg.add_table_column(label=label, init_width_or_weight=weight)
            with dpg.group(horizontal=True):
                dpg.add_button(label="＋ 新增滑杆", callback=lambda: self.add_slider_row(), width=px(130))
                dpg.add_button(label="保存工作区", callback=lambda: self.save_workspace(force=True), width=px(130))
                dpg.add_text("修改范围/步长后点“应用”；“发送”可重发当前值。", color=(148, 158, 171))
            dpg.add_text("点击一个滑杆以启用 ↑ / ↓ 单步调节", tag="slider_focus_status")
            dpg.add_text("尚未发送滑杆指令", tag="slider_tx_status", color=(100, 175, 255))
            dpg.add_text("等待设备调参确认", tag="slider_ack_status", color=(115, 210, 145))
            dpg.add_text("工作区自动保存；关闭后保留滑杆及其数值。", tag="workspace_status", color=(148, 158, 171))
        with dpg.handler_registry(tag="slider_keyboard_handlers"):
            dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left, callback=self.on_slider_mouse_click)
            for key, direction in ((dpg.mvKey_Up, 1), (dpg.mvKey_Down, -1)):
                dpg.add_key_press_handler(key=key, callback=self.on_slider_key_press, user_data=direction)
                dpg.add_key_release_handler(key=key, callback=self.on_slider_key_release)

    def add_slider_row(self, name="PARAM", minimum=0, maximum=10, step=0.001, value=0) -> int:
        limits = validate_range(minimum, maximum, step)
        initial = snap_value(value, *limits)
        with dpg.table_row(parent="sliders_table") as row:
            dpg.add_input_text(default_value=name, width=-1, tag=f"slider_name_{row}")
            for suffix, number in zip(("min", "max", "step"), limits):
                options = {"min_value": 0.001, "min_clamped": True} if suffix == "step" else {}
                dpg.add_input_double(default_value=float(number), width=-1, format="%.6f", step=0,
                                     tag=f"slider_{suffix}_{row}", **options)
            dpg.add_slider_double(default_value=float(initial), min_value=float(limits[0]),
                                  max_value=float(limits[1]), format=self._slider_format(limits),
                                  width=-1, tag=f"slider_handle_{row}",
                                  callback=lambda sender, data, user_data: self.on_slider_changed(row, data))
            dpg.add_text(number_text(initial), tag=f"slider_readout_{row}")
            with dpg.group(horizontal=True):
                dpg.add_button(label="应用", callback=lambda: self.apply_slider_range(row))
                dpg.add_button(label="发送", callback=lambda: self.on_slider_changed(row, force=True))
                dpg.add_button(label="删除", callback=lambda: self.delete_slider_row(row))
        self.slider_rows.append(row)
        self.slider_ranges[row] = limits
        self.slider_values[row] = initial
        return row

    def delete_slider_row(self, row) -> None:
        if self.selected_slider == row:
            self.select_slider(None)
        if row in self.slider_rows:
            self.slider_rows.remove(row)
        for mapping in (self.slider_ranges, self.slider_values, self.slider_last_sent):
            mapping.pop(row, None)
        if dpg.does_item_exist(row):
            dpg.delete_item(row)

    def select_slider(self, row) -> None:
        old = self.selected_slider
        if old in self.slider_ranges:
            dpg.configure_item(f"slider_readout_{old}", color=(220, 220, 220))
        self.selected_slider = row
        if row in self.slider_ranges:
            dpg.configure_item(f"slider_readout_{row}", color=(98, 185, 255))
            dpg.set_value("slider_focus_status", f"已选中 {dpg.get_value(f'slider_name_{row}')}：↑ 加一步，↓ 减一步")
        else:
            dpg.set_value("slider_focus_status", "点击一个滑杆以启用 ↑ / ↓ 单步调节")

    def on_slider_mouse_click(self, sender=None, app_data=None, user_data=None) -> None:
        row = next((r for r in self.slider_rows if dpg.is_item_hovered(f"slider_handle_{r}")), None)
        self.select_slider(row)

    def on_slider_key_press(self, sender, app_data, direction) -> None:
        key = int(app_data)
        # Press 在部分后端含自动重复；一次按下到释放只处理一次。
        if key in self._slider_keys_held:
            return
        self._slider_keys_held.add(key)
        row = self.selected_slider
        if row not in self.slider_ranges or self.active_page not in ("params", "plot"):
            return
        # get_focused_item 在 1.11 返回容器窗口，不能用它判定滑杆焦点。
        # 单步按键时才检查可编辑控件；鼠标点到别处也会清除选中状态。
        for item in dpg.get_all_items():
            item_type = dpg.get_item_info(item)["type"]
            if ("mvInput" in item_type or "mvCombo" in item_type) and (
                    dpg.is_item_focused(item) or dpg.is_item_active(item)):
                return
        self.adjust_slider_key(row, direction)

    def on_slider_key_release(self, sender, app_data, user_data=None) -> None:
        self._slider_keys_held.discard(int(app_data))

    def adjust_slider_key(self, row, direction) -> None:
        value = snap_value(self.slider_values[row] + self.slider_ranges[row][2] * direction,
                           *self.slider_ranges[row])
        if value != self.slider_values[row]:
            self.on_slider_changed(row, value, keyboard=True)

    def _set_slider_value(self, row, value) -> None:
        self.slider_values[row] = value
        dpg.set_value(f"slider_handle_{row}", float(value))
        dpg.set_value(f"slider_readout_{row}", number_text(value))

    def apply_slider_range(self, row) -> None:
        try:
            limits = validate_range(*(dpg.get_value(f"slider_{suffix}_{row}") for suffix in ("min", "max", "step")))
            value = snap_value(self.slider_values[row], *limits)
            self.slider_ranges[row] = limits
            self.slider_last_sent.pop(row, None)
            dpg.configure_item(f"slider_handle_{row}", min_value=float(limits[0]), max_value=float(limits[1]),
                               format=self._slider_format(limits))
            self._set_slider_value(row, value)
            self.log(f"已应用滑杆范围与步长：{dpg.get_value(f'slider_name_{row}')}")
        except ValueError as exc:
            self.log(f"滑杆设置无效：{exc}", "error")

    def on_slider_changed(self, row, raw_value=None, force=False, keyboard=False) -> None:
        if row not in self.slider_ranges:
            return
        if not keyboard and not force and (self._slider_keys_held or dpg.is_key_down(dpg.mvKey_Up)
                                          or dpg.is_key_down(dpg.mvKey_Down)):
            self._set_slider_value(row, self.slider_values[row])
            return  # 丢弃 ImGui 内置方向键的非步长增量，由单步处理函数统一更新。
        try:
            raw = self.slider_values[row] if raw_value is None else raw_value
            value = snap_value(raw, *self.slider_ranges[row])
            self._set_slider_value(row, value)
            name = dpg.get_value(f"slider_name_{row}").strip()
            if not force and self.slider_last_sent.get(row) == (name, value):
                return
            packet = encode_slider_packet(name, value)
            ip, port = dpg.get_value("remote_ip").strip(), int(dpg.get_value("remote_port"))
            send = getattr(self.receiver, "send_from_listener", None) or self.receiver.send_bound
            if send(packet, ip, port) != len(packet):
                raise OSError("UDP 报文未完整发送")
            self.slider_last_sent[row] = (name, value)
            self.slider_tx_count += 1
            dpg.set_value("slider_tx_status", f"已发送 {self.slider_tx_count} 包；最近：{packet.decode('utf-8')}")
            self._last_slider_error = ""
            now = time.monotonic()
            if now - self._last_slider_log >= 0.3 or force or keyboard:
                self.log(f"滑杆 -> {ip}:{port}：{packet.decode('utf-8')}", "tx")
                self._last_slider_log = now
        except (ValueError, OSError, RuntimeError) as exc:
            now, message = time.monotonic(), str(exc)
            if message != self._last_slider_error or now - self._last_slider_error_log >= 2:
                self.log(f"滑杆发送失败：{message}", "error")
                self._last_slider_error, self._last_slider_error_log = message, now

    def handle_slider_reply(self, value: str) -> None:
        dpg.set_value("slider_ack_status", f"设备已确认：{value}")

    def capture_workspace(self) -> dict:
        sliders = []
        for row in self.slider_rows:
            low, high, step = self.slider_ranges[row]
            sliders.append({"name": dpg.get_value(f"slider_name_{row}"), "minimum": number_text(low),
                            "maximum": number_text(high), "step": number_text(step),
                            "value": number_text(self.slider_values[row]),
                            "draft": [dpg.get_value(f"slider_{s}_{row}") for s in ("min", "max", "step")]})
        tags = ("bind_ip", "bind_port", "remote_ip", "remote_port", "max_samples", "imu_zoom", "imu_quality",
                "mcar_mode", "mcar_names", "mcar_period", "plot_window", "plot_refresh_hz")
        controls = {tag: dpg.get_value(tag) for tag in tags if dpg.does_item_exist(tag)}
        rules = [{"name": dpg.get_value(f"rule_name_{row}"), "data_type": dpg.get_value(f"rule_type_{row}"),
                  "offset": dpg.get_value(f"rule_offset_{row}"), "byte_order": dpg.get_value(f"rule_order_{row}")}
                 for row in self.rule_rows]
        return {"version": 1, "sliders": sliders, "buttons": self.capture_buttons(), "controls": controls, "rules": rules,
                "plot_selected": [name for name, tag in self.plot_checks.items() if dpg.get_value(tag)],
                "active_page": self.active_page, "view_yaw": self.imu_view_yaw, "view_elev": self.imu_view_elev}

    def restore_workspace(self, defaults) -> None:
        state = self._saved_workspace
        errors = []
        if state is None:
            for args in defaults:
                self.add_slider_row(*args)
        else:
            self.restore_buttons(state.get("buttons", []))  # 兼容旧版只有滑杆的存档。
            for item in state["sliders"]:
                try:
                    row = self.add_slider_row(item["name"], item["minimum"], item["maximum"], item["step"], item["value"])
                    draft = item.get("draft", [])
                    if len(draft) == 3:
                        for suffix, value in zip(("min", "max", "step"), draft):
                            if isinstance(value, (float, int)) and math.isfinite(value):
                                dpg.set_value(f"slider_{suffix}_{row}", value)
                except (KeyError, TypeError, ValueError) as exc:
                    errors.append(str(exc))
            controls = state.get("controls", {})
            if isinstance(controls, dict):
                for tag, value in controls.items():
                    if tag in ("bind_ip", "remote_ip", "imu_quality", "mcar_names", "plot_refresh_hz") and isinstance(value, str):
                        if tag == "plot_refresh_hz" and value not in ("30", "60", "120"):
                            continue
                        if dpg.does_item_exist(tag):
                            dpg.set_value(tag, value)
                    elif tag in ("bind_port", "remote_port", "max_samples", "mcar_period") and isinstance(value, int):
                        if dpg.does_item_exist(tag):
                            dpg.set_value(tag, value)
                    elif tag in ("imu_zoom", "mcar_mode", "plot_window") and isinstance(value, (float, int, bool)) and math.isfinite(value):
                        if dpg.does_item_exist(tag):
                            dpg.set_value(tag, value)
            saved_rules = state.get("rules")
            if isinstance(saved_rules, list) and len(saved_rules) <= 200:
                valid = []
                from packet_parser import TYPE_FORMATS
                for rule in saved_rules:
                    if (isinstance(rule, dict) and isinstance(rule.get("name"), str)
                            and rule.get("data_type") in TYPE_FORMATS and rule.get("byte_order") in ("little", "big")
                            and isinstance(rule.get("offset"), int) and rule["offset"] >= 0):
                        valid.append(rule)
                for row in list(self.rule_rows):
                    self.delete_rule_row(row)
                for rule in valid:
                    self.add_rule_row(rule["name"], rule["data_type"], rule["offset"], rule["byte_order"])
            selected = state.get("plot_selected", [])
            if isinstance(selected, list):
                for name, tag in self.plot_checks.items():
                    dpg.set_value(tag, name in selected)
            self.set_max_samples()
            self.set_imu_zoom()
            self.set_imu_quality()
            yaw, elev = state.get("view_yaw"), state.get("view_elev")
            if (isinstance(yaw, (float, int)) and isinstance(elev, (float, int))
                    and math.isfinite(yaw) and math.isfinite(elev)):
                self.set_imu_view(yaw, elev)
            self.switch_page(state.get("active_page", "params"))
            self.log(f"已恢复工作区：{len(self.slider_rows)} 个滑杆")
        if self._workspace_warning or errors:
            self.log(self._workspace_warning or "部分无效滑杆未恢复：" + "；".join(errors), "error")
        self._workspace_ready = True
        self._workspace_signature = json.dumps(self.capture_workspace(), ensure_ascii=False, sort_keys=True)

    def save_workspace(self, force=False) -> bool:
        if not self._workspace_ready:
            return False
        try:
            state = self.capture_workspace()
            signature = json.dumps(state, ensure_ascii=False, sort_keys=True, allow_nan=False)
            if force or signature != self._workspace_signature:
                self.workspace_store.save(state)
                self._workspace_signature = signature
                dpg.set_value("workspace_status", "工作区已保存 " + time.strftime("%H:%M:%S"))
                if force:
                    self.log(f"工作区已保存：{self.workspace_store.path}")
            return True
        except (ValueError, OSError) as exc:
            self.log(f"工作区保存失败：{exc}", "error")
            return False

    def autosave_workspace(self) -> None:
        now = time.monotonic()
        if now - self._last_workspace_poll >= 1:
            self._last_workspace_poll = now
            self.save_workspace()

    def close_workspace(self) -> None:
        self.save_workspace(force=True)
        if self._workspace_temp is not None:
            self._workspace_temp.cleanup()
