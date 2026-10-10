"""逐飞科技 WiFi-SPI2.0 UDP 上位机（DearPyGui）。

运行：python app.py
默认协议示例为小端 float32 的 roll/pitch/yaw，用户可在界面中任意修改。
"""

from __future__ import annotations

import csv
import ctypes
from collections import deque
from datetime import datetime
import os
from pathlib import Path
import queue
import struct
import sys
import time
from typing import Any

import dearpygui.dearpygui as dpg

from attitude_3d import draw_aircraft_attitude
from packet_parser import FieldRule, TYPE_FORMATS, parse_packet
from live_plot import LivePlot
from plot_history import PlotHistory
from slider_workspace import SliderWorkspace
from command_buttons import CommandButtons
from udp_receiver import UdpReceiver
from mcar_telemetry import PRESETS, Subscription, control_reply, parse_names, validate_frame


def get_ui_scale() -> float:
    """先启用逐显示器 DPI 感知，阻止 Windows 把整张窗口当位图放大。"""
    if sys.platform != "win32":
        return 1.0
    try:
        user32 = ctypes.windll.user32
        # -4 是 DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2；必须在创建窗口前调用。
        user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        return max(1.0, min(2.5, user32.GetDpiForSystem() / 96.0))
    except (AttributeError, OSError):
        return 1.0


class WifiSpiMonitor(LivePlot, SliderWorkspace, CommandButtons):
    """组合 UDP、解析、记录与 DearPyGui 控件的上位机主类。"""

    REFRESH_INTERVAL = 1 / 120  # 队列处理最多 120 Hz，避免高帧率报文拖慢界面
    MAX_PROCESS_PER_TICK = 300

    def __init__(self, ui_scale: float = 1.0) -> None:
        self.ui_scale = ui_scale
        self.receiver = UdpReceiver()
        self.rule_rows: list[int] = []
        self.initialize_workspace("MCAR", __file__)
        self.initialize_buttons()
        self.initialize_plot()
        self.latest_values: dict[str, Any] = {}
        self.value_tags: dict[str, int] = {}
        self.plot_checks: dict[str, int] = {}
        self.plot_series: dict[str, int] = {}
        self.history: dict[str, PlotHistory] = {}
        self.records: list[dict[str, Any]] = []
        self.packet_times: deque[float] = deque()
        self.active_page = "params"
        self.max_samples = 1200
        self.imu_view_yaw = -55.0
        self.imu_view_elev = 30.0
        self.imu_zoom = 1.0
        self.imu_quality = "high"
        self._plot_dirty = True
        self._attitude_dirty = True
        self._values_dirty = True
        self._last_refresh = 0.0
        self._last_plot_refresh = 0.0
        self._last_plot_fit = 0.0
        self._last_draw = 0.0
        self._last_draw_size = (0, 0)
        self._last_parse_errors: set[str] = set()
        self._log_tags: deque[int] = deque(maxlen=400)
        self._last_rules_signature: tuple[tuple[str, str, int, str], ...] = ()
        self._last_rx_log = 0.0
        self.subscription = Subscription()
        self.mcar_names: list[str] | None = None
        self.last_peer: tuple[str, int] | None = None

    def px(self, value: int) -> int:
        """把布局尺寸换算为物理像素，避免高 DPI 字体挤出固定宽度控件。"""
        return round(value * self.ui_scale)

    # ----------------------------- 日志与配置 -----------------------------
    def log(self, message: str, level: str = "info") -> None:
        """在 GUI 主线程追加一条有颜色的日志。"""
        colors = {"rx": (115, 210, 145, 255), "tx": (100, 175, 255, 255), "error": (245, 105, 105, 255),
                  "info": (220, 220, 220, 255)}
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag = dpg.add_text(f"[{stamp}] {message}", color=colors[level], parent="log_panel")
        if len(self._log_tags) == self._log_tags.maxlen:
            old_tag = self._log_tags.popleft()
            if dpg.does_item_exist(old_tag):
                dpg.delete_item(old_tag)
        self._log_tags.append(tag)
        if self.active_page == "params":
            dpg.set_y_scroll("log_panel", dpg.get_y_scroll_max("log_panel"))

    def clear_log(self) -> None:
        dpg.delete_item("log_panel", children_only=True)
        self._log_tags.clear()

    def switch_page(self, page: str) -> None:
        """左侧导航只显示一个页面；收包线程不受页面切换影响。"""
        if page not in ("params", "plot", "imu"):
            return
        if self.active_page == "params" and page != "params":
            self.release_command_buttons(momentary_only=True)
        self.active_page = page
        slider_host = "plot_slider_host" if page == "plot" else "params_slider_host"
        dpg.move_item("shared_slider_panel", parent=slider_host)
        dpg.configure_item("shared_slider_panel", height=self.px(230 if page == "plot" else 320))
        for name in ("params", "plot", "imu"):
            dpg.configure_item(f"page_{name}", show=name == page)
            dpg.bind_item_theme(f"nav_{name}", "theme_nav_active" if name == page else "theme_nav_idle")
        if page == "plot":
            self._plot_dirty = self._values_dirty = True
            self._last_plot_refresh = 0.0
        elif page == "imu":
            self._attitude_dirty = True
        else:
            dpg.set_y_scroll("log_panel", dpg.get_y_scroll_max("log_panel"))

    def set_max_samples(self) -> None:
        """更改屏幕绘制点数预算，不删除历史采样。"""
        self.max_samples = max(100, min(5000, int(dpg.get_value("max_samples"))))
        self._plot_render_signature = None  # 绘制点数上限不再删除历史。
        self._plot_dirty = True

    def set_imu_view(self, yaw: float, elevation: float) -> None:
        """改变观察视角，不修改真实 IMU 欧拉角。"""
        self.imu_view_yaw = yaw
        self.imu_view_elev = elevation
        self._attitude_dirty = True

    def set_imu_zoom(self) -> None:
        self.imu_zoom = float(dpg.get_value("imu_zoom"))
        self._attitude_dirty = True

    def set_imu_quality(self) -> None:
        self.imu_quality = "high" if dpg.get_value("imu_quality") == "精细" else "low"
        self._attitude_dirty = True

    def add_rule_row(self, name: str = "new_field", data_type: str = "float32", offset: int = 0,
                     byte_order: str = "little") -> None:
        """新增一行可编辑解析规则。tag 保存为删除时使用的 table row。"""
        with dpg.table_row(parent="rules_table") as row:
            dpg.add_input_text(default_value=name, width=-1, tag=f"rule_name_{row}")
            dpg.add_combo(list(TYPE_FORMATS), default_value=data_type, width=-1, tag=f"rule_type_{row}")
            dpg.add_input_int(default_value=offset, min_value=0, min_clamped=True, width=-1, tag=f"rule_offset_{row}")
            dpg.add_combo(["little", "big"], default_value=byte_order, width=-1, tag=f"rule_order_{row}")
            dpg.add_button(label="删除", width=-1, callback=lambda: self.delete_rule_row(row))
        self.rule_rows.append(row)
        self.rebuild_value_view()

    def delete_rule_row(self, row: int) -> None:
        if row in self.rule_rows:
            self.rule_rows.remove(row)
        if dpg.does_item_exist(row):
            dpg.delete_item(row)
        self.rebuild_value_view()

    def collect_rules(self) -> list[FieldRule]:
        """从当前 GUI 控件构建规则，重复/空名称由这里统一处理。"""
        rules: list[FieldRule] = []
        used_names: set[str] = set()
        for row in self.rule_rows:
            if not dpg.does_item_exist(row):
                continue
            name = dpg.get_value(f"rule_name_{row}").strip()
            if not name:
                continue
            if name in used_names:
                # 重名字段会覆盖，主动跳过并记录一次提醒更安全。
                continue
            used_names.add(name)
            rules.append(FieldRule(name, dpg.get_value(f"rule_type_{row}"), int(dpg.get_value(f"rule_offset_{row}")),
                                   dpg.get_value(f"rule_order_{row}")))
        return rules

    def _rules_signature(self, rules: list[FieldRule]) -> tuple[tuple[str, str, int, str], ...]:
        return tuple((r.name, r.data_type, r.offset, r.byte_order) for r in rules)

    def rebuild_value_view(self) -> None:
        """规则变化后重建实时表格和曲线字段勾选项。"""
        rules = self.collect_rules() if dpg.does_item_exist("rules_table") else []
        names = [rule.name for rule in rules]
        selected = {name for name, tag in self.plot_checks.items() if dpg.does_item_exist(tag) and dpg.get_value(tag)}

        # 表格列也是 DearPyGui 的子项，重建时从容器整体替换，避免误删列后留下无效表格。
        dpg.delete_item("values_panel", children_only=True)
        self.value_tags.clear()
        with dpg.table(header_row=True, resizable=True, policy=dpg.mvTable_SizingStretchProp, tag="values_table",
                       parent="values_panel"):
            dpg.add_table_column(label="字段")
            dpg.add_table_column(label="实时值")
            for name in names:
                with dpg.table_row():
                    dpg.add_text(name)
                    self.value_tags[name] = dpg.add_text("--")

        dpg.delete_item("plot_select_panel", children_only=True)
        self.plot_checks.clear()
        for name in names:
            # 这里处于刷新函数而非 DearPyGui 的 with 上下文，必须显式指定父容器。
            self.plot_checks[name] = dpg.add_checkbox(label=name, default_value=name in selected,
                                                      parent="plot_select_panel",
                                                      callback=lambda: setattr(self, "_plot_dirty", True))
        self._values_dirty = self._plot_dirty = self._attitude_dirty = True

    # ----------------------------- 网络收发 -----------------------------
    def send_mcar(self, command: str) -> None:
        """沿用现有 UDP 发送通道；命令必须有换行，响应回固件配置的电脑端口。"""
        try:
            remote_ip = dpg.get_value("remote_ip").strip()
            remote_port = int(dpg.get_value("remote_port"))
            self.receiver.send_bound((command.rstrip("\r\n") + "\n").encode("ascii"), remote_ip, remote_port)
            self.log(f"MCAR -> {remote_ip}:{remote_port}：{command.strip()}", "tx")
        except (ValueError, OSError, UnicodeError) as exc:
            self.subscription.abort(f"发送失败：{exc}")
            self.log(self.subscription.error, "error")

    def use_last_peer(self) -> None:
        if self.last_peer is None:
            self.log("尚未收到数据，请先开始监听", "error")
            return
        dpg.set_value("remote_ip", self.last_peer[0])
        dpg.set_value("remote_port", self.last_peer[1])
        self.log(f"模块地址已设为 {self.last_peer[0]}:{self.last_peer[1]}")

    def set_preset(self, name: str) -> None:
        dpg.set_value("mcar_names", ",".join(PRESETS[name]))

    def apply_subscription(self) -> None:
        try:
            if not self.receiver.is_running:
                raise ValueError("请先开始监听，以接收设备确认")
            command = self.subscription.begin(dpg.get_value("mcar_names"),
                                              int(dpg.get_value("mcar_period")), time.monotonic())
            dpg.set_value("mcar_mode", True)
            self.send_mcar(command)
        except ValueError as exc:
            self.log(str(exc), "error")

    def install_mcar_rules(self, names: list[str]) -> None:
        """仅以设备确认的通道顺序更新字段，不能提前用请求值解码。"""
        dpg.set_value("mcar_names", ",".join(names))
        if names == self.mcar_names:
            return
        for row in self.rule_rows:
            dpg.delete_item(row)
        self.rule_rows.clear()
        for i, name in enumerate(names):
            self.add_rule_row(name, "float32", i * 4, "little")
        self.mcar_names = names
        self.latest_values.clear()
        self.history.clear()
        self._last_rules_signature = self._rules_signature(self.collect_rules())
        self.log("解析字段已按设备确认自动更新：" + ",".join(names))

    def handle_mcar_reply(self, kind: str, value: str) -> None:
        """ASCII 应答只进入日志/配置，不进入数据、曲线或 CSV。"""
        if kind == "SLIDER":
            self.handle_slider_reply(value)
            self.log(f"设备已确认调参：{value}", "rx")
            return  # 调参确认独立于遥测订阅状态机，不改变解析字段。
        if self.subscription.pending and kind != "ERR" and (
                f"{kind} {value}" != self.subscription.commands[self.subscription.index]):
            return  # 延迟的 GET?/重试应答不能重写切换中的映射。
        self.log(f"设备：{kind} {value}", "error" if kind == "ERR" else "rx")
        if kind == "SUB":
            self.install_mcar_rules(parse_names(value))
        elif kind == "LIST":
            dpg.configure_item("mcar_available", items=value.split(","))
            dpg.set_value("mcar_available", value.split(",")[0])
        elif kind == "RATE":
            dpg.set_value("mcar_period", int(value))
        command = self.subscription.acknowledge(kind, value, time.monotonic())
        if command is not None:
            self.send_mcar(command)
        if not self.subscription.pending and not self.subscription.error:
            dpg.set_value("mcar_state", "设备配置已确认")

    def add_available_variable(self) -> None:
        name = dpg.get_value("mcar_available")
        names = [part.strip() for part in dpg.get_value("mcar_names").split(",") if part.strip()]
        if name and name not in names and len(names) < 40:
            dpg.set_value("mcar_names", ",".join(names + [name]))

    def start_listening(self) -> None:
        try:
            bind_ip = dpg.get_value("bind_ip").strip()
            bind_port = int(dpg.get_value("bind_port"))
            self.receiver.start(bind_ip, bind_port)
            self.log(f"开始监听 {bind_ip}:{bind_port}", "info")
        except (ValueError, OSError, RuntimeError) as exc:
            self.log(f"无法开始监听：{exc}", "error")

    def stop_listening(self) -> None:
        self.release_command_buttons()  # 关闭 socket 前尝试发送抬起命令。
        if self.subscription.pending:
            self.subscription.abort("监听已停止，配置未完成；请重新监听并应用")
        self.receiver.stop()
        dpg.set_value("plot_follow", False)  # 停止后保留当前视口，可自由缩放查看。
        self.log("已停止 UDP 监听", "info")

    def send_command(self) -> None:
        """发送文本 UTF-8 或空格分隔十六进制字节，如 'AA 55 01 0D 0A'。"""
        try:
            raw = dpg.get_value("send_payload")
            if not raw.strip():
                raise ValueError("发送内容不能为空")
            is_hex = dpg.get_value("send_as_hex")
            data = bytes.fromhex(raw) if is_hex else (raw.rstrip("\r\n") + "\n").encode("utf-8")
            if not data:
                raise ValueError("发送内容不能为空")
            remote_ip = dpg.get_value("remote_ip").strip()
            remote_port = int(dpg.get_value("remote_port"))
            sender = self.receiver.send_bound if self.receiver.is_running else UdpReceiver.send
            sent = sender(data, remote_ip, remote_port)
            preview = data.hex(" ").upper() if is_hex else raw
            self.log(f"发送 {sent} 字节 -> {remote_ip}:{remote_port}：{preview}", "tx")
        except (ValueError, OSError) as exc:
            self.log(f"发送失败：{exc}", "error")

    # ----------------------------- 接收、解析、图表 -----------------------------
    def _handle_packet(self, timestamp: float, packet: bytes, address: tuple[str, int], rules: list[FieldRule]) -> None:
        self.last_peer = address
        # 丢弃其他来源的控制应答，避免测试器或另一辆车修改当前解析规则。
        try:
            reply = control_reply(packet)
            if reply is not None:
                if address != (dpg.get_value("remote_ip").strip(), int(dpg.get_value("remote_port"))):
                    return
                self.handle_mcar_reply(*reply)
                return
            if self.subscription.pending:
                return  # 暂停/切换过程中不使用旧通道映射解析数据。
            if dpg.get_value("mcar_mode"):
                rules = self.collect_rules()  # 同一批队列中的 SUB 应答可能刚更新规则。
                validate_frame(packet, len(rules))
        except (ValueError, UnicodeError) as exc:
            message = str(exc)
            if {message} != self._last_parse_errors:
                self.log(message, "error")
                self._last_parse_errors = {message}
            return
        values, errors = parse_packet(packet, rules)
        self.latest_values = values
        self.packet_times.append(time.monotonic())
        # 高频传感器可能数百 Hz，接收日志限为每 0.5 秒一条，避免日志本身拖慢 GUI。
        if time.monotonic() - self._last_rx_log >= 0.5:
            self.log(f"收到 UDP 报文：{address[0]}:{address[1]}，{len(packet)} 字节", "rx")
            self._last_rx_log = time.monotonic()
        # 按到达时刻保存，不受绘图采样上限影响，CSV 导出包含全部已解析报文。
        record: dict[str, Any] = {"timestamp": datetime.fromtimestamp(timestamp).isoformat(timespec="milliseconds"),
                                  "source_ip": address[0], "source_port": address[1]}
        record.update(values)
        self.records.append(record)

        if self.plot_time_origin is None:
            self.plot_time_origin = timestamp
        for name, value in values.items():
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            history = self.history.get(name)
            # 历史容量独立于屏幕点数；旧区域通过二分定位按需取样。
            if history is None:
                history = PlotHistory(self.plot_history_capacity)
                self.history[name] = history
            history.append((timestamp - self.plot_time_origin, number))
        self._values_dirty = self._plot_dirty = self._attitude_dirty = True

        if errors:
            new_errors = {f"{name}: {reason}" for name, reason in errors.items()}
            # 同一规则造成的长度不足不在每帧刷屏；规则变更或错误内容变化才提示。
            if new_errors != self._last_parse_errors:
                self.log("解析提示：" + "； ".join(sorted(new_errors)), "error")
                self._last_parse_errors = new_errors
        else:
            self._last_parse_errors.clear()

    def _update_values(self) -> None:
        for name, tag in self.value_tags.items():
            value = self.latest_values.get(name)
            if isinstance(value, float):
                text = f"{value:.6g}"
            elif value is None:
                text = "--"
            else:
                text = str(value)
            dpg.set_value(tag, text)
        self._values_dirty = False

    def _update_attitude(self) -> None:
        """字段名不区分大小写，需命名为 roll、pitch、yaw 才会驱动姿态。"""
        lookup = {str(key).lower(): value for key, value in self.latest_values.items()}
        try:
            roll = float(lookup.get("roll_deg", lookup.get("roll", 0.0)))
            pitch = float(lookup.get("pitch_deg", lookup.get("pitch", 0.0)))
            yaw = float(lookup.get("yaw_deg", lookup.get("nav_yaw_deg", lookup.get("yaw", 0.0))))
        except (TypeError, ValueError):
            roll = pitch = yaw = 0.0
        size = tuple(dpg.get_item_rect_size("attitude_drawlist"))
        if self._attitude_dirty or size != self._last_draw_size:
            draw_aircraft_attitude("attitude_drawlist", roll, pitch, yaw, self.imu_view_yaw,
                                   self.imu_view_elev, self.imu_zoom, self.ui_scale, self.imu_quality)
            for tag, angle in (("imu_roll_value", roll), ("imu_pitch_value", pitch), ("imu_yaw_value", yaw)):
                dpg.set_value(tag, f"{angle:+.2f}°")
            dpg.set_value("imu_data_state", "姿态数据已更新" if self.latest_values else "等待 roll / pitch / yaw 数据")
            self._last_draw = time.monotonic()
            self._last_draw_size = size
            self._attitude_dirty = False

    def refresh(self) -> None:
        """在 DearPyGui 主循环调用：批量取事件，所有 GUI 更新均发生于此线程。"""
        now = time.monotonic()
        if now - self._last_refresh < self.REFRESH_INTERVAL:
            return
        self._last_refresh = now
        retry = self.subscription.poll(now)
        if retry is not None:
            self.send_mcar(retry)
        if self.subscription.pending:
            dpg.set_value("mcar_state", f"等待设备确认：{self.subscription.commands[self.subscription.index]}")
        elif self.subscription.error:
            dpg.set_value("mcar_state", self.subscription.error)

        rules = self.collect_rules()
        signature = self._rules_signature(rules)
        if signature != self._last_rules_signature:
            self._last_rules_signature = signature
            self.rebuild_value_view()

        processed = 0
        while processed < self.MAX_PROCESS_PER_TICK:
            try:
                kind, payload = self.receiver.events.get_nowait()
            except queue.Empty:
                break
            if kind == "packet":
                timestamp, packet, address = payload  # type: ignore[misc]
                self._handle_packet(timestamp, packet, address, rules)
            elif kind == "error":
                self.log(str(payload), "error")
            elif kind == "status":
                self.log(str(payload), "info")
            processed += 1

        while self.packet_times and now - self.packet_times[0] > 1.0:
            self.packet_times.popleft()
        dpg.set_value("network_status", self.receiver.status)
        dpg.set_value("fps_status", f"接收帧率：{len(self.packet_times)} fps   总帧数：{self.receiver.received_count}   排队丢弃：{self.receiver.dropped_count}")
        # 隐藏页仍采集、解析和记录，但不更新它们的图形控件。
        if self.active_page == "plot":
            if self._values_dirty:
                self._update_values()
            if self._plot_dirty and now - self._last_plot_refresh >= 1 / int(dpg.get_value("plot_refresh_hz")):
                self._update_plot()
        elif self.active_page == "imu":
            self._update_attitude()

    # ----------------------------- CSV -----------------------------
    def save_csv(self) -> None:
        try:
            path = dpg.get_value("csv_path").strip()
            if not path:
                path = f"imu_data_{datetime.now():%Y%m%d_%H%M%S}.csv"
            path = os.path.abspath(path)
            if not self.records:
                raise ValueError("当前没有可保存的解析数据")
            # 用所有历史记录中出现过的字段做列，配置中途改变也不会丢列。
            columns = ["timestamp", "source_ip", "source_port"]
            for record in self.records:
                for key in record:
                    if key not in columns:
                        columns.append(key)
            with open(path, "w", newline="", encoding="utf-8-sig") as file:
                writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
                writer.writeheader()
                writer.writerows(self.records)
            self.log(f"CSV 已保存：{path}（{len(self.records)} 条）", "info")
        except (OSError, ValueError) as exc:
            self.log(f"保存 CSV 失败：{exc}", "error")

    def build_ui(self) -> None:
        px = self.px
        # 深色导航主题：当前页面保持高亮，帮助快速判断正在操作哪个工作区。
        with dpg.theme(tag="theme_nav_active"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (35, 108, 194))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (48, 128, 219))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (30, 89, 163))
        with dpg.theme(tag="theme_nav_idle"):
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (39, 46, 57))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (57, 68, 84))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (44, 93, 148))

        with dpg.window(label="逐飞 WiFi-SPI2.0 UDP IMU 上位机", tag="main_window", no_scrollbar=True):
            with dpg.group(horizontal=True):
                with dpg.child_window(width=px(225), height=-1, border=True, tag="sidebar"):
                    dpg.add_spacer(height=px(8))
                    dpg.add_text("WiFi-SPI  /  IMU", color=(98, 185, 255))
                    dpg.add_text("UDP 实时上位机", color=(165, 179, 196))
                    dpg.add_spacer(height=px(18))
                    dpg.add_button(label="01   调参", tag="nav_params", width=-1, height=px(48),
                                   callback=lambda: self.switch_page("params"))
                    dpg.add_spacer(height=px(5))
                    dpg.add_button(label="02   实时绘图", tag="nav_plot", width=-1, height=px(48),
                                   callback=lambda: self.switch_page("plot"))
                    dpg.add_spacer(height=px(5))
                    dpg.add_button(label="03   IMU 姿态", tag="nav_imu", width=-1, height=px(48),
                                   callback=lambda: self.switch_page("imu"))
                    dpg.add_spacer(height=px(20))
                    dpg.add_separator()
                    dpg.add_text("连接状态", color=(98, 185, 255))
                    dpg.add_text("未监听", tag="network_status", wrap=px(200))
                    dpg.add_text("接收帧率：0 fps", tag="fps_status", wrap=px(200))
                    dpg.add_spacer(height=px(12))
                    dpg.add_text("切页不影响后台收包与记录。", color=(148, 158, 171), wrap=px(200))

                # 三个页面始终存在，切换只改变可见性；动态控件均有明确父容器。
                with dpg.child_window(width=-1, height=-1, border=False, tag="content_root", no_scrollbar=True):
                    with dpg.child_window(width=-1, height=-1, border=False, tag="page_params"):
                        dpg.add_text("调参 / 数据连接", color=(98, 185, 255))
                        dpg.add_text("配置监听、发送指令与解析规则；运行日志也集中在本页。")
                        dpg.add_separator()
                        with dpg.group(horizontal=True):
                            with dpg.child_window(width=px(530), height=px(220), border=True):
                                dpg.add_text("UDP 监听", color=(98, 185, 255))
                                dpg.add_input_text(label="本机监听 IP", default_value="0.0.0.0", tag="bind_ip", width=px(240))
                                dpg.add_input_int(label="本机监听端口", default_value=8081, min_value=0,
                                                  max_value=65535, min_clamped=True, max_clamped=True,
                                                  tag="bind_port", width=px(240))
                                with dpg.group(horizontal=True):
                                    dpg.add_button(label="开始监听", callback=lambda: self.start_listening(), width=px(120))
                                    dpg.add_button(label="停止监听", callback=lambda: self.stop_listening(), width=px(120))
                                dpg.add_text("0.0.0.0 表示监听电脑的所有网卡。", color=(148, 158, 171))
                            with dpg.child_window(width=-1, height=px(220), border=True):
                                dpg.add_text("UDP 指令发送", color=(98, 185, 255))
                                dpg.add_input_text(label="模块 IP", default_value="192.168.1.100",
                                                   tag="remote_ip", width=px(240))
                                dpg.add_input_int(label="模块端口", default_value=5001, min_value=1,
                                                  max_value=65535, min_clamped=True, max_clamped=True,
                                                  tag="remote_port", width=px(240))
                                dpg.add_input_text(label="命令内容", hint="文本或 AA 55 01", tag="send_payload", width=px(340))
                                with dpg.group(horizontal=True):
                                    dpg.add_checkbox(label="十六进制", tag="send_as_hex")
                                    dpg.add_button(label="发送 UDP 指令", callback=lambda: self.send_command(), width=px(150))

                        dpg.add_spacer(height=px(6))
                        with dpg.group(tag="params_slider_host"):
                            self.build_slider_ui()
                        dpg.add_spacer(height=px(6))
                        self.build_button_ui()
                        dpg.add_spacer(height=px(6))
                        with dpg.child_window(height=px(280), width=-1, border=True):
                            dpg.add_text("麦轮遥测：选择设备实际上传的变量", color=(98, 185, 255))
                            with dpg.group(horizontal=True):
                                dpg.add_checkbox(label="MCAR JustFloat 校验", default_value=True, tag="mcar_mode")
                                dpg.add_button(label="模块地址取最近来源", callback=lambda: self.use_last_peer())
                                dpg.add_button(label="查询变量", callback=lambda: self.send_mcar("LIST?"))
                                dpg.add_button(label="读取当前配置", callback=lambda: self.send_mcar("GET?"))
                            with dpg.group(horizontal=True):
                                for preset in PRESETS:
                                    dpg.add_button(label=preset + "预设", callback=lambda s, a, name: self.set_preset(name),
                                                   user_data=preset)
                            dpg.add_input_text(label="上传顺序", default_value=",".join(PRESETS["IMU"]),
                                               tag="mcar_names", width=px(620))
                            with dpg.group(horizontal=True):
                                dpg.add_combo(PRESETS["IMU"], default_value="roll_deg", tag="mcar_available", width=px(240))
                                dpg.add_button(label="添加变量", callback=lambda: self.add_available_variable())
                                dpg.add_input_int(label="周期 ms", default_value=10, min_value=2, max_value=1000,
                                                  min_clamped=True, max_clamped=True, tag="mcar_period", width=px(110))
                                dpg.add_button(label="应用上传配置", callback=lambda: self.apply_subscription())
                            dpg.add_text("先暂停、确认通道和周期，再恢复；字段自动按设备应答生成 float32/4字节偏移。")
                            dpg.add_text("等待配置；先监听，再用最近来源设置模块地址。", tag="mcar_state", wrap=px(800))

                        dpg.add_spacer(height=px(6))
                        with dpg.child_window(height=px(245), width=-1, border=True):
                            dpg.add_text("二进制报文解析规则", color=(98, 185, 255))
                            dpg.add_text("设置字段名称、数据类型、起始字节和字节序；默认 3 个小端 float32 欧拉角。")
                            with dpg.table(header_row=True, resizable=True,
                                           policy=dpg.mvTable_SizingStretchProp, tag="rules_table"):
                                for label in ("字段名", "数据类型", "起始字节", "字节序", "操作"):
                                    dpg.add_table_column(label=label)
                            dpg.add_button(label="＋ 新增数据项", callback=lambda: self.add_rule_row(), width=px(145))

                        dpg.add_spacer(height=px(6))
                        with dpg.child_window(height=px(125), width=-1, border=True):
                            dpg.add_text("数据保存", color=(98, 185, 255))
                            with dpg.group(horizontal=True):
                                dpg.add_input_text(label="CSV 文件路径", hint="留空自动命名", tag="csv_path", width=px(500))
                                dpg.add_button(label="保存全部解析数据", callback=lambda: self.save_csv(), width=px(150))
                            dpg.add_text("导出包含时间戳、来源地址及所有已解析字段。", color=(148, 158, 171))

                        dpg.add_spacer(height=px(6))
                        with dpg.child_window(height=px(245), width=-1, border=True):
                            with dpg.group(horizontal=True):
                                dpg.add_text("运行日志", color=(98, 185, 255))
                                dpg.add_button(label="清空日志", callback=lambda: self.clear_log(), width=px(100))
                                dpg.add_text("绿：接收    蓝：发送    红：错误")
                            dpg.add_child_window(tag="log_panel", height=px(195), border=False,
                                                 horizontal_scrollbar=True)

                    with dpg.child_window(width=-1, height=-1, border=False, tag="page_plot", show=False):
                        dpg.add_text("实时绘图", color=(98, 185, 255))
                        dpg.add_text("选中字段查看曲线，下方可直接调参；默认刷新 60 Hz，支持手动回看历史。")
                        dpg.add_separator()
                        with dpg.group(horizontal=True):
                            with dpg.child_window(width=px(270), height=-1, border=True):
                                dpg.add_text("实时解析值", color=(98, 185, 255))
                                dpg.add_group(tag="values_panel")
                                dpg.add_spacer(height=px(10))
                                dpg.add_separator()
                                dpg.add_text("曲线字段", color=(98, 185, 255))
                                dpg.add_child_window(tag="plot_select_panel", height=px(205), border=False)
                                dpg.add_spacer(height=px(8))
                                dpg.add_input_int(label="绘制点数上限", default_value=self.max_samples,
                                                  min_value=100, max_value=5000, min_clamped=True,
                                                  max_clamped=True, tag="max_samples", width=px(115),
                                                  callback=lambda: self.set_max_samples())
                                dpg.add_text("每字段缓存最近 100000 点；绘制超限时保留峰谷，CSV 仍保留全部接收记录。", color=(148, 158, 171), wrap=px(245))
                            with dpg.child_window(width=-1, height=-1, border=True):
                                self.build_plot_controls()
                                with dpg.child_window(width=-1, height=-self.px(240), border=False):
                                    with dpg.plot(label="", height=-1, width=-1, tag="data_plot", anti_aliased=True):
                                        dpg.add_plot_legend()
                                        dpg.add_plot_axis(dpg.mvXAxis, label="会话时间 / s", tag="plot_x_axis")
                                        dpg.add_plot_axis(dpg.mvYAxis, label="数值", tag="plot_y_axis")
                                dpg.add_group(tag="plot_slider_host")

                    with dpg.child_window(width=-1, height=-1, border=False, tag="page_imu", show=False):
                        dpg.add_text("IMU 三维姿态", color=(98, 185, 255))
                        dpg.add_text("根据 roll / pitch / yaw 欧拉角实时旋转机体模型；角度单位为度。")
                        dpg.add_separator()
                        with dpg.group(horizontal=True):
                            for title, tag in (("ROLL / 横滚", "imu_roll_value"),
                                               ("PITCH / 俯仰", "imu_pitch_value"),
                                               ("YAW / 航向", "imu_yaw_value")):
                                with dpg.child_window(width=px(245), height=px(82), border=True):
                                    dpg.add_text(title, color=(148, 158, 171))
                                    dpg.add_text("+0.00°", tag=tag, color=(98, 185, 255))
                        dpg.add_spacer(height=px(6))
                        dpg.add_text("等待 roll / pitch / yaw 数据", tag="imu_data_state")
                        with dpg.group(horizontal=True):
                            dpg.add_text("模型视角", color=(148, 158, 171))
                            dpg.add_button(label="等轴", width=px(75),
                                           callback=lambda: self.set_imu_view(-55, 30))
                            dpg.add_button(label="正视", width=px(75),
                                           callback=lambda: self.set_imu_view(0, 8))
                            dpg.add_button(label="俯视", width=px(75),
                                           callback=lambda: self.set_imu_view(-90, 85))
                            dpg.add_slider_float(label="缩放", tag="imu_zoom", default_value=1.0,
                                                 min_value=0.6, max_value=1.5, width=px(210),
                                                 format="%.2f", callback=lambda: self.set_imu_zoom())
                            dpg.add_combo(["精细", "流畅"], default_value="精细", tag="imu_quality",
                                          width=px(100), callback=lambda: self.set_imu_quality())
                        with dpg.child_window(width=-1, height=-1, border=True):
                            dpg.add_drawlist(width=px(1100), height=px(640), tag="attitude_drawlist")

        # 通过默认规则让启动后的界面立即可用于常见的 3×float32 IMU 报文。
        self.add_rule_row("roll", "float32", 0, "little")
        self.add_rule_row("pitch", "float32", 4, "little")
        self.add_rule_row("yaw", "float32", 8, "little")
        self._last_rules_signature = self._rules_signature(self.collect_rules())
        draw_aircraft_attitude("attitude_drawlist", 0, 0, 0, ui_scale=self.ui_scale)
        self.switch_page("params")
        self.restore_workspace([("pos_xy_kp", 0.01, 20, 0.001, 2.5),
                                ("pos_xy_kd", 0, 5, 0.001, 0),
                                ("pos_yaw_kp", 0.01, 20, 0.001, 2)])

    def shutdown(self) -> None:
        try:
            self.release_command_buttons()
            self.close_workspace()  # GUI 上下文销毁前保存最后一次编辑，包括未联网时的参数。
        finally:
            self.receiver.stop()


def main() -> None:
    ui_scale = get_ui_scale()
    dpg.create_context()
    dpg.configure_app(manual_callback_management=True)  # UI 编辑、按键与存档统一在主线程执行。
    # DearPyGui 默认字体仅含拉丁字符，中文会变成问号；加载系统中文字体并覆盖完整汉字字形范围。
    windows_fonts = Path(os.environ.get("WINDIR", r"C:\Windows")) / "Fonts"
    chinese_font = next((windows_fonts / name for name in ("Deng.ttf", "simhei.ttf", "NotoSansSC-VF.ttf")
                         if (windows_fonts / name).is_file()), None)
    if chinese_font is not None:
        with dpg.font_registry():
            with dpg.font(str(chinese_font), round(17 * ui_scale)) as font_id:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
        dpg.bind_font(font_id)
    monitor = WifiSpiMonitor(ui_scale)
    monitor.build_ui()
    # 当前 DearPyGui Windows 原生后端在部分系统上处理中文 viewport 标题会于首帧崩溃。
    # 标题使用 ASCII，界面内部仍保留中文标签。
    dpg.create_viewport(title="WiFi-SPI2.0 UDP IMU Monitor", width=round(1400 * ui_scale),
                        height=round(900 * ui_scale), vsync=False)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)
    # 打包验收使用：真正渲染 180 帧后自行退出，避免仅以“进程仍在”误判启动成功。
    self_test = "--self-test" in sys.argv
    test_commands: list[str] = []
    if self_test:
        # 自测只模拟设备消息，绝不向实车发送配置命令。
        monitor.send_mcar = test_commands.append
        dpg.set_value("remote_ip", "127.0.0.1")
        dpg.set_value("remote_port", 12345)
    rendered_frames = 0
    try:
        while dpg.is_dearpygui_running():
            frame_started = time.perf_counter()
            dpg.run_callbacks(dpg.get_callback_queue())
            monitor.poll_command_buttons()
            monitor.poll_plot_view()
            monitor.autosave_workspace()
            if self_test and rendered_frames == 30:
                # 打包验收同时覆盖三页切换、解析、曲线创建和姿态绘制。
                monitor.switch_page("plot")
                dpg.set_value(monitor.plot_checks["roll"], True)
                monitor.receiver.events.put(("packet", (time.time(), struct.pack("<fff", 12.5, -4.0, 32.0) + b"\0\0\x80\x7f",
                                                         ("127.0.0.1", 12345))))
            elif self_test and rendered_frames == 90:
                monitor.switch_page("imu")
            elif self_test and rendered_frames == 110:
                monitor.set_imu_view(-90, 85)
                dpg.set_value("imu_zoom", 1.2)
                monitor.set_imu_zoom()
            elif self_test and rendered_frames == 125:
                monitor.set_imu_view(-55, 30)
                dpg.set_value("imu_quality", "流畅")
                monitor.set_imu_quality()
            elif self_test and rendered_frames == 140:
                monitor.switch_page("params")
            elif self_test and rendered_frames == 145:
                monitor.send_mcar(monitor.subscription.begin("x_cm,y_cm,nav_yaw_deg", 20, time.monotonic()))
                for packet in (b"MCAR STREAM 0\n", b"MCAR SLIDER pos_xy_kp 2.5\n",
                               b"MCAR SUB x_cm,y_cm,nav_yaw_deg\n",
                               b"MCAR RATE 20\n", b"MCAR STREAM 1\n",
                               struct.pack("<fff", 25, -50, 40) + b"\0\0\x80\x7f"):
                    monitor.receiver.events.put(("packet", (time.time(), packet, ("127.0.0.1", 12345))))
                monitor._last_refresh = 0
                monitor.refresh()
                assert not monitor.subscription.pending and not monitor.subscription.error
                assert dpg.get_value("slider_ack_status") == "设备已确认：pos_xy_kp 2.5"
                assert monitor.latest_values == {"x_cm": 25, "y_cm": -50, "nav_yaw_deg": 40}
                assert [rule.offset for rule in monitor.collect_rules()] == [0, 4, 8]
                assert test_commands == ["STREAM 0\n", "SUB x_cm,y_cm,nav_yaw_deg\n", "RATE 20\n", "STREAM 1\n"]
            # 高频模拟帧用于验证导入的 OBJ 在动态姿态下仍能正常渲染。
            if self_test and 91 <= rendered_frames < 140:
                angle = rendered_frames - 91
                monitor.receiver.events.put(("packet", (time.time(), struct.pack("<fff", angle * 0.7,
                                                                                 angle * -0.3, angle * 0.5) + b"\0\0\x80\x7f",
                                                         ("127.0.0.1", 12345))))
            monitor.refresh()
            if self_test and rendered_frames == 160:
                from slider_selftest import verify_sliders
                verify_sliders(monitor)
            if self_test and rendered_frames == 165:
                from button_selftest import verify_buttons
                verify_buttons(monitor)
            if self_test and rendered_frames == 170:
                from plot_selftest import verify_plot
                verify_plot(monitor)
            dpg.render_dearpygui_frame()
            # 帧率由软件限频，120 Hz 选项不再被固定的 60 Hz 垂直同步限制。
            frame_hz = max(60, int(dpg.get_value("plot_refresh_hz")))
            delay = 1 / frame_hz - (time.perf_counter() - frame_started)
            if delay > 0:
                time.sleep(delay)
            rendered_frames += 1
            if self_test and rendered_frames >= 180:
                break
    finally:
        monitor.shutdown()
        dpg.destroy_context()


if __name__ == "__main__":
    main()
