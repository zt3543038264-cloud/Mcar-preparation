"""可配置的界面命令按键，所有控件与网络发送均在 GUI 主线程处理。"""
import dearpygui.dearpygui as dpg
from button_protocol import ButtonConfig, ButtonState


class CommandButtons:
    def initialize_buttons(self):
        self.button_rows = []
        self.button_states = {}
        self._button_active = {}
        self._button_visual = {}

    def build_button_ui(self):
        with dpg.child_window(height=self.px(300), width=-1, border=True, tag="command_buttons_panel"):
            dpg.add_text("可配置命令按键", color=(98, 185, 255))
            dpg.add_text("按住：按下/松开发命令；自锁：第一次按下，第二次抬起。空命令表示该动作不发包。")
            with dpg.table(tag="command_buttons_table", header_row=True, resizable=True,
                           policy=dpg.mvTable_SizingStretchProp):
                for name, weight in (("名称", 1), ("按下命令", 2), ("抬起命令", 2), ("模式", .9),
                                     ("格式", .75), ("追加 LF", .65), ("操作按键", 1.5), ("管理", 1.3)):
                    dpg.add_table_column(label=name, init_width_or_weight=weight)
            with dpg.group(horizontal=True):
                dpg.add_button(label="＋ 新增按键", callback=lambda: self.add_command_button())
                dpg.add_button(label="全部抬起", callback=lambda: self.release_command_buttons())
            dpg.add_text("配置随工作区保存；重开默认抬起，不自动发命令。", color=(148, 158, 171))
            dpg.add_text("尚未发送按键命令", tag="command_button_status", color=(100, 175, 255))

    def add_command_button(self, config=None):
        if len(self.button_rows) >= 200:
            self.log("最多支持 200 个命令按键", "error")
            return None
        config = ButtonConfig.from_dict(config or {})
        with dpg.table_row(parent="command_buttons_table") as row:
            dpg.add_input_text(tag=f"cmd_name_{row}", default_value=config.name, width=-1)
            dpg.add_input_text(tag=f"cmd_press_{row}", default_value=config.press_command, width=-1)
            dpg.add_input_text(tag=f"cmd_release_{row}", default_value=config.release_command, width=-1)
            dpg.add_combo(["按住", "自锁"], tag=f"cmd_mode_{row}",
                          default_value="自锁" if config.mode == "latch" else "按住", width=-1)
            dpg.add_combo(["文本", "HEX"], tag=f"cmd_encoding_{row}",
                          default_value="HEX" if config.encoding == "hex" else "文本", width=-1)
            dpg.add_checkbox(tag=f"cmd_newline_{row}", default_value=config.newline)
            with dpg.group():
                # 不依赖默认按钮回调（它发生在松开时），逐帧检测真实按下边沿。
                dpg.add_button(label="按下", tag=f"cmd_trigger_{row}", width=-1)
                dpg.add_text("本机：抬起", tag=f"cmd_state_{row}", color=(148, 158, 171))
            with dpg.group(horizontal=True):
                dpg.add_button(label="抬起", callback=lambda: self.release_command_button(row))
                dpg.add_button(label="删除", callback=lambda: self.delete_command_button(row))
        self.button_rows.append(row)
        self.button_states[row] = ButtonState()
        self._button_active[row] = False
        self._update_button_state(row)
        return row

    def command_button_config(self, row):
        return ButtonConfig(name=dpg.get_value(f"cmd_name_{row}"),
                            press_command=dpg.get_value(f"cmd_press_{row}"),
                            release_command=dpg.get_value(f"cmd_release_{row}"),
                            mode="latch" if dpg.get_value(f"cmd_mode_{row}") == "自锁" else "momentary",
                            encoding="hex" if dpg.get_value(f"cmd_encoding_{row}") == "HEX" else "text",
                            newline=dpg.get_value(f"cmd_newline_{row}"))

    def _send_button_packet(self, packet, endpoint):
        if not self.receiver.is_running:
            raise OSError("请先开始监听")
        if self.receiver.send_bound(packet, *endpoint) != len(packet):
            raise OSError("UDP 报文未完整发送")
        preview = packet.decode("utf-8", errors="backslashreplace").rstrip() if b"\x00" not in packet else packet.hex(" ")
        dpg.set_value("command_button_status", f"按键已发送 -> {endpoint[0]}:{endpoint[1]}：{preview}")
        self.log(f"按键发送 {len(packet)} 字节 -> {endpoint[0]}:{endpoint[1]}：{packet!r}", "tx")

    def _update_button_state(self, row):
        pressed = self.button_states[row].pressed
        mode = "latch" if dpg.get_value(f"cmd_mode_{row}") == "自锁" else "momentary"
        if self._button_visual.get(row) == (pressed, mode):
            return
        self._button_visual[row] = (pressed, mode)
        label = ("再次点击抬起" if pressed else "点击自锁") if mode == "latch" else "按住"
        dpg.configure_item(f"cmd_trigger_{row}", label=label)
        dpg.set_value(f"cmd_state_{row}", "本机：已按下" if pressed else "本机：抬起")
        dpg.configure_item(f"cmd_state_{row}", color=(255, 180, 75) if pressed else (148, 158, 171))
        for suffix in ("name", "press", "release", "mode", "encoding", "newline"):
            dpg.configure_item(f"cmd_{suffix}_{row}", enabled=not pressed)

    def press_command_button(self, row):
        try:
            config = self.command_button_config(row)
            endpoint = (dpg.get_value("remote_ip").strip(), int(dpg.get_value("remote_port")))
            if not self.receiver.is_running:
                raise OSError("请先开始监听")
            self.button_states[row].press(config, endpoint, self._send_button_packet)
        except (OSError, ValueError, RuntimeError) as exc:
            self.log(f"按键操作失败：{exc}", "error")
            dpg.set_value("command_button_status", f"按键操作失败：{exc}")
        self._update_button_state(row)

    def release_command_button(self, row):
        try:
            self.button_states[row].release(self._send_button_packet)
        except (OSError, ValueError, RuntimeError) as exc:
            self.log(f"抬起发送失败，请重新监听后点“抬起”重试：{exc}", "error")
            dpg.set_value("command_button_status", f"抬起发送失败：{exc}")
        self._update_button_state(row)
        return not self.button_states[row].pressed

    def release_command_buttons(self, momentary_only=False):
        for row in list(self.button_rows):
            if not momentary_only or self.command_button_config(row).mode == "momentary":
                self.release_command_button(row)

    def delete_command_button(self, row):
        if not self.release_command_button(row):
            return  # 先完成抬起发送，保留失败按键以供重试。
        self.button_rows.remove(row)
        self.button_states.pop(row)
        self._button_active.pop(row)
        self._button_visual.pop(row, None)
        dpg.delete_item(row)

    def poll_command_buttons(self):
        """边沿检测避免按住时每帧发包；松开在按钮外、页面隐藏也会处理。"""
        for row in self.button_rows:
            input_down = (dpg.is_mouse_button_down(dpg.mvMouseButton_Left)
                          or dpg.is_key_down(dpg.mvKey_Spacebar) or dpg.is_key_down(dpg.mvKey_Return))
            active = self.active_page == "params" and input_down and dpg.is_item_active(f"cmd_trigger_{row}")
            previous = self._button_active[row]
            self._button_active[row] = active
            if active and not previous:
                self.press_command_button(row)
            elif previous and not active and self.command_button_config(row).mode == "momentary":
                self.release_command_button(row)
            self._update_button_state(row)

    def capture_buttons(self):
        return [self.command_button_config(row).to_dict() for row in self.button_rows]

    def restore_buttons(self, data):
        if not isinstance(data, list) or len(data) > 200:
            self.log("按键存档无效，已跳过", "error")
            return
        for config in data:
            try:
                self.add_command_button(config)
            except (TypeError, ValueError) as exc:
                self.log(f"按键配置未恢复：{exc}", "error")
