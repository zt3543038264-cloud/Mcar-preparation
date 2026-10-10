"""实时跟随、暂停查看和保留手动视角的曲线绘制。"""
import time
from bisect import bisect_right
import dearpygui.dearpygui as dpg


class LivePlot:
    def initialize_plot(self):
        self.plot_time_origin = None
        self.plot_history_capacity = 100000
        self._plot_render_signature = None
        self._plot_fit_once = False
        self._plot_pause_time = None

    def build_plot_controls(self):
        with dpg.group(horizontal=True):
            dpg.add_checkbox(label="实时自适应", tag="plot_follow", default_value=True,
                             callback=lambda: self.plot_controls_changed())
            dpg.add_checkbox(label="暂停画面", tag="plot_paused", default_value=False,
                             callback=lambda: self.plot_controls_changed())
            dpg.add_input_double(label="跟随窗口 / s", tag="plot_window", default_value=10,
                                 min_value=1, max_value=300, min_clamped=True, max_clamped=True,
                                 width=self.px(65), format="%.1f", callback=lambda: self.plot_controls_changed())
            dpg.add_combo(["30", "60", "120"], label="刷新 Hz", tag="plot_refresh_hz", default_value="60",
                          width=self.px(65), callback=lambda: self.plot_controls_changed())
            dpg.add_button(label="查看全部历史", callback=lambda: self.show_all_plot_history())
            dpg.add_button(label="恢复实时", callback=lambda: self.resume_live_plot())
        dpg.add_text("滚轮缩放、左键拖动：自动切为手动查看；暂停画面仍继续收包。", color=(148, 158, 171))
        with dpg.handler_registry():
            dpg.add_mouse_wheel_handler(callback=self.plot_manual_interaction)
            for button in (dpg.mvMouseButton_Left, dpg.mvMouseButton_Middle, dpg.mvMouseButton_Right):
                dpg.add_mouse_down_handler(button=button, callback=self.plot_manual_interaction)

    def plot_controls_changed(self):
        if dpg.get_value("plot_paused"):
            if self._plot_pause_time is None:
                self._plot_pause_time = max((h.bounds[1] for h in self.history.values() if len(h)), default=0)
        else:
            self._plot_pause_time = None
        self._plot_dirty = True
        self._plot_render_signature = None
        self._last_plot_fit = 0

    def plot_manual_interaction(self, sender=None, app_data=None, user_data=None):
        if self.active_page == "plot" and dpg.is_item_hovered("data_plot"):
            dpg.set_value("plot_follow", False)
            self._plot_dirty = True

    def resume_live_plot(self):
        dpg.set_value("plot_follow", True)
        dpg.set_value("plot_paused", False)
        self.plot_controls_changed()

    def show_all_plot_history(self):
        selected = [name for name, tag in self.plot_checks.items() if dpg.get_value(tag)]
        bounds = [self.history[name].bounds for name in selected if name in self.history and len(self.history[name])]
        if not bounds:
            return
        dpg.set_value("plot_follow", False)
        low, high = min(b[0] for b in bounds), max(b[1] for b in bounds)
        # ImPlot 的显式 limits 会锁轴；设置一次后立即交还给用户缩放。
        dpg.set_axis_limits("plot_x_axis", low, max(high, low + .001))
        self._all_plot_limits = (low, max(high, low + .001))
        self._all_plot_limits_frame = dpg.get_frame_count()
        self._plot_fit_once = True
        self._plot_dirty = True
        self._plot_render_signature = None

    def poll_plot_view(self):
        if hasattr(self, "_all_plot_limits") and dpg.get_frame_count() > self._all_plot_limits_frame:
            # 让一次渲染采用指定范围，再解除持续锁定。
            dpg.set_axis_limits_auto("plot_x_axis")
            del self._all_plot_limits
        if self.active_page == "plot":
            # 停止收包之后的拖动和缩放仍需要重新取历史数据。
            self._plot_dirty = True

    def _update_plot(self):
        now = time.monotonic()
        interval = 1 / int(dpg.get_value("plot_refresh_hz"))
        self._last_plot_refresh = max(self._last_plot_refresh + interval, now - interval)
        selected = {name for name, tag in self.plot_checks.items() if dpg.get_value(tag)}
        for name in list(self.plot_series):
            if name not in selected:
                dpg.delete_item(self.plot_series.pop(name))
        bounds = [self.history[name].bounds for name in selected if name in self.history and len(self.history[name])]
        follow = dpg.get_value("plot_follow")
        if follow and bounds:
            high = max(b[1] for b in bounds)
            if self._plot_pause_time is not None:
                high = min(high, self._plot_pause_time)
            low = max(min(b[0] for b in bounds), high - dpg.get_value("plot_window"))
        else:
            low, high = getattr(self, "_all_plot_limits", dpg.get_axis_limits("plot_x_axis"))
        # 手动查看旧区域时，新数据不影响该区域，避免重复扫描全部历史。
        signature = (tuple(sorted(selected)), low, high, self.max_samples, self._plot_pause_time,
                     tuple((name, h.bounds[0], bisect_right(h.times, min(high, self._plot_pause_time)
                            if self._plot_pause_time is not None else high, lo=h.start))
                           for name in sorted(selected) if (h := self.history.get(name)) and len(h)))
        if signature == self._plot_render_signature and not self._plot_fit_once:
            return
        for name in selected:
            if name not in self.plot_series:
                self.plot_series[name] = dpg.add_line_series([], [], label=name, parent="plot_y_axis")
            history = self.history.get(name)
            dpg.set_value(self.plot_series[name], history.view(low, high, self.max_samples, self._plot_pause_time)
                          if history else [[], []])
        if self._plot_fit_once:
            dpg.fit_axis_data("plot_y_axis")
            self._plot_fit_once = False
        elif follow and bounds and now - self._last_plot_fit >= .1:
            dpg.fit_axis_data("plot_x_axis")
            dpg.fit_axis_data("plot_y_axis")
            self._last_plot_fit = now
        self._plot_render_signature = signature
        self._plot_dirty = False
