"""真实窗口验证共享滑杆、历史回看、手动视角和暂停继续采集。"""
import ctypes
import ctypes.wintypes
import math
from pathlib import Path
import socket
import tempfile
import time
import dearpygui.dearpygui as dpg
from plot_history import PlotHistory
from test_slider_gui import open_monitor


def frames(monitor, count=8):
    for _ in range(count):
        dpg.run_callbacks(dpg.get_callback_queue())
        monitor.poll_plot_view()
        monitor.refresh()
        dpg.render_dearpygui_frame()
        time.sleep(.018)
    dpg.run_callbacks(dpg.get_callback_queue())


def run():
    with tempfile.TemporaryDirectory(prefix="wifispi-plot-test-") as directory:
        monitor = open_monitor(Path(directory) / "workspace.json")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
                sink.bind(("127.0.0.1", 0))
                sink.settimeout(1)
                dpg.set_value("bind_ip", "127.0.0.1")
                dpg.set_value("bind_port", 0)
                monitor.start_listening()
                dpg.set_value("remote_ip", "127.0.0.1")
                dpg.set_value("remote_port", sink.getsockname()[1])
                h = PlotHistory()
                for i in range(10000):
                    h.append((i * .01, math.sin(i * .01)))
                monitor.history["roll"] = h
                dpg.set_value(monitor.plot_checks["roll"], True)
                monitor.switch_page("plot")
                frames(monitor)
                assert dpg.get_item_info("shared_slider_panel")["parent"] == dpg.get_alias_id("plot_slider_host")
                assert dpg.get_item_configuration("data_plot")["anti_aliased"]
                assert len(h) == 10000
                latest = dpg.get_axis_limits("plot_x_axis")
                assert latest[0] > 80 and latest[1] >= 99, latest
                # 同一组滑杆在绘图页发包，返回调参页保留同一数值及控件 ID。
                row = monitor.slider_rows[0]
                monitor.on_slider_changed(row, 2.501)
                assert sink.recvfrom(1024)[0] == b"[slider,pos_xy_kp,2.501]"
                monitor.switch_page("params")
                assert dpg.get_value(f"slider_handle_{row}") == 2.501
                monitor.switch_page("plot")
                frames(monitor)
                # 原生滚轮操作必须自动退出实时跟随，之后新数据不拉回视角。
                hwnd = ctypes.windll.user32.FindWindowW(None, "WiFiSPI Slider Regression Test")
                ctypes.windll.user32.SetForegroundWindow(hwnd)
                frames(monitor, 12)
                point, size = dpg.get_item_rect_min("data_plot"), dpg.get_item_rect_size("data_plot")
                x, y = int(point[0] + size[0] / 2), int(point[1] + size[1] / 2)
                p = ctypes.wintypes.POINT(x, y)
                ctypes.windll.user32.ClientToScreen(hwnd, ctypes.byref(p))
                ctypes.windll.user32.SetCursorPos(p.x, p.y)
                ctypes.windll.user32.PostMessageW(hwnd, 0x200, 0, (y << 16) | x)
                frames(monitor)
                ctypes.windll.user32.PostMessageW(hwnd, 0x20A, 120 << 16, (p.y << 16) | p.x)
                frames(monitor)
                assert not dpg.get_value("plot_follow")
                dpg.set_axis_limits("plot_x_axis", 10, 20)
                frames(monitor, 1)
                dpg.set_axis_limits_auto("plot_x_axis")
                for i in range(100):
                    h.append((100 + i * .01, 2))
                frames(monitor)
                low, high = dpg.get_axis_limits("plot_x_axis")
                assert abs(low - 10) < .001 and abs(high - 20) < .001, (low, high)
                xs = dpg.get_value(monitor.plot_series["roll"])[0]
                assert min(xs) < 11 and max(xs) < 21, (min(xs), max(xs))
                monitor.show_all_plot_history()
                frames(monitor)
                low, high = dpg.get_axis_limits("plot_x_axis")
                assert low <= .001 and high >= 100.9, (low, high)
                dpg.set_value("plot_paused", True)
                monitor.plot_controls_changed()
                frozen = monitor._plot_pause_time
                h.append((102, 1000))
                frames(monitor)
                assert max(dpg.get_value(monitor.plot_series["roll"])[0]) <= frozen
                assert h.bounds[1] == 102
                monitor.resume_live_plot()
                frames(monitor)
                assert max(dpg.get_value(monitor.plot_series["roll"])[0]) == 102
                monitor.stop_listening()
                assert not dpg.get_value("plot_follow")
                frames(monitor)
        finally:
            monitor.shutdown()
            dpg.destroy_context()
    print("PASS: shared sliders UDP, native wheel, stable manual axes, full history, pause/resume", flush=True)


if __name__ == "__main__":
    run()
