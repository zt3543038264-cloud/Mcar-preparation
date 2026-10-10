"""渲染真实 GUI，验证 Windows 鼠标/键盘消息、UDP 步长与重新打开存档。

运行 python test_slider_gui.py；只使用回环地址和临时存档，不发送实车指令。
"""
import ctypes
import ctypes.wintypes
from decimal import Decimal
from pathlib import Path
import socket
import tempfile
import time

import dearpygui.dearpygui as dpg
from app import WifiSpiMonitor


def frames(count=5):
    for _ in range(count):
        dpg.run_callbacks(dpg.get_callback_queue())
        dpg.render_dearpygui_frame()
        time.sleep(.015)
    dpg.run_callbacks(dpg.get_callback_queue())


def open_monitor(path):
    dpg.create_context()
    dpg.configure_app(manual_callback_management=True)
    monitor = WifiSpiMonitor()
    monitor.initialize_workspace("Test", __file__, path)
    monitor.build_ui()
    dpg.create_viewport(title="WiFiSPI Slider Regression Test", width=1400, height=1000)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)
    frames()
    return monitor


def run():
    with tempfile.TemporaryDirectory(prefix="wifispi-gui-test-") as directory:
        path = Path(directory) / "workspace.json"
        monitor = open_monitor(path)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
                sink.bind(("127.0.0.1", 0))
                sink.settimeout(1)
                dpg.set_value("bind_ip", "127.0.0.1")
                dpg.set_value("bind_port", 0)
                monitor.start_listening()
                dpg.set_value("remote_ip", "127.0.0.1")
                dpg.set_value("remote_port", sink.getsockname()[1])
                row = monitor.slider_rows[0]
                dpg.set_value(f"slider_name_{row}", "pos_xy_kp")
                monitor.on_slider_changed(row, 2.5)
                assert sink.recvfrom(1024)[0] == b"[slider,pos_xy_kp,2.5]"
                hwnd = ctypes.windll.user32.FindWindowW(None, "WiFiSPI Slider Regression Test")
                assert hwnd, "测试视口不存在"
                post = ctypes.windll.user32.PostMessageW
                # 只向测试窗口投递消息；不向其他正在使用的应用注入按键。
                point = dpg.get_item_rect_min(f"slider_handle_{row}")
                size = dpg.get_item_rect_size(f"slider_handle_{row}")
                x, y = int(point[0] + size[0] / 2), int(point[1] + size[1] / 2)
                client_point = ctypes.wintypes.POINT(x, y)
                ctypes.windll.user32.ClientToScreen(hwnd, ctypes.byref(client_point))
                ctypes.windll.user32.SetCursorPos(client_point.x, client_point.y)
                post(hwnd, 0x200, 0, (y << 16) | x)
                frames()
                post(hwnd, 0x201, 1, (y << 16) | x)
                frames()
                post(hwnd, 0x202, 0, (y << 16) | x)
                frames()
                assert monitor.selected_slider == row, (point, size, monitor.selected_slider)
                monitor.on_slider_changed(row, 2.5)
                while sink.recvfrom(1024)[0] != b"[slider,pos_xy_kp,2.5]":
                    pass  # 清除鼠标点击产生的刻度包。
                post(hwnd, 0x100, 0x26, 0)  # WM_KEYDOWN / VK_UP
                frames()
                assert monitor.slider_values[row] == Decimal("2.501"), monitor.slider_values[row]
                assert sink.recvfrom(1024)[0] == b"[slider,pos_xy_kp,2.501]"
                tx = monitor.slider_tx_count
                frames(30)  # 长按不自动重复发送。
                assert monitor.slider_tx_count == tx
                post(hwnd, 0x101, 0x26, 0)  # WM_KEYUP
                frames()
                post(hwnd, 0x100, 0x28, 0)  # VK_DOWN
                frames()
                assert monitor.slider_values[row] == Decimal("2.5"), monitor.slider_values[row]
                assert sink.recvfrom(1024)[0] == b"[slider,pos_xy_kp,2.5]"
                post(hwnd, 0x101, 0x28, 0)
                frames()
                # 编辑框取得焦点时，上下键不能偷偷更改另一个参数。
                monitor.select_slider(row)
                dpg.focus_item(f"slider_name_{row}")
                frames()
                post(hwnd, 0x100, 0x26, 0)
                frames()
                assert monitor.slider_values[row] == Decimal("2.5")
                post(hwnd, 0x101, 0x26, 0)
                frames()
                # 自定义步长、负值、新增和删除的滑杆都要保存。
                dpg.set_value(f"slider_step_{row}", .005)
                monitor.apply_slider_range(row)
                monitor.on_slider_changed(row, 2.515)
                assert sink.recvfrom(1024)[0] == b"[slider,pos_xy_kp,2.515]"
                monitor.add_slider_row("custom_gain", -5, 5, .0025, -1.125)
                monitor.delete_slider_row(monitor.slider_rows[1])
                expected = monitor.capture_workspace()
        finally:
            monitor.shutdown()
            dpg.destroy_context()
        reopened = open_monitor(path)
        try:
            assert reopened.capture_workspace() == expected
            assert reopened.slider_tx_count == 0 and not reopened.receiver.is_running
            for row in list(reopened.slider_rows):
                reopened.delete_slider_row(row)
        finally:
            reopened.shutdown()
            dpg.destroy_context()
        empty = open_monitor(path)
        try:
            assert empty.slider_rows == [], "删除全部后不能恢复默认滑杆"
        finally:
            empty.shutdown()
            dpg.destroy_context()
    print("PASS: native keys, exact UDP, custom step, close/reopen, empty workspace", flush=True)


if __name__ == "__main__":
    run()
