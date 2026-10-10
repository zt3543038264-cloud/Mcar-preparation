"""真实窗口验收：按住/自锁、按钮外松开、配置恢复，UDP 仅发往本机。"""
import ctypes
import ctypes.wintypes
from pathlib import Path
import socket
import tempfile
import time
import dearpygui.dearpygui as dpg
from test_slider_gui import open_monitor


def frames(monitor, count=5):
    for _ in range(count):
        dpg.run_callbacks(dpg.get_callback_queue())
        monitor.poll_command_buttons()
        dpg.render_dearpygui_frame()
        time.sleep(.015)
    dpg.run_callbacks(dpg.get_callback_queue())
    monitor.poll_command_buttons()


def mouse(monitor, row, down, outside=False):
    hwnd = ctypes.windll.user32.FindWindowW(None, "WiFiSPI Slider Regression Test")
    assert hwnd
    point = dpg.get_item_rect_min(f"cmd_trigger_{row}")
    size = dpg.get_item_rect_size(f"cmd_trigger_{row}")
    x, y = int(point[0] + size[0] / 2), int(point[1] + size[1] / 2)
    if outside:
        x, y = 5, 5
    p = ctypes.wintypes.POINT(x, y)
    ctypes.windll.user32.ClientToScreen(hwnd, ctypes.byref(p))
    ctypes.windll.user32.SetCursorPos(p.x, p.y)
    ctypes.windll.user32.PostMessageW(hwnd, 0x200, 0, (y << 16) | x)
    frames(monitor, 2)
    ctypes.windll.user32.PostMessageW(hwnd, 0x201 if down else 0x202, int(down), (y << 16) | x)
    frames(monitor)


def run():
    with tempfile.TemporaryDirectory(prefix="wifispi-buttons-test-") as directory:
        path = Path(directory) / "workspace.json"
        monitor = open_monitor(path)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
                sink.bind(("127.0.0.1", 0))
                sink.settimeout(.3)
                dpg.set_value("bind_ip", "127.0.0.1")
                dpg.set_value("bind_port", 0)
                monitor.start_listening()
                dpg.set_value("remote_ip", "127.0.0.1")
                dpg.set_value("remote_port", sink.getsockname()[1])
                config = {"name": "速度开关", "press_command": "[slider,Vx_cmps,20]",
                          "release_command": "[slider,Vx_cmps,0]", "newline": False}
                row = monitor.add_command_button(config)
                dpg.set_y_scroll("page_params", 430)
                frames(monitor)
                mouse(monitor, row, True)
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,20]"
                assert monitor.button_states[row].pressed
                frames(monitor, 30)
                with_timeout = False
                try:
                    sink.recvfrom(1024)
                except socket.timeout:
                    with_timeout = True
                assert with_timeout, "按住不能连续发包"
                mouse(monitor, row, False, outside=True)
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,0]"
                assert not monitor.button_states[row].pressed
                # 自锁第一次按下后保持，第二次按下才发抬起命令。
                dpg.set_value(f"cmd_mode_{row}", "自锁")
                frames(monitor)
                mouse(monitor, row, True)
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,20]"
                mouse(monitor, row, False)
                assert monitor.button_states[row].pressed
                monitor.switch_page("plot")
                frames(monitor)
                assert monitor.button_states[row].pressed
                monitor.switch_page("params")
                frames(monitor)
                mouse(monitor, row, True)
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,0]"
                mouse(monitor, row, False)
                assert not monitor.button_states[row].pressed
                # 停止监听前必须先发抬起；新增/删除的列表也保存。
                monitor.press_command_button(row)
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,20]"
                extra = monitor.add_command_button({"name": "临时"})
                monitor.delete_command_button(extra)
                expected = monitor.capture_buttons()
                monitor.stop_listening()
                assert sink.recvfrom(1024)[0] == b"[slider,Vx_cmps,0]"
        finally:
            monitor.shutdown()
            dpg.destroy_context()
        reopened = open_monitor(path)
        try:
            assert reopened.capture_buttons() == expected
            assert not reopened.receiver.is_running
            assert all(not state.pressed for state in reopened.button_states.values())
            for row in list(reopened.button_rows):
                reopened.delete_command_button(row)
        finally:
            reopened.shutdown()
            dpg.destroy_context()
        empty = open_monitor(path)
        try:
            assert empty.button_rows == []
        finally:
            empty.shutdown()
            dpg.destroy_context()
    print("PASS: native momentary/latch, outside release, page switch, stop, persistence", flush=True)


if __name__ == "__main__":
    run()
