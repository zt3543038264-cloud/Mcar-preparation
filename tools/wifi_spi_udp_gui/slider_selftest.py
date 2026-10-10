"""EXE 验收：滑杆键盘回调、实际 UDP 回环与持久化，绝不发送实车数据。"""
from decimal import Decimal
import socket
import dearpygui.dearpygui as dpg


def verify_sliders(monitor):
    saved_controls = {tag: dpg.get_value(tag) for tag in ("bind_ip", "bind_port", "remote_ip", "remote_port")}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
            sink.bind(("127.0.0.1", 0))
            sink.settimeout(1)
            dpg.set_value("bind_ip", "127.0.0.1")
            dpg.set_value("bind_port", 0)
            monitor.start_listening()
            dpg.set_value("remote_ip", "127.0.0.1")
            dpg.set_value("remote_port", sink.getsockname()[1])
            row = monitor.add_slider_row("test_gain", -1, 10, .001, 2.5)
            monitor.on_slider_changed(row, 2.501)
            assert sink.recvfrom(1024)[0] == b"[slider,test_gain,2.501]"
            monitor.select_slider(row)
            monitor.on_slider_key_press(None, dpg.mvKey_Up, 1)
            assert sink.recvfrom(1024)[0] == b"[slider,test_gain,2.502]"
            monitor.on_slider_key_press(None, dpg.mvKey_Up, 1)
            assert monitor.slider_values[row] == Decimal("2.502")
            monitor.on_slider_key_release(None, dpg.mvKey_Up)
            monitor.on_slider_key_press(None, dpg.mvKey_Down, -1)
            assert sink.recvfrom(1024)[0] == b"[slider,test_gain,2.501]"
            monitor.on_slider_key_release(None, dpg.mvKey_Down)
            monitor.delete_slider_row(row)
    finally:
        monitor.stop_listening()
        for tag, value in saved_controls.items():
            dpg.set_value(tag, value)
    expected = monitor.capture_workspace()
    assert monitor.save_workspace(force=True)
    state, warning = monitor.workspace_store.load()
    assert state == expected and not warning
    count = monitor.slider_tx_count
    for row in list(monitor.slider_rows):
        monitor.delete_slider_row(row)
    monitor._saved_workspace = state
    monitor.restore_workspace([])
    assert monitor.capture_workspace() == expected
    assert monitor.slider_tx_count == count, "恢复工作区不能自动下发参数"
