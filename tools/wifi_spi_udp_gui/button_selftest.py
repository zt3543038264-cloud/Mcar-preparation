"""打包验收用按钮回环检查，目标地址固定为本机。"""
import socket
import dearpygui.dearpygui as dpg


def verify_buttons(monitor):
    controls = {tag: dpg.get_value(tag) for tag in ("bind_ip", "bind_port", "remote_ip", "remote_port")}
    row = None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink:
            sink.bind(("127.0.0.1", 0))
            sink.settimeout(1)
            dpg.set_value("bind_ip", "127.0.0.1")
            dpg.set_value("bind_port", 0)
            monitor.start_listening()
            dpg.set_value("remote_ip", "127.0.0.1")
            dpg.set_value("remote_port", sink.getsockname()[1])
            row = monitor.add_command_button({"name": "selftest", "press_command": "ON",
                                              "release_command": "OFF", "mode": "latch"})
            monitor.press_command_button(row)
            assert sink.recvfrom(1024)[0] == b"ON\n"
            assert monitor.button_states[row].pressed
            monitor.press_command_button(row)
            assert sink.recvfrom(1024)[0] == b"OFF\n"
            assert not monitor.button_states[row].pressed
            config = monitor.capture_buttons()
            assert monitor.save_workspace(force=True)
            state, warning = monitor.workspace_store.load()
            assert not warning and state["buttons"] == config
            monitor.delete_command_button(row)
            monitor.restore_buttons(state["buttons"])
            assert monitor.capture_buttons() == config
            assert all(not item.pressed for item in monitor.button_states.values())
    finally:
        monitor.stop_listening()
        for tag, value in controls.items():
            dpg.set_value(tag, value)
