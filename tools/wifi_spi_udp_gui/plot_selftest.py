"""EXE 验收：共享滑杆、真实历史样本、暂停和自动/手动视口。"""
import math
import dearpygui.dearpygui as dpg
from plot_history import PlotHistory


def verify_plot(monitor):
    name = next(iter(monitor.plot_checks))
    h = PlotHistory()
    for i in range(10000):
        h.append((i / 100, math.sin(i / 100)))
    monitor.history[name] = h
    dpg.set_value(monitor.plot_checks[name], True)
    monitor.switch_page("plot")
    monitor.resume_live_plot()
    monitor._update_plot()
    dpg.render_dearpygui_frame()
    assert dpg.get_item_info("shared_slider_panel")["parent"] == dpg.get_alias_id("plot_slider_host")
    assert dpg.get_item_configuration("data_plot")["anti_aliased"]
    assert len(h) == 10000
    dpg.set_value("plot_follow", False)
    dpg.set_axis_limits("plot_x_axis", 10, 20)
    dpg.render_dearpygui_frame()
    dpg.set_axis_limits_auto("plot_x_axis")
    monitor._update_plot()
    xs = dpg.get_value(monitor.plot_series[name])[0]
    assert min(xs) < 11 and max(xs) < 21
    dpg.set_value("plot_paused", True)
    monitor.plot_controls_changed()
    h.append((101, 42))
    monitor._update_plot()
    assert max(dpg.get_value(monitor.plot_series[name])[0]) < 101
    monitor.resume_live_plot()
    monitor._update_plot()
    assert max(dpg.get_value(monitor.plot_series[name])[0]) == 101
    monitor.switch_page("params")
    assert dpg.get_item_info("shared_slider_panel")["parent"] == dpg.get_alias_id("params_slider_host")
