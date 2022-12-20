#!/usr/bin/env python3

import sys
import os

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from widgets.monitor import NodeMonitorWindow
from widgets.player import PlayerWindow
from widgets.topicinfo import TopicInfoWindow
from widgets.launcher import LauncherWindow
from widgets.proc import ProcWindow
from widgets.setting import SettingWindow

import util.plotter as plotter
import util.ros2bag2csv as bag2csv


def pb_player_cb():
    hide_widgets()
    ui_player.show()


def pb_nodemon_cb():
    hide_widgets()
    ui_node_monitor.pb_update_cb()
    ui_node_monitor.show()


def pb_topicmon_cb():
    hide_widgets()
    ui_topic_info.pb_update_cb()
    ui_topic_info.show()


def pb_launcher_cb():
    hide_widgets()
    ui_launcher.show()


def pb_proc_cb():
    hide_widgets()
    ui_proc.pb_ref_cb()
    ui_proc.show()


def pb_setting_cb():
    hide_widgets()
    ui_setting.show()


def hide_widgets():
    for widget in widgets:
        widget.hide()


def set_rosdistro(ros_distro):
    for widget in widgets:
        widget.set_rosdistro(ros_distro)


def set_rospath(ros_path):
    for widget in widgets:
        widget.set_rospath(ros_path)


if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

    app = QApplication(sys.argv)
    with open(SCRIPT_DIR + '/ui/stylesheet.css', 'r') as f:
        style = f.read()
        app.setStyleSheet(style)

    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/ros.png'))

    ui_main.pb_player.clicked.connect(pb_player_cb)
    ui_main.pb_nodemon.clicked.connect(pb_nodemon_cb)
    ui_main.pb_topicmon.clicked.connect(pb_topicmon_cb)
    ui_main.pb_launcher.clicked.connect(pb_launcher_cb)
    ui_main.pb_proc.clicked.connect(pb_proc_cb)
    ui_main.pb_setting.clicked.connect(pb_setting_cb)

    ui_player = PlayerWindow(SCRIPT_DIR + '/ui/player.ui')
    ui_node_monitor = NodeMonitorWindow(
        SCRIPT_DIR + '/ui/node_monitor.ui', SCRIPT_DIR + '/ui/node_monitor_topic.ui')
    ui_topic_info = TopicInfoWindow(SCRIPT_DIR + '/ui/topic_info.ui')
    ui_launcher = LauncherWindow(SCRIPT_DIR + '/ui/launcher.ui')
    ui_proc = ProcWindow(SCRIPT_DIR + '/ui/proc.ui')
    ui_setting = SettingWindow(SCRIPT_DIR + '/ui/setting.ui')
    ui_setting.settingRosDistro.connect(set_rosdistro)
    ui_setting.settingRosPath.connect(set_rospath)

    widgets = []
    widgets.append(ui_player)
    widgets.append(ui_node_monitor)
    widgets.append(ui_topic_info)
    widgets.append(ui_launcher)
    widgets.append(ui_proc)
    widgets.append(ui_setting)

    for widget in widgets:
        ui_main.main_box.addWidget(widget)
        widget.hide()
    ui_player.show()

    ui_setting.load_log()
    ui_main.show()

    sys.exit(app.exec())
