#!/usr/bin/env python3

import sys
import os

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from widgets.node_monitor import NodeMonitorWindow
from widgets.topic_monitor import TopicMonitorWindow
from widgets.proc import ProcWindow


def pb_nodemon_cb():
    hide_widgets()
    ui_node_monitor.show_widget()


def pb_topicmon_cb():
    hide_widgets()
    ui_topic_monitor.show_widget()


def pb_proc_cb():
    hide_widgets()
    ui_proc.show_widget()


def hide_widgets():
    for widget in widgets:
        widget.hide()


if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

    app = QApplication(sys.argv)
    with open(SCRIPT_DIR + '/ui/stylesheet.css', 'r') as f:
        style = f.read()
        app.setStyleSheet(style)
    
    path_conf_file = SCRIPT_DIR + '/path.conf'
    if not os.path.exists(path_conf_file):
        with open(path_conf_file, 'w') as f:
            f.write('/opt/ros/humble/setup.bash')
    
    ws_paths = []
    with open(path_conf_file, 'r') as f:
        for path in f.readlines():
            ws_paths.append(path.strip())
    
    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/ros.png'))

    ui_main.pb_nodemon.clicked.connect(pb_nodemon_cb)
    ui_main.pb_topicmon.clicked.connect(pb_topicmon_cb)
    ui_main.pb_proc.clicked.connect(pb_proc_cb)

    ui_node_monitor = NodeMonitorWindow(SCRIPT_DIR + '/ui/node_monitor.ui', SCRIPT_DIR + '/ui/node_monitor_topic.ui', ws_paths)
    ui_topic_monitor = TopicMonitorWindow(SCRIPT_DIR + '/ui/topic_monitor.ui', ws_paths)
    ui_proc = ProcWindow(SCRIPT_DIR + '/ui/proc.ui')

    widgets = []
    widgets.append(ui_node_monitor)
    widgets.append(ui_topic_monitor)
    widgets.append(ui_proc)

    for widget in widgets:
        ui_main.main_box.addWidget(widget)
        widget.hide()
    
    widgets[0].show_widget()
    ui_main.show()

    sys.exit(app.exec())
