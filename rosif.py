#!/usr/bin/env python3

import sys
import os

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from widgets.list import NodeListWindow, TopicListWindow
from widgets.info import NodeInfoWindow, TopicInfoWindow
from widgets.topic_monitor import TopicMonitorWindow
from widgets.proc import ProcWindow


def pb_node_list_cb():
    hide_widgets()
    ui_node_list.show()

def pb_node_info_cb():
    hide_widgets()
    ui_node_info.show()

def pb_topic_list_cb():
    hide_widgets()
    ui_topic_list.show()

def pb_topic_info_cb():
    hide_widgets()
    ui_topic_info.show()

def pb_topic_mon_cb():
    hide_widgets()
    ui_topic_mon.show()

def pb_proc_cb():
    hide_widgets()
    ui_proc.show()

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
            f.write('/opt/ros/humble/setup.bash\n')
    
    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/rosif.png'))

    ui_main.pb_node_list.clicked.connect(pb_node_list_cb)
    ui_main.pb_node_info.clicked.connect(pb_node_info_cb)
    ui_main.pb_topic_list.clicked.connect(pb_topic_list_cb)
    ui_main.pb_topic_info.clicked.connect(pb_topic_info_cb)
    ui_main.pb_topic_mon.clicked.connect(pb_topic_mon_cb)
    ui_main.pb_proc.clicked.connect(pb_proc_cb)

    ui_node_list = NodeListWindow(SCRIPT_DIR)
    ui_node_info = NodeInfoWindow(SCRIPT_DIR)
    ui_topic_list = TopicListWindow(SCRIPT_DIR)
    ui_topic_info = TopicInfoWindow(SCRIPT_DIR)
    ui_topic_mon = TopicMonitorWindow(SCRIPT_DIR)
    ui_proc = ProcWindow(SCRIPT_DIR)

    widgets = []
    widgets.append(ui_node_list)
    widgets.append(ui_node_info)
    widgets.append(ui_topic_list)
    widgets.append(ui_topic_info)
    widgets.append(ui_topic_mon)
    widgets.append(ui_proc)

    for widget in widgets:
        ui_main.main_box.addWidget(widget)
        widget.hide()
    
    widgets[0].show()
    ui_main.show()

    sys.exit(app.exec())
