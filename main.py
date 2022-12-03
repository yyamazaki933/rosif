#!/usr/bin/env python3

import sys
import os
import yaml

from PyQt5.QtWidgets import QApplication, QFileDialog
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from widgets.monitor import TopicMonitorWindow
from widgets.monitor import NodeMonitorWindow
from widgets.player import PlayerWindow
from widgets.topicinfo import TopicInfoWindow
from widgets.proc import ProcWindow
import util.plotter as plotter
import util.ros2bag2csv as bag2csv


def set_rosdistro():
    ros_distro = ui_main.le_distro.text()
    ui_player.set_rosdistro(ros_distro)
    ui_topic_monitor.set_rosdistro(ros_distro)
    ui_node_monitor.set_rosdistro(ros_distro)
    save_log()


def tb_path():
    result = QFileDialog.getOpenFileName(
        ui_main, 'Choose Optional Path File', HOME_DIR, 'Bash File (*.bash)')[0]
    if result == '':
        return
    ui_main.le_path.setText(result)
    set_rospath()


def set_rospath():
    rospath = ui_main.le_path.text()
    ui_player.set_rospath(rospath)
    ui_topic_monitor.set_rospath(rospath)
    ui_node_monitor.set_rospath(rospath)
    save_log()


def save_log():
    rosdistro = ui_main.le_distro.text()
    rospath = ui_main.le_path.text()

    log = {
        'rosdistro': rosdistro,
        'rospath': rospath,
    }

    with open(SCRIPT_DIR + '/ui/main.ui' + '.log', 'w') as file:
        yaml.dump(log, file)


def load_log():
    try:
        with open(SCRIPT_DIR + '/ui/main.ui' + '.log') as file:
            log = yaml.safe_load(file)
            rosdistro = log['rosdistro']
            rospath = log['rospath']

        ui_main.le_distro.setText(rosdistro)
        ui_main.le_path.setText(rospath)

    except FileNotFoundError:
        save_log()


if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    HOME_DIR = os.getenv('HOME')

    app = QApplication(sys.argv)

    ui_player = PlayerWindow(SCRIPT_DIR + '/ui/player.ui')
    ui_topicinfo = TopicInfoWindow(SCRIPT_DIR + '/ui/topic_info.ui')
    ui_node_monitor = NodeMonitorWindow(SCRIPT_DIR + '/ui/node_monitor.ui', SCRIPT_DIR + '/ui/node_monitor_topic.ui')
    ui_topic_monitor = TopicMonitorWindow(SCRIPT_DIR + '/ui/topic_monitor.ui')
    ui_proc = ProcWindow(SCRIPT_DIR + '/ui/proc.ui')

    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/ros.png'))
    ui_main.tb_distro.clicked.connect(set_rosdistro)
    ui_main.tb_path.clicked.connect(tb_path)
    
    ui_main.player_box.addWidget(ui_player)
    ui_main.nodem_box.addWidget(ui_node_monitor)
    ui_main.topicm_box.addWidget(ui_topic_monitor)
    ui_main.topicinfo_box.addWidget(ui_topicinfo)
    ui_main.proc_box.addWidget(ui_proc)

    # ui_util.tb_bag.clicked.connect(tb_bag)
    # ui_util.tb_wcsv.clicked.connect(tb_wcsv)
    # ui_util.tb_rcsv.clicked.connect(tb_rcsv)
    # ui_util.tb_plot.clicked.connect(tb_plot)
    # ui_util.tb_kml.clicked.connect(tb_kml)

    load_log()
    set_rospath()
    set_rosdistro()

    ui_main.show()

    sys.exit(app.exec())
