#!/usr/bin/env python3

import sys
import os

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from widgets.monitor import MonitorWindow
from widgets.player import PlayerWindow
from widgets.debug import DebugWindow
from widgets.proc import ProcWindow
import util.plotter as plotter
import util.ros2bag2csv as bag2csv


def set_rosdistro():
    ros_distro = ui_main.le_distro.text()
    
    ui_player.set_rosdistro(ros_distro)
    ui_debug.set_rosdistro(ros_distro)
    ui_monitor.set_rosdistro(ros_distro)
    ui_proc.set_rosdistro(ros_distro)


def pb_player():
    ui_player.show()


def pb_debugger():
    ui_debug.show()


def pb_monitor():
    ui_monitor.show()


def pb_proc():
    ui_proc.show()


#################
# utility
#################

# def tb_wcsv():
#     bag = main_ui.le_bag.text()
#     topic = main_ui.cb_bagtopic.currentText()

#     bag_dir = os.path.dirname(bag)
#     result = bag2csv.writeCSV(bag_dir, topic)


# def tb_rcsv():
#     result = QFileDialog.getOpenFileName(
#         main_ui, 'Choose ROS CSV File', HOME_DIR, 'CSV File (*.csv)')[0]

#     if result == '':
#         return

#     main_ui.le_csv.clear()
#     main_ui.cb_xaxis.clear()
#     main_ui.cb_yaxis.clear()

#     main_ui.le_csv.setText(result)


# def pb_plot():
#     bag_dir = main_ui.le_csv.text()

#     x_axis = 'msg.pose.position.x'
#     y_axis = 'msg.pose.position.y'
#     # x_axis = main_ui.cb_xaxis.currentText()
#     # y_axis = main_ui.cb_yaxis.currentText()

#     result = plotter.plot(bag_dir, x_axis, y_axis)


# def pb_kml():
#     bag_dir = main_ui.le_csv.text()

#     long_axis = 'msg.longitude'
#     lat_axis = 'msg.latitude'

#     result = plotter.writeKML(bag_dir, long_axis, lat_axis)


if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    HOME_DIR = os.getenv('HOME')

    app = QApplication(sys.argv)

    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/ros.png'))
    ui_main.tb_distro.clicked.connect(set_rosdistro)
    ui_main.pb_player.clicked.connect(pb_player)
    ui_main.pb_debugger.clicked.connect(pb_debugger)
    ui_main.pb_monitor.clicked.connect(pb_monitor)
    # ui_main.pb_launcher.clicked.connect(pb_launcher)
    ui_main.pb_proc.clicked.connect(pb_proc)

    ui_player = PlayerWindow(SCRIPT_DIR + '/ui/player.ui')
    ui_debug = DebugWindow(SCRIPT_DIR + '/ui/debug.ui')
    ui_monitor = MonitorWindow(SCRIPT_DIR + '/ui/monitor.ui')
    ui_proc = ProcWindow(SCRIPT_DIR + '/ui/proc.ui')

    # ui_util.tb_bag.clicked.connect(tb_bag)
    # ui_util.tb_wcsv.clicked.connect(tb_wcsv)
    # ui_util.tb_rcsv.clicked.connect(tb_rcsv)
    # ui_util.tb_plot.clicked.connect(tb_plot)
    # ui_util.tb_kml.clicked.connect(tb_kml)

    set_rosdistro()
    ui_debug.set_monitor(ui_monitor)

    ui_main.show()

    sys.exit(app.exec())
