#!/usr/bin/env python3

import os
import yaml

from PyQt5 import uic, QtWidgets, QtCore
from PyQt5.QtWidgets import QFileDialog


class SettingWindow(QtWidgets.QWidget):

    settingRosDistro = QtCore.pyqtSignal(str)
    settingRosPath = QtCore.pyqtSignal(str)

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)
        self.ui_file = ui_file
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        
        self.pb_rosdistro.clicked.connect(self.pb_rosdistro_cb)
        self.pb_path.clicked.connect(self.pb_path_cb)

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro
        self.le_distro.setText(ros_distro)
        self.save_log()
    
    def set_rospath(self, ros_path):
        self.ros_path = ros_path
        self.le_path.setText(ros_path)
        self.save_log()

    def pb_rosdistro_cb(self):
        rosdistro = self.le_distro.text()
        self.le_distro.setText(rosdistro)
        self.settingRosDistro.emit(rosdistro)

    def pb_path_cb(self):
        HOME_DIR = os.getenv('HOME')
        rospath = QFileDialog.getOpenFileName(
            self, 'Choose Optional Path File', HOME_DIR, 'Bash File (*.bash)')[0]
        if rospath == '':
            return
        self.le_path.setText(rospath)
        self.settingRosPath.emit(rospath)

    def save_log(self):
        rosdistro = self.le_distro.text()
        rospath = self.le_path.text()
        log = {
            'rosdistro': rosdistro,
            'rospath': rospath,
        }
        with open(self.ui_file + '.log', 'w') as file:
            yaml.dump(log, file)

    def load_log(self):
        try:
            with open(self.ui_file + '.log') as file:
                log = yaml.safe_load(file)
                rosdistro = log['rosdistro']
                rospath = log['rospath']

            self.le_distro.setText(rosdistro)
            self.le_path.setText(rospath)
            self.settingRosDistro.emit(rosdistro)
            self.settingRosPath.emit(rospath)
        except FileNotFoundError:
            self.save_log()