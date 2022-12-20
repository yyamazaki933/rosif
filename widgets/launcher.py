#!/usr/bin/env python3

import os
import subprocess
import xml.etree.ElementTree as ET

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QFileDialog


class LauncherWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.home_dir = os.getenv('HOME')
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.is_running = False

        self.pb_xml.clicked.connect(self.pb_xml_cb)
        self.pb_launch.clicked.connect(self.pb_launch_cb)

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def pb_xml_cb(self):
        xml = QFileDialog.getOpenFileName(
            self, 'Choose Launch XML File', self.home_dir, 'Launch XML File (*.launch.xml)')[0]
        if xml == '':
            return

        self.le_xml.setText(xml)

        tree = ET.parse(xml) # xml.etree.ElementTree.ElementTree object
        root = tree.getroot()
        args = root.findall('arg')
        for arg in args:
            arg_name = arg.attrib['name']
            try:
                arg_val = arg.attrib['default']
            except:
                arg_val = ''
            print(arg_name, arg_val)

    def pb_launch_cb(self):
        xml = self.le_xml.text()

        cmd = 'source ' + self.ros_path
        cmd += ' && '
        cmd += 'exec ros2 launch ' + xml

        print(cmd)

        self.cmd_proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash')
        self.is_running = True
