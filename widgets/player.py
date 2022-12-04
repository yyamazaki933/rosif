#!/usr/bin/env python3

import os
import re
import yaml
import subprocess

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtGui import QTextCursor

import monitor.player as player


class PlayerWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.ui_file = ui_file

        self.load_log()
        self.bag_info()

        self.pb_bag.clicked.connect(self.pb_bag_cb)
        self.pb_play.clicked.connect(self.pb_play_cb)
        self.pb_pause.clicked.connect(self.pb_pause_cb)
        self.pb_reset.clicked.connect(self.pb_reset_cb)
        self.sb_offset.valueChanged.connect(self.set_progress_offset)

    def save_log(self):
        rosbag_dir = self.le_bag.text()

        log = { 
            'rosbag_dir': rosbag_dir, 
            }

        with open(self.ui_file + '.log', 'w') as file:
            yaml.dump(log, file)

    def load_log(self):
        try:
            with open(self.ui_file + '.log') as file:
                log = yaml.safe_load(file)
                rosbag_dir = log['rosbag_dir']
            
            self.le_bag.setText(rosbag_dir)

        except FileNotFoundError:
            self.save_log()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro
    
    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def pb_bag_cb(self):
        bag = QFileDialog.getExistingDirectory(
            self, 'Choose Rosbag2 Directory', self.home_dir)

        if bag != '':
            self.le_bag.setText(bag)
            self.bag_info()
            self.save_log()

    def bag_info(self):
        bag = self.le_bag.text()
        
        if self.player != None:
            return

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 bag info ' + bag
        resp = subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)

        info = ''
        dur = 0
        if resp.stdout == '':
            info = resp.stderr
            self.pb_play.setEnabled(False)

        else:
            self.pb_play.setEnabled(True)
            lines = resp.stdout.split('\n')
            for line in lines:
                if line == '':
                    continue
                info += line + '\n'

                if 'Duration:' in line:
                    dur = int(re.split('[:.]', line)[1])

        self.pte_bag.clear()
        self.pte_bag.setPlainText(info)

        self.progress.setRange(0, dur)
        self.progress.reset()
        self.progress.setValue(0)

        row0 = self.pte_bag.document().findBlockByLineNumber(0)
        self.pte_bag.setTextCursor(QTextCursor(row0))

    def pb_play_cb(self):
        rosbag_dir = self.le_bag.text()
        rate = self.sb_rate.value()
        start = self.sb_offset.value()

        self.player = player.RosbagPlayer()
        self.player.playerProglessTick.connect(self.progress_cb)
        self.player.playerFinished.connect(self.pb_reset_cb)
        self.player.setRosbag(rosbag_dir)
        self.player.setSource(self.ros_path)
        self.player.setRate(rate)
        self.player.setStartOffset(start)
        self.player.start()

        self.pb_play.setEnabled(False)
        self.sb_rate.setEnabled(False)
        self.sb_offset.setEnabled(False)
        self.pb_reset.setEnabled(True)
        self.pb_pause.setEnabled(True)

    def pb_pause_cb(self):
        self.player.pause()
        if self.player.is_running:
            self.pb_pause.setText('Pause')
        else:
            self.pb_pause.setText('Resume')

    def pb_reset_cb(self):
        self.player.stop()
        self.player = None

        start = self.sb_offset.value()
        self.progress.setValue(start)
        self.pb_play.setEnabled(True)
        self.sb_rate.setEnabled(True)
        self.sb_offset.setEnabled(True)
        self.pb_reset.setEnabled(False)
        self.pb_pause.setEnabled(False)
        self.pb_pause.setText('Pause')

    def progress_cb(self, elapsed):
        self.progress.setValue(elapsed)
    
    def set_progress_offset(self, value):
        self.progress.setValue(value)
