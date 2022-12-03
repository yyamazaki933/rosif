#!/usr/bin/env python3

import os
import re
import yaml
import subprocess

from PyQt5 import uic
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtGui import QTextCursor

import monitor.player as player


class PlayerWindow():

    def __init__(self, ui_file):
        self.home_dir = os.getenv('HOME')
        self.player = None
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.ui_file = ui_file

        self.ui = uic.loadUi(ui_file)
        self.ui.tb_bag.clicked.connect(self.tb_bag)
        self.ui.pb_play.clicked.connect(self.pb_play)
        self.ui.pb_pause.clicked.connect(self.pb_pause)
        self.ui.pb_reset.clicked.connect(self.pb_reset)
        self.ui.sb_offset.valueChanged.connect(self.set_progress_offset)
    
    def save_log(self):
        rosbag_dir = self.ui.le_bag.text()

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
            
            self.ui.le_bag.setText(rosbag_dir)

        except FileNotFoundError:
            self.save_log()

    def show(self):
        self.load_log()
        self.bag_info()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro
    
    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def tb_bag(self):
        bag = QFileDialog.getExistingDirectory(
            self.ui, 'Choose Rosbag2 Directory', self.home_dir)

        if bag != '':
            self.ui.le_bag.setText(bag)
            self.bag_info()
            self.save_log()

    def bag_info(self):
        bag = self.ui.le_bag.text()
        
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
            self.ui.pb_play.setEnabled(False)

        else:
            self.ui.pb_play.setEnabled(True)
            lines = resp.stdout.split('\n')
            for line in lines:
                if line == '':
                    continue
                info += line + '\n'

                if 'Duration:' in line:
                    dur = int(re.split('[:.]', line)[1])

        self.ui.pte_bag.clear()
        self.ui.pte_bag.setPlainText(info)

        self.ui.progress.setRange(0, dur)
        self.ui.progress.reset()
        self.ui.progress.setValue(0)

        row0 = self.ui.pte_bag.document().findBlockByLineNumber(0)
        self.ui.pte_bag.setTextCursor(QTextCursor(row0))

    def pb_play(self):
        rosbag_dir = self.ui.le_bag.text()
        rate = self.ui.sb_rate.value()
        start = self.ui.sb_offset.value()

        self.player = player.RosbagPlayer()
        self.player.playerProglessTick.connect(self.timer_callback)
        self.player.playerFinished.connect(self.pb_reset)
        self.player.setRosbag(rosbag_dir)
        self.player.setSource(self.ros_path)
        self.player.setRate(rate)
        self.player.setStartOffset(start)
        self.player.start()

        self.ui.pb_play.setEnabled(False)
        self.ui.sb_rate.setEnabled(False)
        self.ui.sb_offset.setEnabled(False)
        self.ui.pb_reset.setEnabled(True)
        self.ui.pb_pause.setEnabled(True)

    def pb_pause(self):
        self.player.pause()
        if self.player.is_running:
            self.ui.pb_pause.setText('Pause')
        else:
            self.ui.pb_pause.setText('Resume')

    def pb_reset(self):
        self.player.stop()
        self.player = None

        start = self.ui.sb_offset.value()
        self.ui.progress.setValue(start)
        self.ui.pb_play.setEnabled(True)
        self.ui.sb_rate.setEnabled(True)
        self.ui.sb_offset.setEnabled(True)
        self.ui.pb_reset.setEnabled(False)
        self.ui.pb_pause.setEnabled(False)
        self.ui.pb_pause.setText('Pause')

    def timer_callback(self, elapsed):
        self.ui.progress.setValue(elapsed)
    
    def set_progress_offset(self, value):
        self.ui.progress.setValue(value)
