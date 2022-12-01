#!/usr/bin/env python3

import os
import subprocess

from PyQt5 import uic
from PyQt5.QtWidgets import QMessageBox

class ProcWindow():

    def __init__(self, ui_file):
        self.home_dir = os.getenv('HOME')
        self.ros_distro = 'humble'

        self.ui = uic.loadUi(ui_file)
        self.ui.pb_ref.clicked.connect(self.pb_ref)
        self.ui.pb_kill.clicked.connect(self.pb_kill)
        self.ui.lw_proc.itemDoubleClicked.connect(self.pb_kill)

    def show(self):
        self.pb_ref()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def pb_ref(self):
        self.ui.lw_proc.clear()

        cmd = 'ps -A -f | grep ros2'
        resp = subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
        
        lines = resp.stdout.split('\n')

        for item in lines:
            if item == '':
                continue

            item_vec = item.split()
            pid = item_vec[1]
            pid = format(pid, '>10')
            proc = str.join(' ', item_vec[7:])

            if "grep ros2" in proc:
                continue
            
            self.ui.lw_proc.addItem(pid + ' : ' + proc)
    
    def pb_kill(self):
        select = self.ui.lw_proc.currentItem()

        if select == None:
            return
        item_vec = select.text().split()
        pid = item_vec[0]
        proc = str.join(' ', item_vec[2:])
        
        message = "Kill the process ?\n"
        message += " PID : " + pid + '\n'
        message += " CMD : " + proc
        resp = QMessageBox.warning(self.ui, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)
        
        if resp == QMessageBox.Yes:

            cmd = 'kill -9 ' + pid
            print(cmd)

            resp = subprocess.run(
                cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
            
            self.pb_ref()
