#!/usr/bin/env python3

import subprocess

from PyQt5 import uic
from PyQt5.QtWidgets import QMessageBox


class ProcWindow():

    def __init__(self, ui_file):
        self.ui = uic.loadUi(ui_file)
        self.ui.pb_ref.clicked.connect(self.pb_ref)
        self.ui.pb_kill.clicked.connect(self.pb_kill)
        self.ui.lw_proc.itemDoubleClicked.connect(self.pb_kill)

    def show(self):
        self.pb_ref()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def pb_ref(self):
        self.ui.lw_proc.clear()

        cmd = 'ps -A -f | grep ros'
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

            if "grep ros" in proc:
                continue

            self.ui.lw_proc.addItem(pid + ' : ' + proc)

    def pb_kill(self):
        select = self.ui.lw_proc.selectedItems()

        if len(select) == 1:
            item_vec = select[0].text().split()
            pid = item_vec[0]
            proc = str.join(' ', item_vec[2:])

            message = "Kill the process ?\n"
            message += " PID : " + pid + '\n'
            message += " CMD : " + proc
            resp = QMessageBox.warning(
                self.ui, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)

            if resp == QMessageBox.Yes:
                self.kill_process(pid)
                self.pb_ref()

        else:
            message = "Kill the process ?"
            resp = QMessageBox.warning(
                self.ui, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)

            if resp == QMessageBox.Yes:
                for item in select:
                    item_vec = item.text().split()
                    pid = item_vec[0]
                    self.kill_process(pid)
                self.pb_ref()

    def kill_process(self, pid):
        cmd = 'kill -9 ' + pid
        print(cmd)

        subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
