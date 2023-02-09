#!/usr/bin/env python3

import subprocess

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QMessageBox


class ProcWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.pb_ref.clicked.connect(self.pb_ref_cb)
        self.pb_kill.clicked.connect(self.pb_kill_cb)
        self.lw_proc.itemDoubleClicked.connect(self.pb_kill_cb)
    
    def show_widget(self):
        self.pb_ref_cb()
        self.show()

    def pb_ref_cb(self):
        self.lw_proc.clear()

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

            if "rosif" in proc:
                continue
            
            if "defunct" in proc:
                continue

            self.lw_proc.addItem(pid + ' : ' + proc)

    def pb_kill_cb(self):
        select = self.lw_proc.selectedItems()
        
        if len(select) == 0:
            return

        elif len(select) == 1:
            item_vec = select[0].text().split()
            pid = item_vec[0]
            proc = str.join(' ', item_vec[2:])

            message = "Kill the process ?\n"
            message += " PID : " + pid + '\n'
            message += " CMD : " + proc
            resp = QMessageBox.warning(
                self, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)

            if resp == QMessageBox.Yes:
                self.kill_process(pid)
                self.pb_ref_cb()

        else:
            message = "Kill the process ?"
            resp = QMessageBox.warning(
                self, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)

            if resp == QMessageBox.Yes:
                for item in select:
                    item_vec = item.text().split()
                    pid = item_vec[0]
                    self.kill_process(pid)
                self.pb_ref_cb()

    def kill_process(self, pid):
        cmd = 'kill -9 ' + pid
        print(cmd)

        subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
