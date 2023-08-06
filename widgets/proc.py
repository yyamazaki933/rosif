#!/usr/bin/env python3

import re

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QMessageBox

from util.common import runCmd

IGNORE_PROCS = [
    "rosif",
    "grep ros",
    "defunct",
    "bin/rqt",
    "player",
    "daemon",
]

class ProcWindow(QtWidgets.QWidget):

    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/proc.ui", self)

        self.pb_ref.clicked.connect(self.pb_ref_cb)
        self.pb_kill.clicked.connect(self.pb_kill_cb)
        self.lw_proc.itemDoubleClicked.connect(self.pb_kill_cb)

    def pb_ref_cb(self):
        self.lw_proc.clear()

        cmd = 'ps -A -f | grep ros'
        resp = runCmd(cmd)
        lines = resp.stdout.split('\n')

        for item in lines:
            if item == '':
                continue

            item_vec = item.split()
            pid = item_vec[1]
            pid = format(pid, '>10')
            proc = str.join(' ', item_vec[7:])
            
            ignore = False
            for s in IGNORE_PROCS:
                if s in proc:
                    ignore = True
                    print("[INFO] IGNORED (", s, "):", proc)
                    break
            if ignore:
                continue

            if "--ros-args" in proc:
                if "__node" in proc:
                    proc = re.search(r"__node:=[^ ]*", proc).group()
                    proc = proc.replace("__node:=", '')
                else:
                    proc_bin = proc.split(' ')[0]
                    proc = proc_bin.split('/')[-1]
            
            if "ros2 launch" in proc:
                proc = proc.replace("/usr/bin/python3 /opt/ros/humble/bin/", '')
                proc = re.sub(r"[^ ]*:=[^ ]*", '', proc)

            self.lw_proc.addItem(pid + ' : ' + proc)

    def pb_kill_cb(self):
        select = self.lw_proc.selectedItems()
        
        if len(select) == 0:
            message = "Are you wants to kill all ros proc?"
            resp = QMessageBox.warning(
                self, "Process Kill", message, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel)

            if resp == QMessageBox.Yes:
                for i in range(self.lw_proc.count()):
                    item_vec = self.lw_proc.item(i).text().split()
                    pid = item_vec[0]
                    self.kill_process(pid)
                self.pb_ref_cb()

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
        runCmd(cmd)