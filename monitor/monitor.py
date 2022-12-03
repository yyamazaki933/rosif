#!/usr/bin/env python3

import time
import re
import subprocess

from PyQt5 import QtCore


class FreqMonitor(QtCore.QThread):

    currentFreqChanged = QtCore.pyqtSignal(int)

    def __init__(self, source, topic):
        super().__init__(None)

        self.__is_canceled = False
        self.freq = 0
        self.msg_source = source
        self.topic = topic

    def run(self):
        print("[INFO] Called FreqMonitor.run()")

        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'exec ros2 topic hz ' + self.topic + ' --window 100'
        print(cmd)

        self.cmd_proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, text=True)

        while not self.__is_canceled:
            line = self.cmd_proc.stdout.readline()

            if line == None:
                return

            if "average rate:" in line:
                self.freq = int(re.split('[:.]', line)[1])
                self.currentFreqChanged.emit(self.freq)

            time.sleep(0.5)

    def stop(self):
        self.cmd_proc.kill()

        while True:
            if self.cmd_proc.poll() != None:
                self.cmd_proc = None
                break
        self.__is_canceled = True

        self.quit()
        self.wait()

        print("[INFO] Called FreqMonitor.stop()")


class EchoMonitor(QtCore.QThread):

    msgUpdated = QtCore.pyqtSignal(str)

    def __init__(self, source, topic):
        super().__init__(None)

        self.__is_canceled = False
        self.msg_source = source
        self.topic = topic

    def run(self):
        print("[INFO] Called EchoMonitor.run()")

        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'ros2 topic echo ' + self.topic + ' --once'

        while not self.__is_canceled:
            try:
                resp = subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
                self.msgUpdated.emit(resp.stdout)
            except subprocess.TimeoutExpired:
                self.msgUpdated.emit('[WARN] topic echo : timeout!')

            time.sleep(1)

    def stop(self):
        self.__is_canceled = True

        self.quit()
        self.wait()

        print("[INFO] Called EchoMonitor.stop()")
