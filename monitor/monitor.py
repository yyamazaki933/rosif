#!/usr/bin/env python3

import re

from PyQt5 import QtCore
from util.common import *


class FreqMonitor(QtCore.QThread):
    currentFreqChanged = QtCore.pyqtSignal(int)

    def __init__(self, source, topic):
        super().__init__(None)

        self.msg_source = source
        self.topic = topic

    def run(self):
        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'ros2 topic hz ' + self.topic + ' --window 100'
        proc = popenCmd(cmd)

        while True:
            line = proc.stdout.readline()
            if "average rate:" in line:
                freq = int(re.split('[:.]', line)[1])
                self.currentFreqChanged.emit(freq)

    def stop(self):
        kill_proc('ros2 topic hz ' + self.topic)


class EchoMonitor(QtCore.QThread):
    msgUpdated = QtCore.pyqtSignal(str)

    def __init__(self, source, topic):
        super().__init__(None)

        self.msg_source = source
        self.topic = topic
    
    def run(self):
        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'ros2 topic echo ' + self.topic
        proc = popenCmd(cmd)

        while True:
            line = proc.stdout.readline()
            if line:
                self.msgUpdated.emit(line)

    def stop(self):
        kill_proc('ros2 topic echo ' + self.topic)
