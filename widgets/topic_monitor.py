#!/usr/bin/env python3

import re
from PyQt5 import uic, QtWidgets, QtCore

from util.common import *


class FreqMonitor(QtCore.QThread):
    currentFreqChanged = QtCore.pyqtSignal(int)

    def __init__(self, path, topic):
        super().__init__(None)

        self.path = path
        self.topic = topic

    def run(self):
        cmd = 'source ' + self.path
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

    def __init__(self, path, topic):
        super().__init__(None)

        self.path = path
        self.topic = topic
    
    def run(self):
        cmd = 'source ' + self.path
        cmd += ' && '
        cmd += 'ros2 topic echo ' + self.topic
        proc = popenCmd(cmd)

        while True:
            line = proc.stdout.readline()
            if line:
                self.msgUpdated.emit(line)

    def stop(self):
        kill_proc('ros2 topic echo ' + self.topic)


class TopicMonitorWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/topic_monitor.ui", self)

        self.topics = []
        self.is_running = False
        self.freq_monitor = None
        self.echo_monitor = None
        self.msg = ''
        self.script_dir = script_dir

        self.pb_update.clicked.connect(self.pb_update_call)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_call)
        self.pb_path.clicked.connect(self.pb_path_call)
        self.pb_start.clicked.connect(self.pb_start_call)
        
        self.prog_hz.setRange(0, 100)
        self.prog_hz.setValue(0)

        with open(script_dir + "/path.conf", 'r') as f:
            for path in f.readlines():
                self.cb_path.addItem(path.strip())
        self.default_path = self.cb_path.itemText(0)

    def show(self):
        if not self.topics:
            self.pb_update_call()
        super().show()

    def pb_update_call(self):
        self.le_type.clear()
        topics = getTopicList(self.default_path)
        self.setTopicList(topics)

    def setTopicList(self, topics):
        self.topics = topics
        self.cb_topic.clear()
        self.cb_topic.addItem('')
        for topic in topics:
            self.cb_topic.addItem(topic)

    def cb_topic_call(self):
        topic_name = self.cb_topic.currentText()
        if topic_name == '': return
        topic_type, _, _ = getTopicInfo(self.default_path, topic_name)
        self.le_type.setText(topic_type)

    def setTopic(self, topic, type):
        self.cb_topic.clear()
        self.cb_topic.addItem(topic)
        self.cb_topic.setCurrentText(topic)
        self.le_type.setText(type)

    def pb_path_call(self):
        path = openPathFileDialog(self)
        if path != '':
            self.cb_path.addItem(path)
            self.cb_path.setCurrentText(path)
            with open(self.script_dir + "/path.conf", 'a') as f:
                f.write(path + "\n")

    def pb_start_call(self):
        if self.is_running:
            self.stop_monitor()
        else:
            self.start_monitor()

    def start_monitor(self):
        self.prog_hz.setRange(0, 100)
        self.prog_hz.setValue(0)
        self.pte_echo.clear()
        
        topic = self.cb_topic.currentText()
        path = self.cb_path.currentText()
        if topic == '': return

        self.freq_monitor = FreqMonitor(path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()
        self.echo_monitor = EchoMonitor(path, topic)
        self.echo_monitor.msgUpdated.connect(self.echo_callback)
        self.echo_monitor.start()
        self.is_running = True
        self.pb_start.setText('Stop Monitor')

    def stop_monitor(self):
        self.freq_monitor.stop()
        self.echo_monitor.stop()
        self.is_running = False
        self.pb_start.setText('Start Monitor')

    def freq_callback(self, freq):
        if freq > self.prog_hz.maximum():
            self.prog_hz.setRange(0, freq)
        self.prog_hz.setValue(freq)

    def echo_callback(self, line):
        self.msg += line
        if line == "---\n":
            self.pte_echo.clear()
            self.pte_echo.setPlainText(self.msg)
            self.msg = ''
