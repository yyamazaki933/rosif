#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets

import monitor.monitor as monitor
from util.common import *


class TopicMonitorWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/topic_monitor.ui", self)

        self.is_running = False
        self.freq_monitor = None
        self.echo_monitor = None
        self.msg = ''
        self.script_dir = script_dir

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_call)
        self.pb_path.clicked.connect(self.pb_path_cb)
        self.pb_start.clicked.connect(self.pb_start_cb)
        
        self.prog_hz.setRange(0, 100)
        self.prog_hz.setValue(0)
        self.pte_echo.clear()
        
        with open(script_dir + "/path.conf", 'r') as f:
            for path in f.readlines():
                self.cb_path.addItem(path.strip())
    
    def pb_update_cb(self):
        topics = getTopicList()
        self.cb_topic.clear()
        self.cb_topic.addItem('')
        for topic in topics:
            self.cb_topic.addItem(topic)

    def cb_topic_call(self):
        topic_name = self.cb_topic.currentText()
        if topic_name == '': return
        topic_type, _, _ = getTopicInfo(topic_name)
        self.le_type.setText(topic_type)

    def setTopic(self, topic, type):
        self.cb_topic.clear()
        self.cb_topic.addItem(topic)
        self.cb_topic.setCurrentText(topic)
        self.le_type.setText(type)

    def pb_path_cb(self):
        path = openPathFileDialog(self)
        if path != '':
            self.cb_path.addItem(path)
            self.cb_path.setCurrentText(path)
            with open(self.script_dir + "/path.conf", 'a') as f:
                f.write(path + "\n")

    def pb_start_cb(self):
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

        self.freq_monitor = monitor.FreqMonitor(path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()
        self.echo_monitor = monitor.EchoMonitor(path, topic)
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
