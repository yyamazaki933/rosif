#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QTextCursor

import monitor.monitor as monitor
from util.common import *


class TopicMonitorWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, paths, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.is_running = False
        self.freq_monitor = None
        self.echo_monitor = None
        self.ros_path = ''
        self.topics = []

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.pb_monitor.clicked.connect(self.pb_monitor_cb)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)
        self.cb_path.currentTextChanged.connect(self.cb_path_call)
        
        self.prog_hz.setRange(0, 100)
        self.prog_hz.setValue(0)
        self.pte_echo.clear()

        for path in paths:
            self.cb_path.addItem(path)
        self.ros_path = paths[0]

    def show_widget(self):
        self.pb_update_cb()
        self.show()
        
    def cb_path_call(self, text):
        self.ros_path = text

    def pb_update_cb(self):
        self.cb_topic.clear()
        self.cb_ns.clear()

        self.topics = getTopicList(self.ros_path)
        ns = getNameSpaceList(self.topics)

        for item in self.topics:
            self.cb_topic.addItem('')
            self.cb_topic.addItem(item)

        for item in ns:
            self.cb_ns.addItem(item)

    def cb_ns_cb(self, namespace):
        self.cb_topic.clear()

        for item in self.topics:
            if namespace in item:
                self.cb_topic.addItem(item)

    def cb_topic_cb(self, topic_name):
        if topic_name == '':
            return

        topic_type, pub_nodes, sub_nodes = getTopicInfo(self.ros_path, topic_name)
        
        self.le_type.setText(topic_type)
        self.lw_pub.clear()
        self.lw_sub.clear()
        for node in pub_nodes:
            self.lw_pub.addItem(node)
        for node in sub_nodes:
            self.lw_sub.addItem(node)

    def pb_monitor_cb(self):
        if self.is_running:
            self.stop_monitor()
            self.pb_monitor.setText('Start Monitor')
        else:
            topic = self.cb_topic.currentText()
            if topic != '':
                self.start_monitor(topic)
                self.pb_monitor.setText('Stop Monitor')

    def start_monitor(self, topic):
        self.freq_monitor = monitor.FreqMonitor(self.ros_path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

        self.echo_monitor = monitor.EchoMonitor(self.ros_path, topic)
        self.echo_monitor.msgUpdated.connect(self.echo_callback)
        self.echo_monitor.start()

        self.is_running = True

    def stop_monitor(self):
        self.freq_monitor.stop()
        self.echo_monitor.stop()

        self.prog_hz.setRange(0, 100)
        self.prog_hz.setValue(0)
        self.pte_echo.clear()

        self.is_running = False

    def freq_callback(self, freq):
        print("[INFO] Called freq_callback() freq =", freq)

        if freq > self.prog_hz.maximum():
            self.prog_hz.setRange(0, freq)

        self.prog_hz.setValue(freq)

    def echo_callback(self, msg):
        print("[INFO] Called echo_callback()")

        self.pte_echo.clear()
        self.pte_echo.setPlainText(msg)
