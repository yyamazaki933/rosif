#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QTextCursor

import monitor.monitor as monitor
from util.common import *


class TopicMonitorWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.is_running = False
        self.freq_monitor = None
        self.echo_monitor = None
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.topics = []

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.pb_monitor.clicked.connect(self.pb_monitor_cb)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def pb_update_cb(self):
        self.cb_topic.clear()
        self.cb_ns.clear()

        self.topics = getTopicList(self.ros_distro)
        ns = getNameSpaceList(self.topics)

        for item in self.topics:
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
        
        topic_type = getTopicType(
            self.ros_distro, topic_name)

        self.le_type.setText(topic_type)

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

        row0 = self.pte_echo.document().findBlockByLineNumber(0)
        self.pte_echo.setTextCursor(QTextCursor(row0))


class NodeMonitorWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, item_ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.nodes = []
        self.sub_topics = []
        self.pub_topics = []
        self.monitor_items = []
        self.item_ui_file = item_ui_file
        self.is_running = False

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.pb_monitor.clicked.connect(self.pb_monitor_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def pb_update_cb(self):
        self.cb_node.clear()
        self.cb_ns.clear()

        self.nodes = getNodeList(self.ros_distro)
        ns = getNameSpaceList(self.nodes)

        for item in self.nodes:
            self.cb_node.addItem(item)

        for item in ns:
            self.cb_ns.addItem(item)

    def cb_ns_cb(self, namespace):
        self.cb_node.clear()

        for item in self.nodes:
            if namespace in item:
                self.cb_node.addItem(item)

    def cb_node_cb(self):
        node_name = self.cb_node.currentText()
        if node_name == '':
            return

        self.sub_topics.clear()
        self.pub_topics.clear()

        self.sub_topics, self.pub_topics = getNodeInfo(
            self.ros_distro, node_name)

    def pb_monitor_cb(self):
        if self.is_running:
            self.stop_monitor()
            self.pb_monitor.setText('Start Monitor')
        else:
            self.start_monitor()
            self.pb_monitor.setText('Stop Monitor')

    def start_monitor(self):
        self.cb_node_cb()

        for topic in self.sub_topics:
            monitor = NodeMonitoritem(
                self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.bl_subs.addWidget(monitor)

        for topic in self.pub_topics:
            monitor = NodeMonitoritem(
                self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.bl_pubs.addWidget(monitor)

        self.is_running = True

    def stop_monitor(self):
        for monitor in self.monitor_items:
            monitor.stop()
            monitor.deleteLater()

        self.monitor_items.clear()
        self.is_running = False


class NodeMonitoritem(QtWidgets.QWidget):

    def __init__(self, ui_file, ros_path, topic, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.topic.setText(topic)

        self.freq_monitor = monitor.FreqMonitor(ros_path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

    def stop(self):
        self.freq_monitor.stop()
        self.freq_monitor = None
        self.freq.setValue(0)

    def freq_callback(self, freq):
        if freq > self.freq.maximum():
            self.freq.setRange(0, freq)

        self.freq.setValue(freq)
