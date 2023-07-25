#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QTextCursor

import monitor.monitor as monitor
from util.common import *


class NodeMonitorWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, item_ui_file, paths, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.ros_path = ''
        self.nodes = []
        self.sub_topics = []
        self.pub_topics = []
        self.monitor_items = []
        self.item_ui_file = item_ui_file
        self.is_running = False

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.pb_monitor.clicked.connect(self.pb_monitor_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)
        self.cb_node.currentTextChanged.connect(self.cb_node_call)
        self.cb_path.currentTextChanged.connect(self.cb_path_call)

        self.bl_pubs.addStretch()
        self.bl_subs.addStretch()

        for path in paths:
            self.cb_path.addItem(path)
        self.ros_path = paths[0]

    def show_widget(self):
        self.pb_update_cb()
        self.show()
    
    def cb_path_call(self, text):
        self.ros_path = text

    def pb_update_cb(self):
        self.cb_ns.clear()
        self.cb_node.clear()
        self.cb_node.addItem('')

        self.nodes = getNodeList(self.ros_path)
        ns = getNameSpaceList(self.nodes)

        for item in self.nodes:
            self.cb_node.addItem(item)

        for item in ns:
            self.cb_ns.addItem(item)

    def cb_ns_cb(self, namespace):
        self.cb_node.clear()
        self.cb_node.addItem('')

        for item in self.nodes:
            if namespace in item:
                self.cb_node.addItem(item)

    def cb_node_call(self, node_name):
        if node_name == '':
            return
        
        if len(self.monitor_items) != 0:
            for monitor in self.monitor_items:
                monitor.stop()
                monitor.deleteLater()
            self.monitor_items.clear()

        self.sub_topics.clear()
        self.pub_topics.clear()

        self.sub_topics, self.pub_topics = getNodeInfo(self.ros_path, node_name)

        for topic in self.sub_topics:
            monitor = NodeMonitoritem(
                self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.bl_subs.insertWidget(self.bl_subs.count()-1, monitor)

        for topic in self.pub_topics:
            monitor = NodeMonitoritem(
                self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.bl_pubs.insertWidget(self.bl_pubs.count()-1, monitor)

    def pb_monitor_cb(self):
        if self.is_running:
            self.stop_monitors()
            self.pb_monitor.setText('Start Monitor')
        else:
            self.start_monitors()
            self.pb_monitor.setText('Stop Monitor')

    def start_monitors(self):
        for monitor in self.monitor_items:
            monitor.start()
        self.is_running = True

    def stop_monitors(self):
        for monitor in self.monitor_items:
            monitor.stop()
        self.is_running = False


class NodeMonitoritem(QtWidgets.QWidget):

    def __init__(self, ui_file, ros_path, topic, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.ros_path = ros_path
        self.topic_name = topic
        self.topic.setText(topic)

    def start(self):
        self.freq_monitor = monitor.FreqMonitor(self.ros_path, self.topic_name)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

    def stop(self):
        self.freq_monitor.stop()
        self.freq.setValue(0)

    def freq_callback(self, freq):
        if freq > self.freq.maximum():
            self.freq.setRange(0, freq)

        self.freq.setValue(freq)
