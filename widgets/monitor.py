#!/usr/bin/env python3

import subprocess

from PyQt5 import uic, QtWidgets
from PyQt5.QtGui import QTextCursor

import monitor.monitor as monitor


class TopicMonitorWindow():

    def __init__(self, ui_file):
        self.freq_monitor = None
        self.echo_monitor = None
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.topics = []
        self.ns = []

        self.ui = uic.loadUi(ui_file)
        self.ui.pb_update.clicked.connect(self.pb_update)
        self.ui.pb_monitor.clicked.connect(self.start_monitor)
        self.ui.cb_ns.currentTextChanged.connect(self.cb_ns)

    def show(self):
        self.pb_update()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def open_monitor(self, topic):
        self.show()
        self.ui.cb_topic.setCurrentText(topic)
        self.start_monitor()

    def update_topics(self):
        self.topics.clear()

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 topic list'

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        topics = resp.stdout.split('\n')

        for item in topics:
            if item == '':
                continue
            self.topics.append(item)

    def update_ns(self, src_list):
        self.ns.clear()
        self.ns.append('/')

        for item in src_list:
            ns_vec = item.split('/')
            ns = '/' + ns_vec[1]
            if len(ns_vec) > 3:
                if ns in self.ns:
                    continue
                self.ns.append(ns)

    def pb_update(self):
        self.ui.cb_topic.clear()
        self.ui.cb_ns.clear()

        self.update_topics()
        self.update_ns(self.topics)

        for item in self.topics:
            self.ui.cb_topic.addItem(item)

        for item in self.ns:
            self.ui.cb_ns.addItem(item)

    def cb_ns(self, namespace):
        self.ui.cb_topic.clear()

        for item in self.topics:
            if namespace in item:
                self.ui.cb_topic.addItem(item)

    def start_monitor(self):
        self.ui.prog_hz.setRange(0, 1)
        self.ui.prog_hz.setValue(0)
        self.ui.pte_echo.clear()

        if self.freq_monitor != None:
            if self.freq_monitor.isRunning() or not self.freq_monitor.isFinished():
                self.freq_monitor.stop()
                self.freq_monitor = None

        if self.echo_monitor != None:
            if self.echo_monitor.isRunning() or not self.echo_monitor.isFinished():
                self.echo_monitor.stop()
                self.echo_monitor = None

        topic = self.ui.cb_topic.currentText()

        if topic == '':
            return

        self.freq_monitor = monitor.FreqMonitor(self.ros_path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

        self.echo_monitor = monitor.EchoMonitor(self.ros_path, topic)
        self.echo_monitor.msgUpdated.connect(self.echo_callback)
        self.echo_monitor.start()

    def freq_callback(self, freq):
        print("[INFO] Called freq_callback() freq =", freq)

        if freq > self.ui.prog_hz.maximum():
            self.ui.prog_hz.setRange(0, freq)

        self.ui.prog_hz.setValue(freq)

        if not self.ui.isVisible():
            self.freq_monitor.stop()
            self.freq_monitor = None
            self.ui.prog_hz.setValue(0)

    def echo_callback(self, msg):
        print("[INFO] Called echo_callback()")

        self.ui.pte_echo.clear()
        self.ui.pte_echo.setPlainText(msg)

        row0 = self.ui.pte_echo.document().findBlockByLineNumber(0)
        self.ui.pte_echo.setTextCursor(QTextCursor(row0))

        if not self.ui.isVisible():
            self.echo_monitor.stop()
            self.echo_monitor = None


class NodeMonitorWindow():

    def __init__(self, ui_file, item_ui_file):
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.nodes = []
        self.ns = []
        self.sub_topics = []
        self.pub_topics = []
        self.is_running = False
        self.monitor_items = []
        self.item_ui_file = item_ui_file

        self.ui = uic.loadUi(ui_file)
        self.ui.pb_update.clicked.connect(self.pb_update)
        self.ui.pb_monitor.clicked.connect(self.pb_monitor)
        self.ui.cb_ns.currentTextChanged.connect(self.cb_ns)

    def show(self):
        self.stop_monitor()
        self.pb_update()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def open_monitor(self, node):
        self.show()
        self.ui.cb_node.setCurrentText(node)
        # self.start_monitor()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def update_nodes(self):
        self.nodes.clear()

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 node list'

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        nodes = resp.stdout.split('\n')

        for item in nodes:
            if item == '':
                continue
            self.nodes.append(item)

    def update_ns(self, src_list):
        self.ns.clear()
        self.ns.append('/')

        for item in src_list:
            ns_vec = item.split('/')
            ns = '/' + ns_vec[1]
            if len(ns_vec) > 3:
                if ns in self.ns:
                    continue
                self.ns.append(ns)

    def pb_update(self):
        self.ui.cb_node.clear()
        self.ui.cb_ns.clear()

        self.update_nodes()
        self.update_ns(self.nodes)

        for item in self.nodes:
            self.ui.cb_node.addItem(item)

        for item in self.ns:
            self.ui.cb_ns.addItem(item)

    def cb_ns(self, namespace):
        self.ui.cb_node.clear()

        for item in self.nodes:
            if namespace in item:
                self.ui.cb_node.addItem(item)

    def cb_node(self):
        node_name = self.ui.cb_node.currentText()
        if node_name == '':
            return

        self.sub_topics.clear()
        self.pub_topics.clear()

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 node info ' + node_name
        print(cmd)

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        lines = resp.stdout.split('\n')

        '''
        /sensing/lidar/front_pandar_qt/ring_outlier_filter
        Subscribers:
            /clock: rosgraph_msgs/msg/Clock
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /sensing/lidar/front_pandar_qt/rectified/pointcloud_ex: sensor_msgs/msg/PointCloud2
        Publishers:
            /parameter_events: rcl_interfaces/msg/ParameterEvent
            /rosout: rcl_interfaces/msg/Log
            /sensing/lidar/front_pandar_qt/outlier_filtered/pointcloud: sensor_msgs/msg/PointCloud2
        Service Servers:
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/describe_parameters: rcl_interfaces/srv/DescribeParameters
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/get_parameters: rcl_interfaces/srv/GetParameters
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/list_parameters: rcl_interfaces/srv/ListParameters
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/set_parameters: rcl_interfaces/srv/SetParameters
            /sensing/lidar/front_pandar_qt/ring_outlier_filter/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
        Service Clients:
        Action Servers:
        Action Clients:
        '''

        pub_node_parse = False
        sub_node_parse = False
        for line in lines:
            if line == '':
                continue

            if 'Subscribers:' in line:
                sub_node_parse = False
                pub_node_parse = True
                continue

            if 'Publishers:' in line:
                pub_node_parse = False
                sub_node_parse = True
                continue

            if 'Service Servers:' in line:
                pub_node_parse = False
                sub_node_parse = False
                continue

            if pub_node_parse:
                topic_name = line[4:].split(':')[0]
                if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                    continue
                self.sub_topics.append(topic_name)

            if sub_node_parse:
                topic_name = line[4:].split(':')[0]
                if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                    continue
                self.pub_topics.append(topic_name)

    def pb_monitor(self):
        if self.is_running:
            self.stop_monitor()
            self.ui.pb_monitor.setText('Start Monitor')
        else:
            self.start_monitor()
            self.ui.pb_monitor.setText('Stop Monitor')

    def start_monitor(self):
        self.cb_node()

        for topic in self.sub_topics:
            monitor = NodeMonitoritem(
                self.ui, self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.ui.bl_subs.addWidget(monitor)

        for topic in self.pub_topics:
            monitor = NodeMonitoritem(
                self.ui, self.item_ui_file, self.ros_path, topic)
            self.monitor_items.append(monitor)
            self.ui.bl_pubs.addWidget(monitor)

        self.is_running = True

    def stop_monitor(self):
        for monitor in self.monitor_items:
            monitor.stop()
            monitor.deleteLater()

        self.monitor_items.clear()
        self.is_running = False


class NodeMonitoritem(QtWidgets.QWidget):

    def __init__(self, parent_ui, ui_file, ros_path, topic, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent_ui = parent_ui

        self.ui = uic.loadUi(ui_file)
        self.ui.topic.setText(topic)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.ui)
        self.setLayout(layout)

        self.freq_monitor = monitor.FreqMonitor(ros_path, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

    def stop(self):
        self.freq_monitor.stop()
        self.freq_monitor = None
        self.ui.freq.setValue(0)

    def freq_callback(self, freq):
        if freq > self.ui.freq.maximum():
            self.ui.freq.setRange(0, freq)

        self.ui.freq.setValue(freq)
