#!/usr/bin/env python3

import subprocess

from PyQt5 import uic


class DebugWindow():

    def __init__(self, ui_file):
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.topic_monitor = None
        self.node_monitor = None

        self.ui = uic.loadUi(ui_file)
        self.ui.pb_update.clicked.connect(self.pb_update)
        self.ui.pb_update_2.clicked.connect(self.pb_update)
        self.ui.cb_node.currentTextChanged.connect(self.cb_node)
        self.ui.cb_topic.currentTextChanged.connect(self.cb_topic)
        self.ui.pb_topicmon.clicked.connect(self.open_topic_monitor)
        self.ui.pb_nodemon.clicked.connect(self.open_node_monitor)

        self.ui.lw_subtopic.itemDoubleClicked.connect(self.topic_selected)
        self.ui.lw_pubtopic.itemDoubleClicked.connect(self.topic_selected)
        self.ui.lw_subnode.itemDoubleClicked.connect(self.node_selected)
        self.ui.lw_pubnode.itemDoubleClicked.connect(self.node_selected)

    def show(self):
        self.pb_update()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

    def set_rospath(self, ros_path):
        self.ros_path = ros_path

    def set_monitor(self, topic_monitor, node_monitor):
        self.topic_monitor = topic_monitor
        self.node_monitor = node_monitor

    def open_topic_monitor(self):
        topic_name = self.ui.cb_topic.currentText()
        self.topic_monitor.open_monitor(topic_name)

    def open_node_monitor(self):
        node_name = self.ui.cb_node.currentText()
        self.node_monitor.open_monitor(node_name)

    def pb_update(self):
        # node list
        self.ui.cb_node.clear()

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 node list'
        print(cmd)

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        nodes = resp.stdout.split('\n')

        for item in nodes:
            if item == '':
                continue
            self.ui.cb_node.addItem(item)

        # topic list
        self.ui.cb_topic.clear()

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 topic list'
        print(cmd)

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        topics = resp.stdout.split('\n')

        for item in topics:
            if item == '':
                continue
            self.ui.cb_topic.addItem(item)

    def cb_node(self):
        node_name = self.ui.cb_node.currentText()
        if node_name == '':
            return

        self.ui.lw_subtopic.clear()
        self.ui.lw_pubtopic.clear()
        sub_topics = []
        pub_topics = []

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
                sub_topics.append(topic_name)

            if sub_node_parse:
                topic_name = line[4:].split(':')[0]
                pub_topics.append(topic_name)

        for item in sub_topics:
            self.ui.lw_subtopic.addItem(item)

        for item in pub_topics:
            self.ui.lw_pubtopic.addItem(item)

    def cb_topic(self):
        topic_name = self.ui.cb_topic.currentText()
        if topic_name == '':
            return

        self.ui.lw_pubnode.clear()
        self.ui.lw_subnode.clear()
        pub_nodes = []
        sub_nodes = []

        cmd = 'source /opt/ros/' + self.ros_distro + '/setup.bash'
        cmd += ' && '
        cmd += 'ros2 topic info -v ' + topic_name
        print(cmd)

        resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                              capture_output=True, text=True, timeout=3)
        lines = resp.stdout.split('\n')

        '''
        Type: sensor_msgs/msg/PointCloud2

        Publisher count: 1

        Node name: ring_outlier_filter
        Node namespace: /sensing/lidar/front_pandar_qt
        Topic type: sensor_msgs/msg/PointCloud2
        Endpoint type: PUBLISHER
        GID: 01.10.ed.12.0f.5d.a8.53.c7.2f.3b.6f.00.00.4e.03.00.00.00.00.00.00.00.00
        QoS profile:
        Reliability: BEST_EFFORT
        History (Depth): KEEP_LAST (5)
        Durability: VOLATILE
        Lifespan: Infinite
        Deadline: Infinite
        Liveliness: AUTOMATIC
        Liveliness lease duration: Infinite

        Subscription count: 1

        Node name: concatenate_data
        Node namespace: /sensing/lidar
        Topic type: sensor_msgs/msg/PointCloud2
        Endpoint type: SUBSCRIPTION
        GID: 01.10.3c.06.36.28.8b.66.8d.8d.c2.85.00.00.27.04.00.00.00.00.00.00.00.00
        QoS profile:
        Reliability: BEST_EFFORT
        History (Depth): KEEP_LAST (5)
        Durability: VOLATILE
        Lifespan: Infinite
        Deadline: Infinite
        Liveliness: AUTOMATIC
        Liveliness lease duration: Infinite
        '''

        pub_node_parse = False
        sub_node_parse = False
        topic_type = ''
        name = ''
        namespace = ''
        for line in lines:
            if line == '':
                continue

            if 'Type:' in line:
                topic_type = line[6:]
                self.ui.le_type.setText(topic_type)

            if 'Publisher count:' in line:
                sub_node_parse = False
                pub_node_parse = True

            if 'Subscription count:' in line:
                pub_node_parse = False
                sub_node_parse = True

            if 'Node name:' in line:
                name = line[11:]

            if 'Node namespace:' in line:
                namespace = line[16:]

                if namespace == '/':
                    node_name = '/' + name
                else:
                    node_name = namespace + '/' + name

                if pub_node_parse:
                    pub_nodes.append(node_name)

                if sub_node_parse:
                    sub_nodes.append(node_name)

        for item in pub_nodes:
            self.ui.lw_pubnode.addItem(item)

        for item in sub_nodes:
            self.ui.lw_subnode.addItem(item)

    def node_selected(self, item):
        node_name = item.text()
        print("[INFO] Selected List Item :", node_name)
        self.ui.cb_node.setCurrentText(node_name)
        self.ui.tab_debug.setCurrentIndex(1)

    def topic_selected(self, item):
        topic_name = item.text()
        print("[INFO] Selected List Item :", topic_name)
        self.ui.cb_topic.setCurrentText(topic_name)
        self.ui.tab_debug.setCurrentIndex(0)
