#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets

from util.common import *


class TopicInfoWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.topics = []
        # self.node_monitor = None

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)

        # self.lw_subnode.itemDoubleClicked.connect(self.node_selected)
        # self.lw_pubnode.itemDoubleClicked.connect(self.node_selected)

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro
    
    def set_rospath(self, ros_path):
        self.ros_path = ros_path
    
    # def set_monitor(self, node_monitor):
    #     self.node_monitor = node_monitor

    # def open_node_monitor(self):
    #     node_name = self.cb_node.currentText()
    #     self.node_monitor.open_monitor(node_name)

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

    def cb_topic_cb(self):
        topic_name = self.cb_topic.currentText()
        if topic_name == '':
            return

        self.lw_pubnode.clear()
        self.lw_subnode.clear()

        topic_type, pub_nodes, sub_nodes = getTopicInfo(
            self.ros_distro, topic_name)

        self.le_type.setText(topic_type)

        for item in pub_nodes:
            self.lw_pubnode.addItem(item)

        for item in sub_nodes:
            self.lw_subnode.addItem(item)

    # def node_selected(self, item):
    #     node_name = item.text()
    #     print("[INFO] Selected List Item :", node_name)
    #     self.cb_node.setCurrentText(node_name)
    #     self.tab_debug.setCurrentIndex(1)
