#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets, QtCore
from util.common import *
from widgets.topic_monitor import TopicMonitorWindow


class NodeInfoWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/node_info.ui", self)

        self.script_dir = script_dir
        self.nodes = []
        self.child_windows = []

        self.pb_update.clicked.connect(self.pb_update_call)
        self.cb_node.currentTextChanged.connect(self.cb_node_call)
        self.lw_pubs.itemDoubleClicked.connect(self.lw_item_call)
        self.lw_subs.itemDoubleClicked.connect(self.lw_item_call)

        with open(script_dir + "/path.conf", 'r') as f:
            self.default_path = f.readlines()[0].strip()

    def show(self):
        if not self.nodes:
            self.pb_update_call()
        super().show()
        
    def pb_update_call(self):
        nodes = getNodeList(self.default_path)
        self.setNodeList(nodes)
    
    def setNodeList(self, nodes):
        self.nodes = nodes
        self.cb_node.clear()
        self.cb_node.addItem('')
        for node in nodes:
            self.cb_node.addItem(node)

    def cb_node_call(self):
        node_name = self.cb_node.currentText()
        self.lw_pubs.clear()
        self.lw_subs.clear()
        if node_name == '': return
        
        sub_topics, pub_topics = getNodeInfo(self.default_path, node_name)
        for topic in pub_topics:
            self.lw_pubs.addItem(topic)
        for topic in sub_topics:
            self.lw_subs.addItem(topic)
    
    def lw_item_call(self, item:QtWidgets.QListWidgetItem):
        topic = item.text()
        topics = getTopicList(self.default_path)
        topic_info_ui = TopicInfoWindow(self.script_dir)
        topic_info_ui.setTopicList(topics)
        topic_info_ui.setTopic(topic)
        topic_info_ui.show()
        self.child_windows.append(topic_info_ui)

    def setNode(self, node):
        self.cb_node.setCurrentText(node)


class TopicInfoWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/topic_info.ui", self)

        self.script_dir = script_dir
        self.topics = []
        self.child_windows = []

        self.pb_update.clicked.connect(self.pb_update_call)
        self.cb_topic.currentTextChanged.connect(self.cb_topic_call)
        self.lw_pubs.itemDoubleClicked.connect(self.lw_item_call)
        self.lw_subs.itemDoubleClicked.connect(self.lw_item_call)
        self.pb_mon.clicked.connect(self.openMonitor)

        with open(script_dir + "/path.conf", 'r') as f:
            self.default_path = f.readlines()[0].strip()

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
        for topic in self.topics:
            self.cb_topic.addItem(topic)

    def cb_topic_call(self):
        topic_name = self.cb_topic.currentText()
        self.lw_pubs.clear()
        self.lw_subs.clear()
        if topic_name == '': return

        topic_type, pub_nodes, sub_nodes = getTopicInfo(self.default_path, topic_name)
        for node in pub_nodes:
            self.lw_pubs.addItem(node)
        for node in sub_nodes:
            self.lw_subs.addItem(node)
        self.le_type.setText(topic_type)

    def lw_item_call(self, item:QtWidgets.QListWidgetItem):
        node = item.text()
        nodes = getNodeList(self.default_path)
        node_info_ui = NodeInfoWindow(self.script_dir)
        node_info_ui.setNodeList(nodes)
        node_info_ui.setNode(node)
        node_info_ui.show()
        self.child_windows.append(node_info_ui)

    def setTopic(self, topic):
        self.cb_topic.setCurrentText(topic)

    def openMonitor(self):
        topic = self.cb_topic.currentText()
        type = self.le_type.text()
        monitor = TopicMonitorWindow(self.script_dir)
        monitor.setTopicList(self.topics)
        monitor.setTopic(topic, type)
        monitor.show()
        self.child_windows.append(monitor)
