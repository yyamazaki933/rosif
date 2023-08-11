#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets, QtCore
from util.common import *
from widgets.info import NodeInfoWindow, TopicInfoWindow


class NodeListWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/node_list.ui", self)

        self.script_dir = script_dir
        self.nodes = []
        self.child_windows = []

        self.pb_update.clicked.connect(self.pb_update_call)
        self.le_filter.editingFinished.connect(self.updateNodeList)
        self.list.itemDoubleClicked.connect(self.lw_item_call)

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
        self.updateNodeList()

    def updateNodeList(self):
        keyword = self.le_filter.text()
        if keyword == '': keyword = '/'
        self.list.clear()
        for item in self.nodes:
            if keyword in item:
                self.list.addItem(item)
    
    def lw_item_call(self, item:QtWidgets.QListWidgetItem):
        node = item.text()
        node_info_ui = NodeInfoWindow(self.script_dir)
        node_info_ui.setNodeList(self.nodes)
        node_info_ui.setNode(node)
        node_info_ui.show()
        self.child_windows.append(node_info_ui)


class TopicListWindow(QtWidgets.QWidget):
    def __init__(self, script_dir, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(script_dir + "/ui/topic_list.ui", self)

        self.script_dir = script_dir
        self.topics = []
        self.child_windows = []

        self.pb_update.clicked.connect(self.pb_update_call)
        self.le_filter.editingFinished.connect(self.updateTopicList)
        self.list.itemDoubleClicked.connect(self.lw_item_call)

        with open(script_dir + "/path.conf", 'r') as f:
            self.default_path = f.readlines()[0].strip()

    def show(self):
        if not self.topics:
            self.pb_update_call()
        super().show()

    def pb_update_call(self):
        topics = getTopicList(self.default_path)
        self.setTopicList(topics)
    
    def setTopicList(self, topics):
        self.topics = topics
        self.updateTopicList()

    def updateTopicList(self):
        keyword = self.le_filter.text()
        if keyword == '': keyword = '/'
        self.list.clear()
        for item in self.topics:
            if keyword in item:
                self.list.addItem(item)
    
    def lw_item_call(self, item:QtWidgets.QListWidgetItem):
        topic = item.text()
        topic_info_ui = TopicInfoWindow(self.script_dir)
        topic_info_ui.setTopicList(self.topics)
        topic_info_ui.setTopic(topic)
        topic_info_ui.show()
        self.child_windows.append(topic_info_ui)
