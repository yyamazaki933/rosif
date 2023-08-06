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

    def pb_update_call(self):
        self.nodes = getNodeList()
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

    def pb_update_call(self):
        self.topics = getTopicList()
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
        topic_info_ui.setTopic(topic)
        topic_info_ui.show()
        self.child_windows.append(topic_info_ui)
