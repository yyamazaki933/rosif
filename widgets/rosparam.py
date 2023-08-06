#!/usr/bin/env python3

from PyQt5 import uic, QtWidgets
from util.common import *


class RosParamWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, paths, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.ros_path = ''
        self.nodes = []
        self.params = []

        self.pb_update.clicked.connect(self.pb_update_cb)
        self.cb_ns.currentTextChanged.connect(self.cb_ns_cb)
        self.cb_path.currentTextChanged.connect(self.cb_path_call)
        self.cb_node.currentTextChanged.connect(self.cb_node_call)
        self.lw_param.currentTextChanged.connect(self.lw_param_call)

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
        self.params = getParamList(self.ros_path, node_name)

        self.lw_param.clear()
        for param in self.params:
            self.lw_param.addItem(param)
    
    def lw_param_call(self, name):
        val = getParam(self.ros_path, name)
        self.le_val.setText(val)

    def pb_set_call(self):
        if self.is_running:
            self.stop_monitors()
            self.pb_monitor.setText('Start Monitor')
        else:
            self.start_monitors()
            self.pb_monitor.setText('Stop Monitor')
