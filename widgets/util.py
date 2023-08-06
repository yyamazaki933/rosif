#!/usr/bin/env python3

import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog

from util.plotter import *
from util.ros2bag2csv import *


class UtilWindow(QWidget):

    def __init__(self, ui_file, paths, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.tb_bag.clicked.connect(self.tb_bag_call)
        self.tb_csv.clicked.connect(self.tb_csv_call)

        self.home_dir = os.getenv('HOME')
        self.ros_path = paths[0]

    def show_widget(self):
        self.show()

    def tb_bag_call(self):
        bag = QFileDialog.getOpenFileName(
            self, 'Choose Rosbag2 File', self.home_dir, 'SQLite3 database File (*.db3)')[0]

        bagdir = os.path.dirname(bag)
        if bagdir == '':
            return

        self.le_bag.setText(bagdir)
        _, topics = getTopicList(bagdir)

        self.cb_topic.clear()
        for item in topics:
            self.cb_topic.addItem(item)

    def tb_csv_call(self):
        bag_dir = self.le_bag.text()
        topic = self.cb_topic.currentText()
        
        csv_name = writeCSV(bag_dir, topic)
        self.le_csv.setText(csv_name)
