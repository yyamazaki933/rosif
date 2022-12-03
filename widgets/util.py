#!/usr/bin/env python3

import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QFileDialog

from util.plotter import *
from util.ros2bag2csv import *


class UtilWindow(QWidget):

    def __init__(self, ui_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

    def tb_wcsv(self):
        bag_dir = self.le_bag.text()
        topic = self.cb_bagtopic.currentText()

        _ = writeCSV(bag_dir, topic)

    def tb_rcsv(self):
        home = os.getenv('HOME')
        result = QFileDialog.getOpenFileName(
            self, 'Choose ROS CSV File', home, 'CSV File (*.csv)')[0]

        if result == '':
            return

        self.le_csv.clear()
        self.cb_xaxis.clear()
        self.cb_yaxis.clear()

        self.le_csv.setText(result)

    def pb_plot(self):
        bag_dir = self.le_csv.text()

        x_axis = 'msg.pose.position.x'
        y_axis = 'msg.pose.position.y'
        # x_axis = main_ui.cb_xaxis.currentText()
        # y_axis = main_ui.cb_yaxis.currentText()

        _ = plot(bag_dir, x_axis, y_axis)

    def pb_kml(self):
        bag_dir = self.le_csv.text()

        long_axis = 'msg.longitude'
        lat_axis = 'msg.latitude'

        _ = writeKML(bag_dir, long_axis, lat_axis)
