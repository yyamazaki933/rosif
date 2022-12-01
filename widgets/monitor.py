#!/usr/bin/env python3

import os
import subprocess

from PyQt5 import uic
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtGui import QTextCursor

import monitor.monitor as monitor


class MonitorWindow():

    def __init__(self, ui_file):
        self.home_dir = os.getenv('HOME')
        self.freq_monitor = None
        self.echo_monitor = None
        self.ros_distro = 'humble'
        self.topics = []
        self.ns = []

        self.ui = uic.loadUi(ui_file)
        self.ui.pb_update.clicked.connect(self.pb_update)
        self.ui.tb_bash.clicked.connect(self.tb_bash)
        self.ui.pb_monitor.clicked.connect(self.start_freq_monitor)
        self.ui.cb_ns.currentTextChanged.connect(self.cb_ns)

    def show(self):
        self.pb_update()
        self.ui.show()

    def hide(self):
        self.ui.hide()

    def set_rosdistro(self, ros_distro):
        self.ros_distro = ros_distro

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

    def tb_bash(self):
        result = QFileDialog.getOpenFileName(
            self.ui, 'Choose ROS Bash File', self.home_dir, 'ROS Bash File (*.bash)')[0]
        if result != '':
            self.ui.le_bash.setText(result)

    def open_monitor(self, topic):
        self.show()

        self.ui.cb_topic.setCurrentText(topic)

        self.start_freq_monitor()

    def start_freq_monitor(self):
        if self.freq_monitor != None:
            if self.freq_monitor.isRunning() or not self.freq_monitor.isFinished():
                self.freq_monitor.stop()
                self.freq_monitor = None

        if self.echo_monitor != None:
            if self.echo_monitor.isRunning() or not self.echo_monitor.isFinished():
                self.echo_monitor.stop()
                self.echo_monitor = None

        self.ui.prog_hz.setRange(0, 1)
        self.ui.prog_hz.setValue(0)
        self.ui.pte_echo.clear()

        topic = self.ui.cb_topic.currentText()
        msg_source = self.ui.le_bash.text()

        if topic == '':
            return

        self.freq_monitor = monitor.FreqMonitor(msg_source, topic)
        self.freq_monitor.currentFreqChanged.connect(self.freq_callback)
        self.freq_monitor.start()

        self.echo_monitor = monitor.EchoMonitor(msg_source, topic)
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
