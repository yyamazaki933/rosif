#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import QApplication, QFileDialog
from PyQt5.QtGui import QIcon, QTextCursor
from PyQt5 import uic, QtCore
import subprocess

import util.plotter as plotter
import util.ros2bag2csv as bag2csv

import re
import time


#################
# common
#################

def exec_cmd(cmd):
    print('CMD:', cmd)
    resp = subprocess.run(
        cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)
    print('OUT:', resp.stdout)
    print('ERR:', resp.stderr)
    print('---')

    return resp.stdout


def pb_update():
    global nodes
    global topics

    nodes.clear()
    topics.clear()

    # node list
    ui_debug.cb_node.clear()

    cmd = 'source ' + ros_bash + ' && ' + 'ros2 node list'
    stdout = exec_cmd(cmd)
    nodes = stdout.split('\n')

    for item in nodes:
        if item == '':
            continue
        ui_debug.cb_node.addItem(item)

    # topic list
    ui_debug.cb_topic.clear()
    ui_monitor.cb_topic.clear()

    cmd = 'source ' + ros_bash + ' && ' + 'ros2 topic list'
    stdout = exec_cmd(cmd)
    topics = stdout.split('\n')

    for item in topics:
        if item == '':
            continue
        ui_debug.cb_topic.addItem(item)
        ui_monitor.cb_topic.addItem(item)


#################
# main
#################

def tb_distro():
    global ros_distro
    global ros_bash

    ros_distro = ui_main.le_distro.text()
    ros_bash = '/opt/ros/' + ros_distro + '/setup.bash'


def pb_player():
    ui_player.show()


def pb_debugger():
    pb_update()
    ui_debug.show()


def pb_monitor():
    pb_update()
    ui_monitor.show()


#################
# debug
#################

def cb_node(node_name):
    ui_debug.lw_subtopic.clear()
    ui_debug.lw_pubtopic.clear()
    sub_topics = []
    pub_topics = []

    if node_name == '':
        return

    cmd = 'source ' + ros_bash + ' && ' + 'ros2 node info ' + node_name
    stdout = exec_cmd(cmd)
    lines = stdout.split('\n')

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
        ui_debug.lw_subtopic.addItem(item)

    for item in pub_topics:
        ui_debug.lw_pubtopic.addItem(item)


def cb_topic(topic_name):
    ui_debug.lw_pubnode.clear()
    ui_debug.lw_subnode.clear()
    pub_nodes = []
    sub_nodes = []

    if topic_name == '':
        return

    cmd = 'source ' + ros_bash + ' && ' + 'ros2 topic info -v ' + topic_name
    stdout = exec_cmd(cmd)
    lines = stdout.split('\n')

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
            ui_debug.le_type.setText(topic_type)

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
            node_name = namespace + name

            if pub_node_parse:
                pub_nodes.append(node_name)

            if sub_node_parse:
                sub_nodes.append(node_name)

    for item in pub_nodes:
        ui_debug.lw_pubnode.addItem(item)

    for item in sub_nodes:
        ui_debug.lw_subnode.addItem(item)


def open_monitor():
    topic_name = ui_debug.cb_topic.currentText()
    topic_type = ui_debug.le_type.text()

    ui_monitor.cb_topic.setCurrentText(topic_name)
    ui_monitor.le_type.setText(topic_type)

    ui_monitor.show()

    start_freq_monitor()


#################
# topic monitor
#################

class FreqMonitor(QtCore.QThread):

    currentFreqChanged = QtCore.pyqtSignal(int)

    def __init__(self, source, topic):
        super().__init__(None)
        
        self.__is_canceled = False
        self.freq = 0
        self.msg_source = source
        self.topic = topic

    def run(self):
        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'exec ros2 topic hz ' + self.topic + ' --window 100'

        self.cmd_proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, text=True)
        
        while not self.__is_canceled:
            line = self.cmd_proc.stdout.readline()
        
            if line == None:
                return
            
            if "average rate:" in line:
                self.freq = int(re.split('[:.]', line)[1])
                self.currentFreqChanged.emit(self.freq)
            
            time.sleep(0.1)
    
    def stop(self):
        self.cmd_proc.kill()
        
        while True:
            if self.cmd_proc.poll() != None:
                self.cmd_proc = None
                break
        self.__is_canceled = True

        self.quit()
        self.wait()

        print("[INFO] Called FreqMonitor.stop()")


def start_freq_monitor():
    global freq_monitor

    if freq_monitor != None:
        if freq_monitor.isRunning() or not freq_monitor.isFinished():
            freq_monitor.stop()
            freq_monitor = None

    ui_monitor.prog_hz.setRange(0, 1)
    ui_monitor.prog_hz.setValue(0)
    topic = ui_monitor.cb_topic.currentText()
    msg_source = ui_monitor.le_bash.text()

    if topic == '':
        return

    freq_monitor = FreqMonitor(msg_source, topic)
    freq_monitor.currentFreqChanged.connect(monitor_callback)

    freq_monitor.start()

    print("[INFO] start_freq_monitor()")


def monitor_callback(freq):
    global freq_monitor

    print("[INFO] Called monitor_callback() freq =", freq)

    if freq > ui_monitor.prog_hz.maximum():
        ui_monitor.prog_hz.setRange(0, freq)
    
    ui_monitor.prog_hz.setValue(freq)

    if not ui_monitor.isVisible():
        freq_monitor.stop()
        freq_monitor = None
        ui_monitor.prog_hz.setValue(0)


def echo_once():
    ui_monitor.pte_echo.clear()

    topic = ui_monitor.cb_topic.currentText()
    msg_source = ui_monitor.le_bash.text()

    if topic == '':
        return

    cmd = 'source ' + msg_source
    cmd += ' && '
    cmd += 'ros2 topic echo ' + topic + ' --once'
    stdout = exec_cmd(cmd)

    ui_monitor.pte_echo.setPlainText(stdout)

    row0 = ui_monitor.pte_echo.document().findBlockByLineNumber(0)
    ui_monitor.pte_echo.setTextCursor(QTextCursor(row0))

    print("[INFO] echo_once()")


#################
# player
#################

def tb_bag():
    bag = QFileDialog.getExistingDirectory(
        ui_player, 'Choose Rosbag2 Directory', HOME_DIR)

    if bag == '':
        return

    ui_player.le_bag.setText(bag)

    info, dur = bag_info(bag)

    ui_player.pte_bag.clear()
    ui_player.pte_bag.setPlainText(info)

    ui_player.progress.setRange(0, dur)
    ui_player.progress.reset()
    ui_player.progress.setValue(0)

    row0 = ui_player.pte_bag.document().findBlockByLineNumber(0)
    ui_player.pte_bag.setTextCursor(QTextCursor(row0))


def bag_info(bag):
    cmd = 'source ' + ros_bash + ' && ' + 'ros2 bag info ' + bag
    stdout = exec_cmd(cmd)

    lines = stdout.split('\n')
    info = ''
    dur = 0
    for line in lines:
        if line == '':
            continue
        info += line + '\n'

        if 'Duration:' in line:
            # Duration: 418.838s
            dur = int(re.split('[:.]', line)[1])

    return info, dur


def tb_bash():
    result = QFileDialog.getOpenFileName(
        None, 'Choose ROS Bash File', HOME_DIR, 'ROS Bash File (*.bash)')[0]

    if result == '':
        return

    ui_player.le_bash.setText(result)
    ui_monitor.le_bash.setText(result)


def pb_play():
    global player_proc
    global timer
    global timer_tick

    rosbag_dir = ui_player.le_bag.text()
    msg_source = ui_player.le_bash.text()
    rate = ui_player.sb_rate.value()
    start = ui_player.sb_offset.value()

    info, dur = bag_info(rosbag_dir)

    ui_player.pte_bag.clear()
    ui_player.pte_bag.setPlainText(info)

    ui_player.progress.setRange(0, dur)
    ui_player.progress.setValue(start)

    row0 = ui_player.pte_bag.document().findBlockByLineNumber(0)
    ui_player.pte_bag.setTextCursor(QTextCursor(row0))

    cmd = 'source ' + msg_source
    cmd += ' && '
    cmd += 'exec ros2 bag play ' + rosbag_dir
    cmd += ' --rate ' + str(rate)
    if start != 0.0:
        cmd += ' --start-offset ' + str(start)

    print(cmd)
    player_proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash')

    timer_tick = int(1000 / rate)
    timer.start(timer_tick)

    ui_player.pb_play.setEnabled(False)
    ui_player.sb_rate.setEnabled(False)
    ui_player.sb_offset.setEnabled(False)
    ui_player.pb_reset.setEnabled(True)
    ui_player.pb_pause.setEnabled(True)


def timer_callback():
    global timer

    value = ui_player.progress.value()
    max = ui_player.progress.maximum()

    if value < max:
        value = value + 1
        ui_player.progress.setValue(int(value))
    else:
        timer.stop()


def pb_pause():
    global timer
    global timer_tick

    cmd = 'source ' + ros_bash
    cmd += ' && '
    cmd += 'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'
    _ = exec_cmd(cmd)

    if timer.isActive():
        timer.stop()
    else:
        timer.start(timer_tick)


def pb_reset():
    global player_proc
    global timer

    player_proc.kill()
    timer.stop()

    while True:
        if player_proc.poll() != None:
            time.sleep(0.5)
            break
    player_proc = None

    ui_player.progress.setValue(0)
    ui_player.pb_play.setEnabled(True)
    ui_player.sb_rate.setEnabled(True)
    ui_player.sb_offset.setEnabled(True)
    ui_player.pb_reset.setEnabled(False)
    ui_player.pb_pause.setEnabled(False)


#################
# utility
#################

# def tb_wcsv():
#     bag = main_ui.le_bag.text()
#     topic = main_ui.cb_bagtopic.currentText()

#     bag_dir = os.path.dirname(bag)
#     result = bag2csv.writeCSV(bag_dir, topic)


# def tb_rcsv():
#     result = QFileDialog.getOpenFileName(
#         main_ui, 'Choose ROS CSV File', HOME_DIR, 'CSV File (*.csv)')[0]

#     if result == '':
#         return

#     main_ui.le_csv.clear()
#     main_ui.cb_xaxis.clear()
#     main_ui.cb_yaxis.clear()

#     main_ui.le_csv.setText(result)


# def pb_plot():
#     bag_dir = main_ui.le_csv.text()

#     x_axis = 'msg.pose.position.x'
#     y_axis = 'msg.pose.position.y'
#     # x_axis = main_ui.cb_xaxis.currentText()
#     # y_axis = main_ui.cb_yaxis.currentText()

#     result = plotter.plot(bag_dir, x_axis, y_axis)


# def pb_kml():
#     bag_dir = main_ui.le_csv.text()

#     long_axis = 'msg.longitude'
#     lat_axis = 'msg.latitude'

#     result = plotter.writeKML(bag_dir, long_axis, lat_axis)


if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    HOME_DIR = os.getenv('HOME')
    ros_distro = 'humble'
    ros_bash = '/opt/ros/' + ros_distro + '/setup.bash'

    # global var
    nodes = []
    topics = []
    player_proc = None
    freq_monitor = None
    timer_tick = 1000
    timer = QtCore.QTimer()
    timer.timeout.connect(timer_callback)

    app = QApplication(sys.argv)

    ui_main = uic.loadUi(SCRIPT_DIR + '/ui/main.ui')
    ui_main.setWindowIcon(QIcon(SCRIPT_DIR + '/img/ros.png'))
    ui_main.tb_distro.clicked.connect(tb_distro)
    ui_main.pb_player.clicked.connect(pb_player)
    ui_main.pb_debugger.clicked.connect(pb_debugger)
    ui_main.pb_monitor.clicked.connect(pb_monitor)
    # ui_main.pb_launcher.clicked.connect(pb_launcher)
    tb_distro()

    ui_player = uic.loadUi(SCRIPT_DIR + '/ui/player.ui')
    ui_player.tb_bag.clicked.connect(tb_bag)
    ui_player.tb_bash.clicked.connect(tb_bash)
    ui_player.pb_play.clicked.connect(pb_play)
    ui_player.pb_pause.clicked.connect(pb_pause)
    ui_player.pb_reset.clicked.connect(pb_reset)

    # ui_util.tb_bag.clicked.connect(tb_bag)
    # ui_util.tb_wcsv.clicked.connect(tb_wcsv)
    # ui_util.tb_rcsv.clicked.connect(tb_rcsv)
    # ui_util.tb_plot.clicked.connect(tb_plot)
    # ui_util.tb_kml.clicked.connect(tb_kml)

    ui_debug = uic.loadUi(SCRIPT_DIR + '/ui/debug.ui')
    ui_debug.pb_update.clicked.connect(pb_update)
    ui_debug.pb_update_2.clicked.connect(pb_update)
    ui_debug.cb_node.currentTextChanged.connect(cb_node)
    ui_debug.cb_topic.currentTextChanged.connect(cb_topic)
    ui_debug.pb_monitor.clicked.connect(open_monitor)

    ui_monitor = uic.loadUi(SCRIPT_DIR + '/ui/monitor.ui')
    ui_monitor.pb_update.clicked.connect(pb_update)
    ui_monitor.tb_bash.clicked.connect(tb_bash)
    ui_monitor.pb_monitor.clicked.connect(start_freq_monitor)

    ui_main.show()

    sys.exit(app.exec())
