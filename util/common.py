#!/usr/bin/env python3

import os
import subprocess
from PyQt5.QtWidgets import QFileDialog


def runCmd(cmd:str, timeout=None):
    print("[INFO] runCmd()", cmd)
    return subprocess.run(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)

def popenCmd(cmd:str):
    print("[INFO] popenCmd()", cmd)
    return subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def getNodeList(path):
    node_l = []

    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 node list'

    resp = runCmd(cmd)
    nodes = resp.stdout.split('\n')

    for item in nodes:
        if item == '':
            continue
        node_l.append(item)
    return node_l


def getTopicList(path):
    topic_l = []

    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 topic list'

    resp = runCmd(cmd)
    topics = resp.stdout.split('\n')

    for item in topics:
        if item == '' or item == "/clock" or item == "/parameter_events" or item == "/rosout":
            continue
        topic_l.append(item)
    return topic_l


def getNodeInfo(path, node_name):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 node info ' + node_name

    resp = runCmd(cmd)
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

    sub_topics = []
    pub_topics = []

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
            if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                continue
            sub_topics.append(topic_name)

        if sub_node_parse:
            topic_name = line[4:].split(':')[0]
            if topic_name == "/clock" or topic_name == "/parameter_events" or topic_name == "/rosout":
                continue
            pub_topics.append(topic_name)

    return sub_topics, pub_topics


def getTopicInfo(path, topic_name):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 topic info -v ' + topic_name

    resp = runCmd(cmd)
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

    pub_nodes = []
    sub_nodes = []
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

    return topic_type, pub_nodes, sub_nodes


def getTopicType(path, topic_name):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 topic type ' + topic_name

    resp = runCmd(cmd)
    return resp.stdout.strip('\n')


def getParamList(path, node_name):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 param list ' + node_name

    print('[INFO]', cmd)
    resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                          capture_output=True, text=True, timeout=3)
    list = resp.stdout.split('\n')
    return [i.strip() for i in list]


def getParam(path, param):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 param get ' + param

    print('[INFO]', cmd)
    resp = subprocess.run(cmd, shell=True, executable='/bin/bash',
                          capture_output=True, text=True, timeout=3)
    val = resp.stdout.split(':')[1].strip()
    return val


def kill_proc(keyword):
    cmd = 'ps -A -f | grep ros'

    resp = runCmd(cmd)
    lines = resp.stdout.split('\n')

    for item in lines:
        if item == '':
            continue

        item_vec = item.split()
        pid = item_vec[1]
        pid = format(pid, '>10')
        proc = str.join(' ', item_vec[7:])

        if keyword in proc:
            cmd = 'kill -9 ' + pid
            runCmd(cmd)


def openPathFileDialog(parent):
    home_dir = os.getenv('HOME')
    return QFileDialog.getOpenFileName(parent, 'Open setup.bash file', home_dir, 'Bash File (*.bash)')[0]
