#!/usr/bin/env python3

import parser.geometry_msgs_parser as geometry_msgs
import parser.sensor_msgs_parser as sensor_msgs
import sys

import pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


SUPORT_TYPE = [
    sensor_msgs.NAVSATFIX_TYPE,
    geometry_msgs.POSE_STAMPED_TYPE
]


def getTopicList(bag_dir):
    topics = []
    supported_topics = []

    with Reader(bag_dir) as reader:

        for connection in reader.connections:

            topics.append(connection.topic)

            if connection.msgtype in SUPORT_TYPE:
                supported_topics.append(connection.topic)
                print(connection.topic, ':', connection.msgtype)
            else:
                print(connection.topic, ':', connection.msgtype, "*")

        print("* is unsupported type.")

    return topics, supported_topics


def writeCSV(bag_dir, topic):
    with Reader(bag_dir) as reader:
        data = []
        length = 0

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)

                if connection.msgtype == sensor_msgs.NAVSATFIX_TYPE:
                    if length == 0:
                        data.append(sensor_msgs.NAVSATFIX_HEADER)
                    data.append(sensor_msgs.navsatfix_parser(msg))
                    length += 1

                if connection.msgtype == geometry_msgs.POSE_STAMPED_TYPE:
                    if length == 0:
                        data.append(geometry_msgs.POSE_STAMPED_HEADER)
                    data.append(geometry_msgs.pose_stamped_parser(msg))
                    length += 1

                print("length:", length, end='\r')

        csv_name = bag_dir + '/' + topic.replace('/', '-') + '.csv'
        csv = pd.DataFrame(data)
        csv.to_csv(csv_name, header=False, index=False)

        print("Save:" + csv_name)
        return csv_name


if __name__ == '__main__':

    bag_dir = sys.argv[1]

    if len(sys.argv) == 2:
        getTopicList(bag_dir)

    elif len(sys.argv) == 3:
        topic = sys.argv[2]
        writeCSV(bag_dir, topic)
