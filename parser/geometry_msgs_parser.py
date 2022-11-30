#!/usr/bin/env python3
from parser.common_msgs_parser import timestamp_parser

POSE_STAMPED_TYPE = 'geometry_msgs/msg/PoseStamped'
POSE_STAMPED_HEADER = [
    'msg.header.stamp', 
    'msg.header.frame_id', 
    'msg.pose.position.x', 
    'msg.pose.position.y', 
    'msg.pose.position.z',
    'msg.pose.orientation.x',
    'msg.pose.orientation.y',
    'msg.pose.orientation.z',
    'msg.pose.orientation.w']

def pose_stamped_parser(msg):
    line = [
        timestamp_parser(msg), 
        msg.header.frame_id, 
        msg.pose.position.x, 
        msg.pose.position.y, 
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ]
    return line