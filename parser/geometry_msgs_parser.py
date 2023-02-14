#!/usr/bin/env python3
from parser.common_msgs_parser import parseHeader

POSE_STAMPED_TYPE = 'geometry_msgs/msg/PoseStamped'
POSE_COV_STAMPED_TYPE = 'geometry_msgs/msg/PoseWithCovarianceStamped'

def getHeaderRow(type):
    if type == POSE_STAMPED_TYPE:
        header = [
            'msg.header.stamp', 
            'msg.header.frame_id', 
            'msg.pose.position.x', 
            'msg.pose.position.y', 
            'msg.pose.position.z',
            'msg.pose.orientation.x',
            'msg.pose.orientation.y',
            'msg.pose.orientation.z',
            'msg.pose.orientation.w']
        return header

    if type == POSE_COV_STAMPED_TYPE:
        header = [
            'msg.header.stamp', 
            'msg.header.frame_id', 
            'msg.pose.pose.position.x', 
            'msg.pose.pose.position.y', 
            'msg.pose.pose.position.z',
            'msg.pose.pose.orientation.x',
            'msg.pose.pose.orientation.y',
            'msg.pose.pose.orientation.z',
            'msg.pose.pose.orientation.w']

        for i in range(36):
            header.append('msg.pose.covariance[' + str(i) + ']')
        return header

def parsePose(msg):
    pose = [
        msg.pose.position.x, 
        msg.pose.position.y, 
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ]
    return pose

def parse(type, msg):
    line = parseHeader(msg)

    if type == POSE_STAMPED_TYPE:
        return line.extend(parsePose(msg))

    elif type == POSE_COV_STAMPED_TYPE:
        line.extend(parsePose(msg.pose))
        for cov in msg.pose.covariance:
            line.append(cov)
        return line
        