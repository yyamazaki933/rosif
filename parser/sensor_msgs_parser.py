#!/usr/bin/env python3
from parser.common_msgs_parser import timestamp_parser

NAVSATFIX_TYPE = 'sensor_msgs/msg/NavSatFix'
NAVSATFIX_HEADER = [
    'msg.header.stamp', 
    'msg.header.frame_id', 
    'msg.status.status',
    'msg.status.service',
    'msg.latitude',
    'msg.longitude',
    'msg.altitude'
    ]

def navsatfix_parser(msg):
    line = [
        timestamp_parser(msg), 
        msg.header.frame_id, 
        msg.status.status,
        msg.status.service,
        msg.latitude,
        msg.longitude,
        msg.altitude
    ]
    return line