#!/usr/bin/env python3

def timestamp_parser(msg):
    stamp = msg.header.stamp.sec + (float)(msg.header.stamp.nanosec / 1000000000)
    return stamp

def parseHeader(msg):
    header = [
        timestamp_parser(msg),
        msg.header.frame_id
    ]
    return header
