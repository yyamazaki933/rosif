#!/usr/bin/env python3

def timestamp_parser(msg):
    stamp = msg.header.stamp.sec + (float)(msg.header.stamp.nanosec / 1000000000)
    return stamp
