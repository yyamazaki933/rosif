#!/usr/bin/env python3

import time
import subprocess

from PyQt5 import QtCore


class RosbagPlayer(QtCore.QThread):

    playerProglessTick = QtCore.pyqtSignal(int)
    playerFinished = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__(None)

        self.is_running = False
        self.is_stopped = False
        self.msg_source = ''
        self.rosbag_dir = ''
        self.rate = 1.0
        self.offset = 0
        self.cmd_proc = None
        self.elapsed = 0

    def setRosbag(self, rosbag_dir):
        self.rosbag_dir = rosbag_dir

    def setSource(self, source):
        self.msg_source = source

    def setRate(self, rate):
        self.rate = rate

    def setStartOffset(self, offset):
        self.offset = offset
        self.elapsed = offset

    def run(self):
        print("[INFO] Called RosbagPlayer.run()")

        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'exec ros2 bag play ' + self.rosbag_dir
        cmd += ' --rate ' + str(self.rate)
        if self.offset != 0:
            cmd += ' --start-offset ' + str(self.offset)

        self.cmd_proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash')
        self.is_running = True

        timer_tick = 1.0 / self.rate

        while not self.is_stopped:
            if self.is_running:

                if self.cmd_proc.poll() != None:
                    self.playerFinished.emit()
                    print("[INFO] RosbagPlayer Finished")
                    break

                self.playerProglessTick.emit(self.elapsed)
                self.elapsed += 1
                time.sleep(timer_tick)

    def pause(self):
        cmd = 'source ' + self.msg_source
        cmd += ' && '
        cmd += 'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'

        _ = subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)

        if self.is_running:
            self.is_running = False
        else:
            self.is_running = True

        print("[INFO] Called RosbagPlayer.pause()")

    def stop(self):
        self.cmd_proc.kill()
        self.is_running = False
        self.is_stopped = True

        self.quit()
        self.wait()

        time.sleep(1)

        print("[INFO] Called RosbagPlayer.stop()")
