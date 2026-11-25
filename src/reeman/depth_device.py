"""Pass Gazebo depth frame bytes to the virtual OpenNI2 device."""

import socket
import struct


class OpenNIDepthInput:
    def __init__(self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)

    def write(self, message, data):
        if (message["encoding"], message["width"], message["height"], message["step"]) != (
            "16UC1", 320, 160, 320 * 2,
        ):
            return
        if len(data) != 320 * 160 * 2:
            return
        seconds, nanos = message["stamp"]
        try:
            self.socket.sendmsg(
                (struct.pack("<Q", seconds * 1_000_000 + nanos // 1000), data),
                [], 0, "/sim-state/openni-depth.sock",
            )
        except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
            pass
