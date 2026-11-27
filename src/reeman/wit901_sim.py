"""Run the original WIT901 driver against a Gazebo-backed pseudo-terminal."""

import math
import os
import socket
from contextlib import ExitStack
import struct
import subprocess
import sys


def frame(kind, values, full_scale):
    counts = [max(-32768, min(32767, round(value * 32768 / full_scale)))
              for value in values]
    packet = struct.pack('<BBhhhh', 0x55, kind, *counts, 0)
    return packet + bytes((sum(packet) & 0xff,))


DIRECT_INPUT = "/sim-state/wit901-device.sock"
DIRECT_SAMPLE = struct.Struct("<10d")


def main(driver_command, direct=False):
    import pty
    import tty

    import rospy
    from sensor_msgs.msg import Imu
    from tf.transformations import euler_from_quaternion

    # The launch prefix receives __name and __log for the child, not the adapter.
    rospy.init_node('wit901_gazebo_device', argv=[sys.argv[0]])
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    # A private ROS parameter assignment overrides the stock physical port.
    command = [*driver_command, '_wit_port:=' + os.ttyname(slave)]
    def send_measurement(orientation, gyro, acceleration):
        angles = euler_from_quaternion(orientation)
        packet = (
            frame(0x51, acceleration, 16 * 9.80665)
            + frame(0x52, gyro, math.radians(2000))
            + frame(0x53, angles, math.pi)
        )
        try:
            os.write(master, packet)
        except BlockingIOError:
            rospy.logwarn_throttle(5, 'WIT901 serial buffer is full')

    def from_ros(message):
        q, w, a = message.orientation, message.angular_velocity, message.linear_acceleration
        send_measurement((q.x, q.y, q.z, q.w), (w.x, w.y, w.z), (a.x, a.y, a.z))

    try:
        with ExitStack() as devices:
            if direct:
                receiver = devices.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM))
                receiver.bind(DIRECT_INPUT)
                devices.callback(os.unlink, DIRECT_INPUT)
                receiver.settimeout(0.2)
            with subprocess.Popen(command) as driver:
                if not direct:
                    sub = rospy.Subscriber('/sim/imu_raw', Imu, from_ros, queue_size=1)
                    devices.callback(sub.unregister)
                try:
                    while not rospy.is_shutdown():
                        if direct:
                            if driver.poll() is not None:
                                return driver.returncode
                            try:
                                values = DIRECT_SAMPLE.unpack(receiver.recv(DIRECT_SAMPLE.size))
                            except socket.timeout:
                                continue
                            send_measurement(values[:4], values[4:7], values[7:])
                        else:
                            try:
                                return driver.wait(timeout=0.2)
                            except subprocess.TimeoutExpired:
                                pass
                finally:
                    if driver.poll() is None:
                        driver.terminate()
                        try:
                            driver.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            driver.kill()
    finally:
        os.close(master)
        os.close(slave)
    return 0


if __name__ == '__main__':
    args = sys.argv[1:]
    direct = args[:1] == ['--direct']
    sys.exit(main(args[1:] if direct else args, direct=direct))
