"""Run the original WIT901 driver against a Gazebo-backed pseudo-terminal."""

import math
import os
import struct
import subprocess
import sys


def frame(kind, values, full_scale):
    counts = [max(-32768, min(32767, round(value * 32768 / full_scale)))
              for value in values]
    packet = struct.pack('<BBhhhh', 0x55, kind, *counts, 0)
    return packet + bytes((sum(packet) & 0xff,))


def main(driver_command):
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
    try:
        with subprocess.Popen(command) as driver:
            def send_measurement(message):
                acceleration = message.linear_acceleration
                gyro = message.angular_velocity
                attitude = message.orientation
                angles = euler_from_quaternion(
                    (attitude.x, attitude.y, attitude.z, attitude.w))
                packet = (
                    frame(0x51, (acceleration.x, acceleration.y, acceleration.z),
                          16 * 9.80665)
                    + frame(0x52, (gyro.x, gyro.y, gyro.z), math.radians(2000))
                    + frame(0x53, angles, math.pi)
                )
                try:
                    os.write(master, packet)
                except BlockingIOError:
                    rospy.logwarn_throttle(5, 'WIT901 serial buffer is full')

            subscription = rospy.Subscriber('/sim/imu_raw', Imu, send_measurement,
                                            queue_size=1)
            try:
                while not rospy.is_shutdown():
                    try:
                        return driver.wait(timeout=0.2)
                    except subprocess.TimeoutExpired:
                        continue
            finally:
                subscription.unregister()
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
    sys.exit(main(sys.argv[1:]))
