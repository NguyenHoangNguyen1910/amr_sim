#!/usr/bin/env python3
"""Print live ROS sensor readings from the running simulation."""
import math
import struct
import threading
import time

from driver_msgs.msg import BumperEvent, RangeSensorEvent
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud2, PointField, Range


DEPTH_ENCODINGS = {"16UC1": ("H", 2, 0.001), "32FC1": ("f", 4, 1.0)}


def meters(value):
    if math.isfinite(value):
        return f"{value:.2f} m"
    return "no echo" if value > 0 else "below minimum" if value < 0 else "invalid"


def scan(message):
    if not message.ranges:
        return "empty scan"
    center = message.ranges[len(message.ranges) // 2]
    return f"{len(message.ranges)} rays, front {meters(center)}"


def cloud(message):
    return f"{message.width} x {message.height}, {len(message.data):,} bytes"


def lidar_cloud(message):
    description = cloud(message)
    offsets = {field.name: field.offset for field in message.fields
               if field.datatype == PointField.FLOAT32}
    if not {"x", "y", "z"} <= offsets.keys() or not message.width or not message.height:
        return description
    unpack = struct.Struct(">f" if message.is_bigendian else "<f").unpack_from
    middle = message.width // 2
    distance = math.inf
    for row in range(message.height):
        for column in range(max(0, middle - 5), min(message.width, middle + 6)):
            offset = row * message.row_step + column * message.point_step
            x, y, z = (unpack(message.data, offset + offsets[axis])[0] for axis in "xyz")
            if x > 0 and all(map(math.isfinite, (x, y, z))):
                distance = min(distance, math.sqrt(x * x + y * y + z * z))
    return f"{description} | front {meters(distance)}"


def depth(message):
    if not message.width or not message.height or message.encoding not in DEPTH_ENCODINGS:
        return f"{message.width} x {message.height}, {message.encoding}"
    fmt, size, scale = DEPTH_ENCODINGS[message.encoding]
    offset = (message.height // 2) * message.step + (message.width // 2) * size
    if offset + size > len(message.data):
        return "incomplete depth image"
    order = ">" if message.is_bigendian else "<"
    value = struct.unpack_from(order + fmt, message.data, offset)[0] * scale
    center = meters(value) if value > 0 else "invalid depth"
    return f"{message.width} x {message.height}, center {center}"


def imu(message):
    az = message.linear_acceleration.z
    wz = message.angular_velocity.z
    return f"accel Z {az:+.2f} m/s², gyro Z {wz:+.2f} rad/s"


def odom(message):
    pose = message.pose.pose.position
    return f"x {pose.x:+.3f} m, y {pose.y:+.3f} m"


def main():
    rospy.init_node("reeman_sensor_monitor", anonymous=True)
    streams = [
        ("/scan", LaserScan, scan),
        ("/scan2", LaserScan, scan),
        ("/lidar3d/points", PointCloud2, lidar_cloud),
        ("/imu", Imu, imu),
        ("/camera/depth/image_raw", Image, depth),
        ("/camera/depth/points", PointCloud2, cloud),
        ("/camera/scan", LaserScan, scan),
        ("/mobile/odom", Odometry, odom),
    ]
    for sensor in ("ultrasonic", "cliff"):
        for side in ("left", "center", "right"):
            streams.append((f"/{sensor}/{side}", Range, lambda msg: meters(msg.range)))
    events = [
        ("/mobile_base/events/range_sensor", RangeSensorEvent,
         lambda msg: f"MCU bytes {msg.ultrasound1}/{msg.ultrasound2}/{msg.ultrasound3}"),
        ("/mobile_base/events/bumper", BumperEvent,
         lambda msg: f"{'PRESSED' if msg.state == BumperEvent.PRESSED else 'RELEASED'}, index {msg.bumper}"),
    ]
    messages = {}
    lock = threading.Lock()

    def receive(message, topic, render):
        value = render(message)
        header = getattr(message, "header", None)
        with lock:
            prior = messages.get(topic)
            stamp = header.stamp.to_sec() if header is not None else None
            rate = None
            if stamp is not None and prior and prior[3] is not None and stamp > prior[3]:
                rate = 1 / (stamp - prior[3])
                if prior[2] is not None:
                    rate = 0.25 * rate + 0.75 * prior[2]
            messages[topic] = (value, time.monotonic(), rate, stamp)

    continuous_topics = {topic for topic, _, _ in streams}
    subscriptions = [
        rospy.Subscriber(topic, msg_type,
                         lambda message, t=topic, r=render: receive(message, t, r),
                         queue_size=1, buff_size=2 ** 23)
        for topic, msg_type, render in streams + events
    ]
    while not rospy.is_shutdown():
        sim_time = rospy.Time.now().to_sec()
        with lock:
            snapshot = messages.copy()
        print(f"\nROS sensor readings | sim {sim_time:.2f} s", flush=True)
        for topic, _, _ in streams + events:
            item = snapshot.get(topic)
            if item:
                value, arrived, rate, _ = item
                status = f" | {rate:.1f} Hz sim" if rate else ""
                if topic in continuous_topics:
                    age = time.monotonic() - arrived
                    if age > 3:
                        status += f" | stale {age:.0f} s"
            else:
                value, status = ("no contact event yet" if topic.endswith("/bumper")
                                 else "waiting"), ""
            print(f"  {topic:<36} {value}{status}")
        time.sleep(2)


if __name__ == "__main__":
    main()