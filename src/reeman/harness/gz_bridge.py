#!/usr/bin/env python3
"""Move sensor measurements from Jetty to the Noetic process over a Unix socket."""

import argparse
from contextlib import ExitStack
from functools import partial
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import threading
import time

from gz.msgs.camera_info_pb2 import CameraInfo
from gz.msgs.clock_pb2 import Clock
from gz.msgs.contacts_pb2 import Contacts
from gz.msgs.image_pb2 import Image, R_FLOAT32
from gz.msgs.imu_pb2 import IMU
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.model_pb2 import Model
from gz.msgs.odometry_pb2 import Odometry
from gz.msgs.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs.twist_pb2 import Twist
from gz.transport import Node
import numpy as np


FRAME = struct.Struct("!II")
VELOCITY = struct.Struct("!dd")
COMMAND_TIMEOUT = 0.6
POSITIONS = ("left", "center", "right")
BULK_STREAMS = frozenset(("lidar3d", "depth", "depth_points", "camera_info", "color"))


def xyz(vector):
    return (vector.x, vector.y, vector.z)


def xyzw(quaternion):
    return (*xyz(quaternion), quaternion.w)


def scan(msg):
    return {
        "angles": (msg.angle_min, msg.angle_max, msg.angle_step),
        "limits": (msg.range_min, msg.range_max),
        "ranges": list(msg.ranges),
        "intensities": list(msg.intensities),
    }, b""


def range_measurement(msg):
    # Use only rays inside the inscribed cone of the horizontal/vertical grid.
    # The intended angular widths are small and both axes point forward.
    half_cone = min(abs(msg.angle_max), abs(msg.vertical_angle_max))
    min_range = math.inf
    cone_limit = math.cos(half_cone) - 1e-9
    for index, value in enumerate(msg.ranges):
        horizontal = msg.angle_min + (index % msg.count) * msg.angle_step
        vertical = msg.vertical_angle_min + (index // msg.count) * msg.vertical_angle_step
        in_cone = not half_cone or math.cos(horizontal) * math.cos(vertical) >= cone_limit
        if in_cone and (value == -math.inf or msg.range_min <= value <= msg.range_max):
            min_range = min(min_range, value)
    return {
        "value": min_range,
        "limits": (msg.range_min, msg.range_max),
        "fov": 2 * half_cone or msg.angle_max - msg.angle_min,
    }, b""


def imu(msg):
    return {
        "orientation": xyzw(msg.orientation),
        "angular": xyz(msg.angular_velocity),
        "linear": xyz(msg.linear_acceleration),
        "covariances": [
            list(msg.orientation_covariance.data),
            list(msg.angular_velocity_covariance.data),
            list(msg.linear_acceleration_covariance.data),
        ],
    }, b""


def odom(msg):
    return {
        "position": xyz(msg.pose.position),
        "orientation": xyzw(msg.pose.orientation),
        "linear": xyz(msg.twist.linear),
        "angular": xyz(msg.twist.angular),
    }, b""


def joints(msg):
    return {
        "joints": [
            (joint.name, joint.axis1.position, joint.axis1.velocity, joint.axis1.force)
            for joint in msg.joint if joint.HasField("axis1")
        ],
    }, b""


def cloud(msg):
    return {
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "bigendian": msg.is_bigendian,
        "dense": msg.is_dense,
        # Gazebo's PointField enum starts at 0, ROS 1 starts at 1.
        "fields": [(field.name, field.offset, field.datatype + 1, field.count) for field in msg.field],
    }, msg.data


def optical_cloud(msg):
    metadata, data = cloud(msg)
    if not data:
        return metadata, data
    offsets = {field.name: field.offset for field in msg.field}
    points = bytearray(data)
    number = ">f4" if msg.is_bigendian else "<f4"
    x, y, z = (
        np.ndarray((msg.height, msg.width), dtype=number, buffer=points,
                   offset=offsets[axis], strides=(msg.row_step, msg.point_step))
        for axis in ("x", "y", "z")
    )
    forward = x.copy()
    np.negative(y, out=x)
    np.negative(z, out=y)
    z[:] = forward
    return metadata, points


def image(msg):
    if msg.pixel_format_type == R_FLOAT32:
        # OpenNI/DaBai publishes unsigned millimeters, with zero for invalid depth.
        values = np.frombuffer(msg.data, dtype="<f4").reshape(msg.height, msg.step // 4)[:, :msg.width]
        valid = np.isfinite(values) & (values > 0) & (values <= 65.535)
        millimeters = np.zeros(values.shape, dtype="<u2")
        millimeters[valid] = np.rint(values[valid] * 1000.0).astype("<u2")
        return {
            "width": msg.width, "height": msg.height,
            "encoding": "16UC1", "step": msg.width * 2,
        }, millimeters.tobytes()
    return {
        "width": msg.width, "height": msg.height,
        "encoding": "rgb8", "step": msg.step,
    }, msg.data


def camera_info(msg):
    return {
        "width": msg.width, "height": msg.height,
        "distortion": list(msg.distortion.k),
        "intrinsics": list(msg.intrinsics.k),
        "projection": list(msg.projection.p),
        "rectification": list(msg.rectification_matrix),
    }, b""


def contacts(msg):
    return {
        "contacts": [
            (contact.collision1.name, contact.collision2.name, [xyz(p) for p in contact.position])
            for contact in msg.contact
        ],
    }, b""


def clock(msg):
    return {"stamp": (msg.sim.sec, msg.sim.nsec)}, b""


def teleop(msg):
    return {"forward": msg.linear.x, "yaw": msg.angular.z}, b""


class Bridge:
    def __init__(self, separate_bulk=False, clock_hz=None):
        self.node = Node()
        self.condition = threading.Condition()
        self.pending = {}
        self.bulk_pending = {} if separate_bulk else None
        self.clock_period_ns = 1_000_000_000 // clock_hz if clock_hz else 0
        self.last_clock_ns = None
        self.connected = False
        self.velocity = self.node.advertise("/sim/cmd_vel", Twist)

        routes = [
            ("/clock", Clock, "clock", clock),
            ("/sim/teleop", Twist, "teleop", teleop),
            ("/sim/scan", LaserScan, "scan", scan),
            ("/sim/imu", IMU, "imu", imu),
            ("/sim/odom", Odometry, "odom", odom),
            ("/sim/joint_states", Model, "joints", joints),
            ("/sim/lidar3d/points", PointCloudPacked, "lidar3d", cloud),
            ("/sim/rgbd/points", PointCloudPacked, "depth_points", optical_cloud),
            ("/sim/rgbd/camera_info", CameraInfo, "camera_info", camera_info),
            ("/sim/rgbd/depth_image", Image, "depth", image),
            ("/sim/rgbd/image", Image, "color", image),
            ("/sim/bumper", Contacts, "bumper", contacts),
        ]
        for sensor in ("ultrasonic", "cliff"):
            for position in POSITIONS:
                route = f"{sensor}/{position}"
                routes.append((f"/sim/{route}", LaserScan, route, range_measurement))
        for topic, msg_type, kind, convert in routes:
            if not self.node.subscribe(msg_type, topic, partial(self.queue, kind, convert)):
                raise RuntimeError(f"Cannot subscribe to {topic}")

    def queue(self, kind, convert, msg):
        # Keep one pending sample per stream instead of accumulating delayed frames
        # if Gazebo briefly produces faster than the ROS receiver consumes.
        with self.condition:
            if self.connected:
                if kind == "clock" and self.clock_period_ns:
                    # Avoid waking every ROS clock subscriber for every physics step.
                    stamp_ns = msg.sim.sec * 1_000_000_000 + msg.sim.nsec
                    last = self.last_clock_ns
                    if last is not None and 0 <= stamp_ns - last < self.clock_period_ns:
                        return
                    self.last_clock_ns = stamp_ns
                pending = self.pending
                if self.bulk_pending is not None and kind in BULK_STREAMS:
                    pending = self.bulk_pending
                pending[kind] = (convert, msg)
                self.condition.notify_all()

    def publish(self, forward=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = forward
        msg.angular.z = yaw
        self.velocity.publish(msg)

    def receive_commands(self, connection, stop):
        command = bytearray()
        last_command = None
        try:
            while not stop.is_set():
                try:
                    if not select.select([connection], [], [], 0.1)[0]:
                        raise TimeoutError()
                    data = connection.recv(VELOCITY.size - len(command))
                    if not data:
                        break
                    command.extend(data)
                    if len(command) == VELOCITY.size:
                        forward, yaw = VELOCITY.unpack(command)
                        if math.isfinite(forward) and math.isfinite(yaw):
                            self.publish(forward, yaw)
                            last_command = time.monotonic()
                        command.clear()
                except TimeoutError:
                    pass
                if last_command is not None and time.monotonic() - last_command >= COMMAND_TIMEOUT:
                    self.publish()
                    last_command = None
        except OSError:
            pass
        finally:
            self.publish()
            stop.set()
            with self.condition:
                self.condition.notify_all()

    def send_measurements(self, connection, stop, pending):
        try:
            while not stop.is_set():
                with self.condition:
                    self.condition.wait_for(lambda: pending or stop.is_set())
                    if stop.is_set():
                        break
                    batch = tuple(pending.items())
                    pending.clear()
                for kind, (convert, msg) in batch:
                    header, data = convert(msg)
                    if "stamp" not in header:
                        stamp = msg.header.stamp
                        header["stamp"] = (stamp.sec, stamp.nsec)
                    header["kind"] = kind
                    metadata = json.dumps(header, separators=(",", ":")).encode()
                    connection.sendall(FRAME.pack(len(metadata), len(data)) + metadata)
                    if data:
                        connection.sendall(data)
        except OSError:
            pass
        finally:
            stop.set()
            with self.condition:
                self.condition.notify_all()

    def handle(self, connection, bulk_connection=None):
        connection.settimeout(5.0)
        stop = threading.Event()
        worker = threading.Thread(target=self.receive_commands, args=(connection, stop), daemon=True)
        bulk = None
        if bulk_connection is not None:
            bulk_connection.settimeout(5.0)
            bulk = threading.Thread(
                target=self.send_measurements, args=(bulk_connection, stop, self.bulk_pending), daemon=True
            )
        with self.condition:
            self.connected = True
        worker.start()
        if bulk is not None:
            bulk.start()
        try:
            self.send_measurements(connection, stop, self.pending)
        finally:
            worker.join()
            if bulk is not None:
                bulk.join()
            with self.condition:
                self.connected = False
                self.last_clock_ns = None
                self.pending.clear()
                if self.bulk_pending is not None:
                    self.bulk_pending.clear()


def listen(stack, path):
    server = stack.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))
    server.bind(str(path))
    stack.callback(path.unlink, missing_ok=True)
    os.chmod(path, 0o600)
    server.listen(1)
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("socket", type=Path)
    parser.add_argument("--bulk-socket", type=Path)
    parser.add_argument("--clock-hz", type=int)
    args = parser.parse_args()
    if args.clock_hz is not None and not 1 <= args.clock_hz <= 1_000_000_000:
        parser.error("--clock-hz must be a positive frequency in Hz")
    bridge = Bridge(separate_bulk=args.bulk_socket is not None, clock_hz=args.clock_hz)
    with ExitStack() as stack:
        server = listen(stack, args.socket)
        bulk_server = listen(stack, args.bulk_socket) if args.bulk_socket is not None else None
        if bulk_server is not None:
            bulk_server.settimeout(30.0)
        try:
            print(f"Gazebo sensor bridge listening on {args.socket}", flush=True)
            while True:
                with ExitStack() as connections:
                    connection = connections.enter_context(server.accept()[0])
                    try:
                        bulk_connection = connections.enter_context(bulk_server.accept()[0]) if bulk_server else None
                    except socket.timeout:
                        continue
                    print("ROS sensor bridge connected", flush=True)
                    bridge.handle(connection, bulk_connection)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
