#!/usr/bin/env python3
"""Publish Jetty measurements on the robot18 ROS Noetic interfaces."""

import argparse
from contextlib import ExitStack, suppress
from functools import partial
import json
import math
import socket
import struct
import threading
import time

import rospy
from driver_msgs.msg import (
    BatteryInfo, BumperEvent, CliffEvent, DockInfraRed, MotorPower, PowerSystemEvent,
    RangeSensorEvent, RobotStateEvent, SensorState,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField, Range
from std_msgs.msg import Empty, Header, Int32


FRAME = struct.Struct("!II")
VELOCITY = struct.Struct("!dd")
POSITIONS = ("left", "center", "right")
DEPTH_FRAME = "camera_depth_optical_frame"
COLOR_FRAME = "camera_color_optical_frame"
CLIFF_THRESHOLD = 0.22
BUMPER_RELEASE_DELAY = 0.12
ODOM_COVARIANCE = [0.0] * 36
ODOM_COVARIANCE[0] = ODOM_COVARIANCE[7] = 0.001
ODOM_COVARIANCE[14] = ODOM_COVARIANCE[21] = ODOM_COVARIANCE[28] = 1e6
ODOM_COVARIANCE[35] = 0.01


def header(message, frame_id=""):
    return Header(stamp=rospy.Time(*message["stamp"]), frame_id=frame_id)


def assign_vector(target, value):
    target.x, target.y, target.z = value


def assign_quaternion(target, value):
    target.x, target.y, target.z, target.w = value


def receive(stream, length):
    payload = stream.read(length)
    if len(payload) != length:
        raise EOFError("Gazebo sensor bridge disconnected")
    return payload


class Bridge:
    def __init__(self, connection):
        self.connection = connection
        self.command_lock = threading.Lock()
        self.enabled = True
        self.ranges = {}
        self.cliffs = {}
        self.bumper = set()
        self.last_contact = None
        self.encoder = {}

        self.publishers = {
            "clock": rospy.Publisher("/clock", Clock, queue_size=5),
            "teleop": rospy.Publisher("/input/key_cmd_vel", Twist, queue_size=1),
            "scan": rospy.Publisher("/scan", LaserScan, queue_size=3),
            "imu": rospy.Publisher("/imu", Imu, queue_size=10),
            "odom": rospy.Publisher("/mobile/odom", Odometry, queue_size=5),
            "joints": rospy.Publisher("/joint_states", JointState, queue_size=5),
            "lidar3d": rospy.Publisher("/lidar3d/points", PointCloud2, queue_size=2),
            "depth_points": rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=2),
            "camera_info": rospy.Publisher("/camera/depth/camera_info", CameraInfo, queue_size=2),
            "depth": rospy.Publisher("/camera/depth/image_raw", Image, queue_size=2),
            "color": rospy.Publisher("/camera/color/image_raw", Image, queue_size=2),
            "bumper": rospy.Publisher("/mobile_base/events/bumper", BumperEvent, queue_size=5),
            "range_event": rospy.Publisher("/mobile_base/events/range_sensor", RangeSensorEvent, queue_size=5),
            "cliff_event": rospy.Publisher("/mobile_base/events/cliff", CliffEvent, queue_size=5),
            "core": rospy.Publisher("/mobile_base/sensors/core", SensorState, queue_size=5),
            "online": rospy.Publisher("/mobile_base/events/online", RobotStateEvent, latch=True, queue_size=1),
            "battery": rospy.Publisher("/mobile_base/events/battery", PowerSystemEvent, latch=True, queue_size=1),
            "charger": rospy.Publisher("/mobile_base/events/charger", PowerSystemEvent, latch=True, queue_size=1),
            "battery_info": rospy.Publisher("/mobile_base/events/battery_info", BatteryInfo, latch=True, queue_size=1),
            "dock_ir": rospy.Publisher("/mobile_base/sensors/dock_ir", DockInfraRed, queue_size=2),
        }
        self.handlers = {
            "clock": self.clock,
            "teleop": self.teleop,
            "scan": self.scan,
            "imu": self.imu,
            "odom": self.odom,
            "joints": self.joints,
            "lidar3d": partial(self.cloud, "lidar3d_link"),
            "depth_points": partial(self.cloud, DEPTH_FRAME),
            "camera_info": self.camera_info,
            "depth": partial(self.image, DEPTH_FRAME),
            "color": partial(self.image, COLOR_FRAME),
            "bumper": self.contacts,
        }
        for sensor in ("ultrasonic", "cliff"):
            for position in POSITIONS:
                route = "{}/{}".format(sensor, position)
                self.publishers[route] = rospy.Publisher("/" + route, Range, queue_size=2)
                self.handlers[route] = partial(self.range, sensor, position)
        self.subscribers = [
            rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.velocity, queue_size=1),
            rospy.Subscriber("/mobile_base/commands/force_stop", Int32, self.force_stop, queue_size=1),
            rospy.Subscriber("/mobile_base/commands/motor_power", MotorPower, self.motor_power, queue_size=1),
            rospy.Subscriber("/enable", Empty, partial(self.set_enabled, True), queue_size=1),
            rospy.Subscriber("/disable", Empty, partial(self.set_enabled, False), queue_size=1),
        ]

    def advertise_online(self):
        self.publishers["online"].publish(RobotStateEvent(RobotStateEvent.ONLINE))
        # Fixed nominal readings for an undocked virtual base. There is no
        # battery discharge or charging dynamics model in this environment.
        self.publishers["battery"].publish(PowerSystemEvent(event=100))
        self.publishers["charger"].publish(PowerSystemEvent(event=PowerSystemEvent.UNPLUGGED))
        self.publishers["battery_info"].publish(BatteryInfo(
            manufacturer="Gazebo", nominal_voltage=16100, temperature=250,
            rated_capacity=5000, full_capacity=5000, capacity=5000, battery_health=100,
        ))

    def publish(self, message, payload):
        kind = message["kind"]
        result = self.handlers[kind](message, payload)
        if result is not None:
            self.publishers[kind].publish(result)

    def teleop(self, message, _):
        forward, yaw = message["forward"], message["yaw"]
        if not math.isfinite(forward) or not math.isfinite(yaw):
            return None
        command = Twist()
        if yaw:
            command.angular.z = max(-0.75, min(0.75, yaw))
        else:
            command.linear.x = max(-0.30, min(0.30, forward))
        return command

    def clock(self, message, _):
        stamp = rospy.Time(*message["stamp"])
        if self.last_contact is not None:
            if stamp < self.last_contact or (stamp - self.last_contact).to_sec() > BUMPER_RELEASE_DELAY:
                self.update_bumpers(set())
                self.last_contact = None
        return Clock(clock=stamp)

    def scan(self, message, _):
        angle_min, angle_max, angle_increment = message["angles"]
        range_min, range_max = message["limits"]
        return LaserScan(
            header=header(message, "laser_link"), angle_min=angle_min, angle_max=angle_max,
            angle_increment=angle_increment, scan_time=0.1, range_min=range_min, range_max=range_max,
            ranges=message["ranges"], intensities=message["intensities"],
        )

    def imu(self, message, _):
        result = Imu(header=header(message, "imu_link"))
        assign_quaternion(result.orientation, message["orientation"])
        assign_vector(result.angular_velocity, message["angular"])
        assign_vector(result.linear_acceleration, message["linear"])
        for name, values in zip(
            ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"),
            message["covariances"],
        ):
            if len(values) == 9:
                setattr(result, name, values)
        return result

    def odom(self, message, _):
        result = Odometry(header=header(message, "odom"), child_frame_id="base_footprint")
        assign_vector(result.pose.pose.position, message["position"])
        assign_quaternion(result.pose.pose.orientation, message["orientation"])
        assign_vector(result.twist.twist.linear, message["linear"])
        assign_vector(result.twist.twist.angular, message["angular"])
        result.pose.covariance = ODOM_COVARIANCE
        result.twist.covariance = ODOM_COVARIANCE
        self.publish_core(result.header.stamp)
        return result

    def joints(self, message, _):
        result = JointState(header=header(message))
        for name, position, velocity, effort in message["joints"]:
            if name not in ("wheel_left_joint", "wheel_right_joint"):
                continue
            result.name.append(name)
            result.position.append(position)
            result.velocity.append(velocity)
            result.effort.append(effort)
            self.encoder[name] = round(position * 4096 / math.tau) % 65536
        return result

    def cloud(self, frame_id, message, data):
        return PointCloud2(
            header=header(message, frame_id), height=message["height"], width=message["width"],
            fields=[PointField(*field) for field in message["fields"]],
            is_bigendian=message["bigendian"], point_step=message["point_step"],
            row_step=message["row_step"], data=data, is_dense=message["dense"],
        )

    def camera_info(self, message, _):
        result = CameraInfo(
            header=header(message, DEPTH_FRAME), width=message["width"], height=message["height"],
            distortion_model="plumb_bob", D=message["distortion"],
        )
        if len(message["intrinsics"]) == 9:
            result.K = message["intrinsics"]
        if len(message["projection"]) == 12:
            result.P = message["projection"]
        if len(message["rectification"]) == 9:
            result.R = message["rectification"]
        else:
            result.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return result

    def image(self, frame_id, message, data):
        return Image(
            header=header(message, frame_id), width=message["width"], height=message["height"],
            encoding=message["encoding"], step=message["step"], data=data, is_bigendian=0,
        )

    def range(self, sensor, position, message, _):
        value = message["value"]
        minimum, maximum = message["limits"]
        if sensor == "ultrasonic":
            radiation = Range.ULTRASOUND
            # The base only exposes uint8 values. Convert meters to cm with a
            # saturated no-return value, while the ROS Range retains +Inf.
            centimeters = 100 * value if math.isfinite(value) else value
            self.ranges[position] = round(max(0, min(255, centimeters)))
            if position == "center" and len(self.ranges) == len(POSITIONS):
                self.publishers["range_event"].publish(
                    RangeSensorEvent(*(self.ranges[name] for name in POSITIONS))
                )
        else:
            radiation = Range.INFRARED
            detected = value > CLIFF_THRESHOLD
            if detected != self.cliffs.get(position, False):
                self.publishers["cliff_event"].publish(
                    CliffEvent(POSITIONS.index(position), int(detected), 0)
                )
            self.cliffs[position] = detected
        return Range(
            header=header(message, "{}_{}_link".format(sensor, position)),
            radiation_type=radiation, field_of_view=message["fov"],
            min_range=minimum, max_range=maximum, range=value,
        )

    def contacts(self, message, _):
        active = set()
        for first, second, _positions in message["contacts"]:
            sides = set()
            for key, value in (
                ("bumper_left", BumperEvent.LEFT), ("bumper_center", BumperEvent.CENTER),
                ("bumper_right", BumperEvent.RIGHT), ("bumper_back", BumperEvent.BACK),
            ):
                if key in first or key in second:
                    sides.add(value)
            active.update(sides or {BumperEvent.CENTER})
        if active:
            self.last_contact = rospy.Time(*message["stamp"])
        self.update_bumpers(active)

    def update_bumpers(self, active):
        for bumper in active - self.bumper:
            self.publishers["bumper"].publish(BumperEvent(bumper, BumperEvent.PRESSED))
        for bumper in self.bumper - active:
            self.publishers["bumper"].publish(BumperEvent(bumper, BumperEvent.RELEASED))
        self.bumper = active

    def publish_core(self, stamp):
        # Emulate the sensor/encoder fields used by ROS consumers. Electrical
        # readings are nominal because Gazebo has no corresponding base ADC.
        bumper_mask = sum(
            mask for side, mask in (
                (BumperEvent.LEFT, SensorState.BUMPER_LEFT),
                (BumperEvent.CENTER, SensorState.BUMPER_CENTRE),
                (BumperEvent.RIGHT, SensorState.BUMPER_RIGHT),
            ) if side in self.bumper
        )
        cliff_mask = sum(1 << index for index, name in enumerate(reversed(POSITIONS)) if self.cliffs.get(name))
        self.publishers["core"].publish(SensorState(
            header=Header(stamp=stamp, frame_id="base_link"),
            time_stamp=(stamp.to_nsec() // 1_000_000) % 65536,
            bumper=bumper_mask, cliff=cliff_mask,
            left_encoder=self.encoder.get("wheel_left_joint", 0),
            right_encoder=self.encoder.get("wheel_right_joint", 0),
            battery=161, bottom=[0, 0, 0], current=[0, 0], analog_input=[0, 0, 0, 0],
        ))
        if self.publishers["dock_ir"].get_num_connections():
            self.publishers["dock_ir"].publish(DockInfraRed(
                header=Header(stamp=stamp, frame_id="dock_ir_link"), data=[0, 0, 0],
            ))

    def send_velocity(self, forward, yaw):
        try:
            self.connection.sendall(VELOCITY.pack(forward, yaw))
        except OSError:
            rospy.signal_shutdown("Gazebo sensor bridge disconnected")

    def velocity(self, twist):
        with self.command_lock:
            if self.enabled:
                self.send_velocity(twist.linear.x, twist.angular.z)

    def set_enabled(self, enabled, _):
        with self.command_lock:
            self.enabled = enabled
            if not enabled:
                self.send_velocity(0.0, 0.0)

    def motor_power(self, command):
        if command.state in (MotorPower.OFF, MotorPower.ON):
            self.set_enabled(command.state == MotorPower.ON, None)

    def force_stop(self, command):
        # The driver forwards stop commands to the base. Deceleration and
        # persistence inside the physical motor board are not simulated.
        if command.data in (1, 2):
            with self.command_lock:
                self.send_velocity(0.0, 0.0)

    def close(self):
        with suppress(OSError):
            self.connection.shutdown(socket.SHUT_RDWR)


def connect(endpoint):
    deadline = time.monotonic() + 30.0
    while True:
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.connect(endpoint)
            return connection
        except (FileNotFoundError, ConnectionRefusedError):
            connection.close()
            if time.monotonic() >= deadline or rospy.is_shutdown():
                raise
            time.sleep(0.1)


def read_measurements(stream, bridge):
    while not rospy.is_shutdown():
        metadata_size, payload_size = FRAME.unpack(receive(stream, FRAME.size))
        if not 0 < metadata_size <= 1024 * 1024 or payload_size > 32 * 1024 * 1024:
            raise ValueError("Invalid Gazebo sensor packet length")
        message = json.loads(receive(stream, metadata_size))
        bridge.publish(message, receive(stream, payload_size))


def read_bulk_measurements(stream, bridge):
    try:
        read_measurements(stream, bridge)
    except (EOFError, OSError, ValueError) as error:
        if not rospy.is_shutdown():
            rospy.logerr("Gazebo bulk sensor bridge: %s", error)
    finally:
        bridge.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("socket")
    parser.add_argument("--bulk-socket")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("reeman_gazebo_bridge")
    with ExitStack() as stack:
        connection = stack.enter_context(connect(args.socket))
        stream = stack.enter_context(connection.makefile("rb"))
        bridge = Bridge(connection)
        bulk_connection = stack.enter_context(connect(args.bulk_socket)) if args.bulk_socket else None

        def close():
            bridge.close()
            if bulk_connection is not None:
                with suppress(OSError):
                    bulk_connection.shutdown(socket.SHUT_RDWR)

        rospy.on_shutdown(close)
        rospy.loginfo("Gazebo sensor bridge connected")
        bridge.advertise_online()
        bulk_thread = None
        if bulk_connection is not None:
            bulk_stream = stack.enter_context(bulk_connection.makefile("rb"))
            bulk_thread = threading.Thread(
                target=read_bulk_measurements, args=(bulk_stream, bridge), daemon=True
            )
            bulk_thread.start()
        try:
            read_measurements(stream, bridge)
        except (EOFError, OSError) as error:
            if not rospy.is_shutdown():
                raise SystemExit(str(error))
        finally:
            close()
            if bulk_thread is not None:
                bulk_thread.join()
            if not rospy.is_shutdown():
                bridge.publishers["online"].publish(RobotStateEvent(RobotStateEvent.OFFLINE))


if __name__ == "__main__":
    main()
