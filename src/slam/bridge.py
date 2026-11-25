"""Feed Gazebo scan and wheel odometry to a fresh Cartographer per session."""

import json
import math
import os
from pathlib import Path
import subprocess
from urllib.request import Request, urlopen

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


def set_stamp(stamp, values):
    stamp.sec, stamp.nanosec = values


def set_components(target, values):
    for field, value in values.items():
        setattr(target, field, value)


class MappingBridge(Node):
    def __init__(self):
        super().__init__("amr_mapping_bridge")
        self.base = os.environ["AMR_SLAM_API"].rstrip("/")
        self.headers = {"X-Slam-Key": os.environ["AMR_SLAM_KEY"], "Content-Type": "application/json"}
        self.session = None
        self.processes = []
        self.sequence = 0
        self.odom_stamp = None
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.transforms = StaticTransformBroadcaster(self)
        laser = TransformStamped()
        laser.header.frame_id = "base_footprint"
        laser.child_frame_id = "reeman_laser"
        laser.transform.translation.x = 0.22
        laser.transform.translation.z = 0.22
        laser.transform.rotation.w = 1.0
        self.transforms.sendTransform(laser)
        self.subscription = None
        self.create_timer(0.10, self.poll)
        self.previous_error = None

    def request(self, path, payload=None):
        body = None if payload is None else json.dumps(payload).encode()
        with urlopen(Request(
            self.base + path, data=body, headers=self.headers,
            method="GET" if body is None else "PUT",
        ), timeout=2) as response:
            return json.load(response)

    def close_session(self):
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.processes.clear()

    def switch(self, session):
        self.close_session()
        self.session = session
        self.sequence = 0
        self.odom_stamp = None
        if not session:
            return
        directory = Path(get_package_prefix("cartographer_ros")) / "lib/cartographer_ros"
        topic = f"/amr_slam/session_{session}/map"
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, lambda message: self.on_map(session, message),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.processes.append(subprocess.Popen([
            str(directory / "cartographer_node"),
            "-configuration_directory", str(Path(__file__).parent),
            "-configuration_basename", "amr_2d.lua",
            "--ros-args", "-p", "use_sim_time:=true",
        ]))
        self.processes.append(subprocess.Popen([
            str(directory / "cartographer_occupancy_grid_node"),
            "-resolution", "0.05", "-publish_period_sec", "1.0",
            "--ros-args", "-p", "use_sim_time:=true", "-r", f"map:={topic}",
        ]))

    def poll(self):
        try:
            response = self.request(f"/bridge?after={self.sequence}&previous={self.session or ''}")
            if response["session"] != self.session:
                self.switch(response["session"])
                return
            if any(process.poll() is not None for process in self.processes):
                raise RuntimeError("Cartographer exited. Check the SLAM container logs.")
            scan, odometry = response.get("scan"), response.get("odometry")
            if scan and odometry:
                self.publish(scan, odometry)
                self.sequence = scan["sequence"]
        except (OSError, RuntimeError, ValueError) as error:
            self.report(error)

    def report(self, error):
        message = str(error)
        if self.previous_error != message:
            self.get_logger().error(message)
            self.previous_error = message

    def publish(self, scan, odometry):
        clock = Clock()
        set_stamp(clock.clock, max(scan["stamp"], odometry["stamp"]))
        self.clock_pub.publish(clock)
        if odometry["stamp"] != self.odom_stamp:
            odom = Odometry()
            set_stamp(odom.header.stamp, odometry["stamp"])
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            set_components(odom.pose.pose.position, odometry["position"])
            set_components(odom.pose.pose.orientation, odometry["orientation"])
            set_components(odom.twist.twist.linear, odometry["linear_velocity"])
            set_components(odom.twist.twist.angular, odometry["angular_velocity"])
            self.odom_pub.publish(odom)
            self.odom_stamp = odometry["stamp"]
        message = LaserScan()
        set_stamp(message.header.stamp, scan["stamp"])
        message.header.frame_id = "reeman_laser"
        for name in ("angle_min", "angle_max", "range_min", "range_max"):
            setattr(message, name, float(scan[name]))
        message.angle_increment = float(scan["angle_step"])
        message.ranges = [math.inf if distance is None else distance for distance in scan["ranges"]]
        self.scan_pub.publish(message)

    def on_map(self, session, message):
        if session != self.session or not message.data:
            return
        origin = message.info.origin.position
        try:
            self.request(f"/bridge/{session}", {
                "width": message.info.width, "height": message.info.height,
                "resolution": message.info.resolution, "x": origin.x, "y": origin.y,
                "cells": list(message.data),
            })
        except (OSError, ValueError) as error:
            self.report(error)


if __name__ == "__main__":
    rclpy.init()
    bridge = MappingBridge()
    try:
        rclpy.spin(bridge)
    finally:
        bridge.close_session()
        bridge.destroy_node()
        rclpy.try_shutdown()
