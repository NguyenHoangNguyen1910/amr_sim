#!/usr/bin/env python3
"""Verify ROS messages, original processing nodes, TF and closed-loop movement."""
import math
from pathlib import Path
import sys
import threading
import time

from driver_msgs.msg import (
    BatteryInfo, DockInfraRed, PowerSystemEvent, RangeSensorEvent, RobotStateEvent, SensorState
)
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from rosgraph_msgs.msg import Clock
import rosnode
import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, Range
import tf2_ros


def wait_until(check, limit, description):
    end = time.monotonic() + limit
    while time.monotonic() < end and not rospy.is_shutdown():
        result = check()
        if result:
            return result
        time.sleep(0.1)
    raise RuntimeError(description)


def main():
    rospy.init_node("reeman_sim_smoke", anonymous=True)
    imu_frame = "base_link" if Path("/sim-state/device-inputs").exists() else "imu_link"
    topics = {
        "/clock": (Clock, lambda m: m.clock.to_sec() > 0),
        "/scan": (LaserScan, lambda m: m.header.frame_id == "laser_link" and len(m.ranges) > 1000
                  and any(math.isfinite(x) for x in m.ranges)),
        "/scan2": (LaserScan, lambda m: len(m.ranges) > 100),
        "/imu": (Imu, lambda m: m.header.frame_id == imu_frame
                 and 8 < m.linear_acceleration.z < 12),
        "/mobile/odom": (Odometry, lambda m: m.child_frame_id == "base_footprint"),
        "/odom": (Odometry, lambda m: m.child_frame_id == "base_footprint"),
        "/joint_states": (JointState, lambda m: "wheel_left_joint" in m.name),
        "/camera/depth/image_raw": (Image, lambda m: (m.width, m.height) == (320, 160)
                                    and m.encoding == "16UC1" and bool(m.data)),
        "/camera/depth/camera_info": (CameraInfo, lambda m: (m.width, m.height) == (320, 160)
                                      and m.K[0] > 0),
        "/camera/depth/points": (PointCloud2, lambda m: m.header.frame_id == "camera_depth_optical_frame"
                                  and m.width * m.height > 100 and bool(m.data)),
        "/camera/scan": (LaserScan, lambda m: len(m.ranges) > 100),
        "/lidar3d/points": (PointCloud2, lambda m: m.header.frame_id == "lidar3d_link"
                             and m.width * m.height > 100 and bool(m.data)),
        "/mobile_base/events/range_sensor": (RangeSensorEvent, lambda m: True),
        "/mobile_base/events/online": (RobotStateEvent, lambda m: True),
        "/mobile_base/events/battery": (PowerSystemEvent, lambda m: True),
        "/mobile_base/events/battery_info": (BatteryInfo, lambda m: m.nominal_voltage > 0),
        "/mobile_base/events/charger": (PowerSystemEvent, lambda m: True),
        "/mobile_base/sensors/core": (SensorState, lambda m: len(m.bottom) == 3),
        "/mobile_base/sensors/dock_ir": (DockInfraRed, lambda m: list(m.data) == [0, 0, 0]),
        "/map": (OccupancyGrid, lambda m: m.info.resolution > 0 and bool(m.data)),
    }
    for location in ("left", "center", "right"):
        topics[f"/ultrasonic/{location}"] = (Range, lambda m: m.radiation_type == Range.ULTRASOUND)
        topics[f"/cliff/{location}"] = (Range, lambda m: m.min_range < m.range < m.max_range)

    received = {}
    failures = {}
    lock = threading.Lock()
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    def observe(name, validator, message):
        with lock:
            try:
                if validator(message):
                    received[name] = message
                    failures.pop(name, None)
                elif name not in received:
                    failures[name] = "message does not satisfy the expected contract"
            except (ValueError, AttributeError, TypeError) as error:
                failures[name] = str(error)

    subscriptions = [
        rospy.Subscriber(name, msg_type, lambda message, n=name, v=validator: observe(n, v, message),
                         queue_size=1, buff_size=2 ** 23)
        for name, (msg_type, validator) in topics.items()
    ]

    def complete():
        with lock:
            return len(received) == len(topics)

    try:
        wait_until(complete, 65, "missing or invalid sensor/processing messages")
    except RuntimeError:
        with lock:
            for name in sorted(topics.keys() - received.keys()):
                print(f"FAIL {name}: {failures.get(name, 'no message')}", file=sys.stderr)
            for name in sorted(received):
                print(f"PASS {name}")
        raise

    for name in sorted(received):
        print(f"PASS {name}")
    nodes = {"/cartographer_node", "/ekf_se_odom", "/map_server", "/move_base",
             "/robot_state_publisher", "/pcl_filter", "/pointcloud_to_laserscan"}
    wait_until(lambda: nodes <= set(rosnode.get_node_names()), 10, "original nodes missing")
    for frame in ("laser_link", "imu_link", "camera_depth_optical_frame", "lidar3d_link"):
        wait_until(lambda: buffer.can_transform("base_link", frame, rospy.Time()), 5,
                   f"missing TF for {frame}")
    wait_until(lambda: buffer.can_transform("map", "base_footprint", rospy.Time()), 20,
               "Cartographer/EKF did not supply map -> base_footprint")
    print(f"PASS original nodes ({len(nodes)}), sensor TF and map -> base_footprint")

    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    wait_until(publisher.get_num_connections, 5, "velocity mux is not subscribed")
    start_pose = received["/mobile/odom"].pose.pose.position
    command = Twist()
    command.linear.x = 0.12

    def moved():
        publisher.publish(command)
        position = received["/mobile/odom"].pose.pose.position
        return math.hypot(position.x - start_pose.x, position.y - start_pose.y) > 0.025

    try:
        wait_until(moved, 8, "ROS /cmd_vel did not move the Gazebo robot")
    finally:
        for _ in range(4):
            publisher.publish(Twist())
            time.sleep(0.1)
    print("PASS /cmd_vel -> original velocity mux -> Gazebo -> /mobile/odom")
    if "--plan" in sys.argv:
        rospy.wait_for_service("/move_base/make_plan", 12)
        transform = buffer.lookup_transform("map", "base_footprint", rospy.Time()).transform
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position.x = transform.translation.x
        start.pose.position.y = transform.translation.y
        start.pose.orientation = transform.rotation
        goal = PoseStamped()
        goal.header = start.header
        goal.pose.position.x = start.pose.position.x + 0.35
        goal.pose.position.y = start.pose.position.y
        goal.pose.orientation = start.pose.orientation
        planner = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        plan = planner(start, goal, 0.1).plan
        if len(plan.poses) < 2:
            raise RuntimeError("Original move_base returned no room plan")
        print(f"PASS original move_base global planner ({len(plan.poses)} poses)")
    assert subscriptions and listener


if __name__ == "__main__":
    main()