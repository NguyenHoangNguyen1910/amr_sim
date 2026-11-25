"""Small web adapter for the original Reeman ROS graph."""

import json
from array import array
from collections import deque
import math
import mimetypes
from pathlib import Path
import socket
import socketserver
import shutil
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from defined_routes import DefinedRoutes, RouteModel
from docking import Docking
from obstacle_wait import ObstacleWait
from relocation import ROOM_CONFIG, Relocation


class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = {}
        self.map = None
        self.revision = 0
        self.navigation = []
        self.selected = None
        self.sent = False
        self.path = []
        self.speed = 0.5
        self.command_lock = threading.RLock()
        route_store = Path("/sim-routes")
        self.routes = DefinedRoutes(RouteModel(route_store / "model.xml")) if route_store.is_dir() else None
        self.obstacle_wait = ObstacleWait(route_store / "stop_move_time.txt") if self.routes else None
        self.scan = None
        self.recent_scans = deque(maxlen=12)
        self.imu = None
        self.camera_scan = None
        self.depth_message = None
        self.wheel_odom = None
        self.filtered_odom = None
        self.sequence = 0
        self.planner = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self.mapping_lock = threading.Lock()
        self.mapping_session = None
        self.mapping_grid = None
        self.mapping_revision = 0
        self.mapping_frame = None
        self.mapping_tf = None
        self.mapping_odom_tf = None
        self.mapping_overlay = None
        self.mapping_processes = []
        self.mapping_subscriptions = []
        self.room_mode = Path("/sim-state/room-mode").is_file()
        self.camera_demo = Path("/sim-state/camera-obstacle-demo").is_file()
        self.room_localization = (
            rospy.Publisher("/robot_api/localization_ready", Bool, latch=True, queue_size=1)
            if self.room_mode and self.routes else None
        )
        self.room_ready = False
        self.room_readiness_timer = None
        self.relocating = False
        rospy.on_shutdown(self.stop_mapping)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.relocation = (Relocation(self.buffer, self.room_mode)
                           if self.routes and (not self.room_mode or Path(ROOM_CONFIG).is_dir()) else None)
        self.dock = Docking(route_store / "waypoint.yaml", self.buffer, self.routes) if self.routes else None
        self.action = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.teleop = rospy.Publisher("/input/key_cmd_vel", Twist, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1),
            rospy.Subscriber("/move_base/status", GoalStatusArray, self.on_navigation, queue_size=1),
            rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1),
            rospy.Subscriber("/scan2", LaserScan, self.on_filtered, queue_size=1),
            rospy.Subscriber("/camera/scan", LaserScan, self.on_camera_scan, queue_size=1),
            rospy.Subscriber("/camera/depth/image_raw", Image, self.on_depth, queue_size=1),
            rospy.Subscriber("/imu", Imu, self.on_imu, queue_size=1),
            rospy.Subscriber("/mobile/odom", Odometry, self.on_wheel_odom, queue_size=1),
            rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1),
        ]
        if self.room_mode:
            threading.Thread(target=self.initialize_room, daemon=True).start()

    def initialize_room(self):
        """Start the original SLAM trajectory after Gazebo and EKF have data."""
        deadline = time.monotonic() + 120
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = self.scan is not None and "EKF /odom" in self.samples
            if ready:
                try:
                    self.start_trajectory(
                        "/start_trajectory",
                        ROOM_CONFIG if self.relocation else
                        "/home/reeman/catkin_ws/install_isolated/share/cartographer_ros/configuration_files",
                        "room",
                    )
                    if self.room_localization is not None:
                        self.room_readiness_timer = rospy.Timer(rospy.Duration(0.5), self.update_room_readiness)
                except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
                    rospy.logerr("Could not initialize room Cartographer: %s", error)
                return
            time.sleep(0.25)

    def update_room_readiness(self, _):
        if self.relocating:
            return
        ready = False
        try:
            pose = self.buffer.lookup_transform("map", "base_footprint", rospy.Time())
            now = rospy.Time.now()
            with self.lock:
                data_stamps = (self.scan.header.stamp, self.filtered_odom.header.stamp)
            ready = all(-0.1 <= (now - stamp).to_sec() < 1.0
                        for stamp in (*data_stamps, pose.header.stamp))
        except tf2_ros.TransformException:
            pass
        self.set_room_ready(ready)

    def set_room_ready(self, ready):
        if ready != self.room_ready:
            self.room_ready = ready
            self.room_localization.publish(Bool(ready))

    @staticmethod
    def start_trajectory(service, directory, name):
        rospy.wait_for_service(service, timeout=12)
        request = StartTrajectoryRequest()
        request.configuration_directory = str(directory)
        request.configuration_basename = "slam_2d.lua"
        request.trajectory_type = "slam"
        request.trajectory_id = name
        result = rospy.ServiceProxy(service, StartTrajectory)(request)
        if result.status.code != 0:
            raise RuntimeError(result.status.message)

    def record(self, name, message, value):
        with self.lock:
            self.samples[name] = {"seconds": message.header.stamp.to_sec(), "value": value}

    def on_scan(self, msg):
        self.record("LiDAR /scan", msg, "{} tia".format(len(msg.ranges)))
        with self.lock:
            self.scan = msg
            self.recent_scans.append(msg)

    def on_filtered(self, msg):
        self.record("LiDAR /scan2", msg, "{} tia".format(len(msg.ranges)))

    def on_camera_scan(self, msg):
        with self.lock:
            self.camera_scan = msg
            self.samples["Camera /camera/scan"] = {"seconds": msg.header.stamp.to_sec(),
                                                  "value": "{} tia".format(len(msg.ranges))}

    def on_depth(self, msg):
        self.record("Camera depth", msg, "{} × {}".format(msg.width, msg.height))
        with self.lock:
            self.depth_message = msg
            self.sequence += 1

    def on_imu(self, msg):
        self.record("IMU /imu", msg, "{:.3f} rad/s".format(msg.angular_velocity.z))
        with self.lock:
            self.imu = msg

    def on_odom(self, msg):
        self.record("EKF /odom", msg, "{:.3f} m/s".format(msg.twist.twist.linear.x))
        with self.lock:
            self.filtered_odom = msg

    def on_wheel_odom(self, msg):
        with self.lock:
            self.wheel_odom = msg

    def wheel_feedback(self):
        with self.lock:
            samples = {"/mobile/odom": self.wheel_odom, "/odom": self.filtered_odom}
        return {
            topic: None if msg is None else {
                "stamp": msg.header.stamp.to_sec(), "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                "linear_mps": msg.twist.twist.linear.x, "angular_rps": msg.twist.twist.angular.z,
            }
            for topic, msg in samples.items()
        }

    def on_navigation(self, msg):
        with self.lock:
            self.navigation = [{"state": s.status, "text": s.text} for s in msg.status_list[-8:]]

    def on_map(self, msg):
        info = msg.info
        origin = info.origin.position
        cells = bytes(value + 1 for value in msg.data)
        with self.lock:
            if self.map and self.map[1] == cells and self.map[0]["origin"] == [origin.x, origin.y]:
                return
            self.revision += 1
            self.map = ({
                "width": info.width, "height": info.height, "resolution": info.resolution,
                "origin": [origin.x, origin.y], "revision": self.revision,
            }, cells)

    def snapshot(self):
        pose = None
        try:
            transform = self.buffer.lookup_transform("map", "base_footprint", rospy.Time()).transform
            q = transform.rotation
            pose = {"x": transform.translation.x, "y": transform.translation.y,
                    "yaw": math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))}
        except tf2_ros.TransformException:
            pass
        with self.lock:
            return {"pose": pose, "sensors": dict(self.samples), "navigation": self.navigation,
                    "map": self.map[0] if self.map else None}

    def goal(self, x, y, yaw):
        if self.relocating:
            raise ValueError("Đang đặt lại định vị robot")
        if self.dock and self.dock.status()["active"]:
            raise ValueError("Dừng về sạc trước khi chọn đích khác")
        if self.routes:
            if self.routes.status()[0] == "NAVIGATING":
                raise ValueError("Dừng tuyến đang chạy trước khi chọn đích tự do")
            self.routes.clear()
        if self.action.action_client.pub_goal.get_num_connections() == 0:
            raise RuntimeError("move_base chưa sẵn sàng")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2)
        self.action.send_goal(goal)

    @staticmethod
    def stamped(x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def plan(self, goal):
        pose = self.snapshot()["pose"]
        if not pose:
            raise RuntimeError("Cartographer chưa có vị trí")
        if self.action.simple_state == actionlib.SimpleGoalState.ACTIVE:
            raise RuntimeError("Dừng hành trình hiện tại trước khi lập đường")
        if self.routes:
            self.routes.clear()
        plan = self.planner(
            self.stamped(pose["x"], pose["y"], pose["yaw"]),
            self.stamped(goal["x"], goal["y"], pose["yaw"]), 0.0,
        )
        points = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in plan.plan.poses]
        if not points:
            raise ValueError("move_base không tìm thấy đường đi")
        self.selected, self.path, self.sent = goal, points, False

    def check_route_idle(self):
        if self.relocating:
            raise ValueError("Đang đặt lại định vị robot")
        if self.dock and self.dock.status()["active"]:
            raise ValueError("Dừng về sạc trước khi thực hiện thao tác này")
        if self.routes and self.routes.status()[0] == "NAVIGATING":
            raise ValueError("Dừng tuyến đang chạy trước khi thực hiện thao tác này")
        if self.action.simple_state == actionlib.SimpleGoalState.ACTIVE:
            raise ValueError("Dừng điều hướng hiện tại trước khi đổi tuyến")

    def save_route_graph(self, data):
        with self.command_lock:
            self.check_route_idle()
            graph = self.routes.model.save(data)
            if self.selected and self.selected.get("kind") == "defined-route":
                self.selected, self.path, self.sent = None, [], False
            return graph

    def plan_defined_route(self, name):
        with self.command_lock:
            self.check_route_idle()
            self.selected, self.path = self.routes.plan(name)
            self.sent = False

    def start_navigation(self):
        with self.command_lock:
            if not self.selected or self.sent:
                raise ValueError("Cần lập đường trước")
            if self.selected.get("kind") == "defined-route":
                self.routes.start(self.selected["name"])
            else:
                self.goal(self.selected["x"], self.selected["y"], 0)
            self.sent = True

    def free_map_point(self, x, y):
        with self.lock:
            image = self.map
        if image is None:
            return False
        info, cells = image
        col, row = (math.floor((v - origin) / info["resolution"])
                    for v, origin in zip((x, y), info["origin"]))
        return (0 <= col < info["width"] and 0 <= row < info["height"]
                and 0 < cells[row * info["width"] + col] < 66)

    def gazebo_barriers(self, command, **data):
        with self.command_lock:
            if command == "remove":
                return obstacle_request(command, **data)
            if command == "add" and not self.free_map_point(*data["pose"][:2]):
                raise ValueError("Chọn vùng trống trên bản đồ để đặt vật cản")
            pose = self.relocation.pose_in("map")
            return obstacle_request(command, robot=(pose["x"], pose["y"], pose["yaw"]), **data)

    def save_dock(self, pose):
        with self.command_lock:
            self.check_route_idle()
            self.dock.save(pose, self.free_map_point)
            if self.selected and self.selected.get("kind") == "charging-dock":
                self.selected, self.path, self.sent = None, [], False
        return self.dock.status()

    def return_to_dock(self):
        with self.command_lock:
            self.check_route_idle()
            self.dock.start()
            self.selected = {**self.dock.status()["pose"], "name": "charging_pile", "kind": "charging-dock"}
            self.path = []
            self.sent = True
        return self.dock.status()

    def relocate(self, x, y, yaw):
        with self.command_lock:
            if not self.free_map_point(x, y):
                raise ValueError("Chọn vị trí robot trong vùng trống trên bản đồ")
            self.relocating = True
            try:
                self.cancel_navigation()
                if self.room_localization is not None:
                    self.set_room_ready(False)
                return self.relocation.apply(x, y, yaw)
            finally:
                self.relocating = False
                if self.room_localization is not None and self.room_readiness_timer is not None:
                    self.update_room_readiness(None)

    def cancel_navigation(self):
        with self.command_lock:
            if self.dock:
                self.dock.stop()
            if self.routes:
                self.routes.stop()
            self.action.cancel_all_goals()
            self.drive(0.0, 0.0)
            self.selected, self.path, self.sent = None, [], False

    def nav_status(self):
        states = {0: "NAVIGATING", 1: "NAVIGATING", 2: "CANCELLED", 3: "SUCCEEDED",
                  4: "BLOCKED", 5: "ERROR", 6: "NAVIGATING", 7: "NAVIGATING"}
        state = ("IDLE" if self.selected is None else
                 states.get(self.action.get_state(), "ERROR") if self.sent else "READY")
        message = self.action.get_goal_status_text() if self.selected and self.sent else state
        result = {"state": state, "goal": self.selected, "global_path": self.path,
                  "message": message, "speed_mps": self.speed}
        if self.selected and self.selected.get("kind") == "defined-route":
            if self.sent:
                state, message, nodes = self.routes.status()
                result.update(state=state, message=message, route_points=nodes)
            else:
                result["message"] = "Sẵn sàng chạy tuyến đến " + self.selected["name"]
        elif self.selected and self.selected.get("kind") == "charging-dock":
            dock = self.dock.status()
            state = "NAVIGATING" if dock["active"] else ("SUCCEEDED" if dock["charging"] else dock["state"])
            result.update(state=state, message=dock["message"])
        return result

    def sensor_snapshot(self):
        with self.lock:
            scan, imu, depth, sequence = self.scan, self.imu, self.depth_message, self.sequence
            camera = self.camera_scan
        now = rospy.Time.now()
        def packet(message, data):
            return {"data": data, "fresh": message is not None,
                    "age_seconds": (now - message.header.stamp).to_sec() if message else None}
        scan_data = {"ranges": [v if math.isfinite(v) else None for v in scan.ranges],
                     "angle_min": scan.angle_min, "angle_max": scan.angle_max,
                     "angle_step": scan.angle_increment, "range_max": scan.range_max,
                     "count": len(scan.ranges)} if scan else None
        imu_data = {"yaw": 2 * math.atan2(imu.orientation.z, imu.orientation.w),
                    "angular_velocity": {"z": imu.angular_velocity.z},
                    "linear_acceleration": {"z": imu.linear_acceleration.z}} if imu else None
        sensors = {"reeman_lidar": packet(scan, scan_data), "imu": packet(imu, imu_data),
                   "depth": packet(depth, {"width": depth.width, "height": depth.height,
                                           "sequence": sequence} if depth else None)}
        if self.routes:
            camera_data = None
            if camera:
                closest = min((r for r in camera.ranges if camera.range_min <= r <= camera.range_max), default=None)
                camera_data = {"closest_m": closest, "frame": camera.header.frame_id}
            sensors["camera_scan"] = packet(camera, camera_data)
        return sensors

    def receive_mapping_grid(self, message, session):
        with self.mapping_lock:
            if session != self.mapping_session or not self.mapping_processes:
                return
            info = message.info
            self.mapping_frame = message.header.frame_id
            grid = {"width": info.width, "height": info.height,
                    "resolution": info.resolution, "x": info.origin.position.x,
                    "y": info.origin.position.y, "cells": list(message.data)}
            if self.mapping_grid != grid:
                self.mapping_grid = grid
                self.mapping_revision += 1

    def start_mapping(self):
        with self.mapping_lock:
            if self.mapping_processes:
                raise ValueError("Đang dựng bản đồ")
            cartographer = Path("/home/reeman/catkin_ws/install_isolated")
            config = cartographer / "share/cartographer_ros/configuration_files"
            executables = cartographer / "lib/cartographer_ros"
            session = Path(tempfile.mkdtemp(prefix="scan_", dir="/sim-state"))
            shutil.copytree(config, session / "config")
            state = session / "maps"
            state.mkdir()
            lua = session / "config/slam_2d.lua"
            text = lua.read_text()
            text = text.replace('"/data/maps/"', '"{}/"'.format(state))
            if self.room_mode and not Path("/sim-state/native-slam-profile").is_file():
                # The virtual base has a stable EKF heading, including
                # during turns where the local scan match can be ambiguous.
                text = text.replace(
                    "return options",
                    "TRAJECTORY_BUILDER_2D.scan_matcher_variance = 1e-5\n"
                    "TRAJECTORY_BUILDER_2D.pose_tracker.orientation_model_variance = 1e-4\n\n"
                    "return options",
                )
            lua.write_text(text)
            namespace = "/amr_slam/" + session.name
            self.mapping_session = session.name
            self.mapping_grid = None
            self.mapping_revision = 0
            self.mapping_frame = None
            self.mapping_overlay = None
            buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5))
            self.mapping_tf = buffer

            def record_transforms(message, static):
                setter = buffer.set_transform_static if static else buffer.set_transform
                for transform in message.transforms:
                    setter(transform, namespace)

            # Keep the live map transform out of the independent SLAM session.
            # Cartographer still needs the EKF's odom to base transform to
            # publish its own map to odom transform, as in the native launch.
            self.mapping_odom_tf = rospy.Publisher(namespace + "/tf", TFMessage, queue_size=10)
            odom_tf = self.mapping_odom_tf

            def forward_odometry_transform(message):
                transforms = [transform for transform in message.transforms
                              if transform.header.frame_id == "odom"
                              and transform.child_frame_id == "base_footprint"]
                if transforms:
                    odom_tf.publish(TFMessage(transforms))

            self.mapping_subscriptions = [
                rospy.Subscriber(namespace + "/map", OccupancyGrid, self.receive_mapping_grid,
                                 session.name, queue_size=1),
                rospy.Subscriber(namespace + "/tf", TFMessage, record_transforms, False, queue_size=10),
                rospy.Subscriber("/tf_static", TFMessage, record_transforms, True, queue_size=10),
                rospy.Subscriber("/tf", TFMessage, forward_odometry_transform, queue_size=10),
            ]
            log = (session / "cartographer.log").open("ab")
            try:
                self.mapping_processes.append(subprocess.Popen([
                    str(executables / "cartographer_node"),
                    "-configuration_directory", str(lua.parent),
                    "-configuration_basename", lua.name,
                    "-start_trajectory_with_default_topics=false",
                    "__ns:=" + namespace, "scan:=/scan", "odom:=/odom", "/tf:=" + namespace + "/tf",
                ], stdout=log, stderr=log))
                self.mapping_processes.append(subprocess.Popen([
                    str(executables / "cartographer_occupancy_grid_node"), "-resolution", "0.05",
                    "__ns:=" + namespace, "/map:=" + namespace + "/map",
                ], stdout=log, stderr=log))
            finally:
                log.close()
        service = namespace + "/start_trajectory"
        try:
            self.start_trajectory(service, lua.parent, session.name)
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            self.stop_mapping()
            raise RuntimeError("Cartographer: {}".format(error)) from error

    def stop_mapping(self):
        self.update_mapping_overlay()
        with self.mapping_lock:
            for process in self.mapping_processes:
                if process.poll() is None:
                    process.terminate()
            for process in self.mapping_processes:
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            self.mapping_processes.clear()
            for subscriber in self.mapping_subscriptions:
                subscriber.unregister()
            self.mapping_subscriptions.clear()
            if self.mapping_odom_tf is not None:
                self.mapping_odom_tf.unregister()
                self.mapping_odom_tf = None
            self.mapping_tf = None

    @staticmethod
    def transform_pose(transform):
        translation, q = transform.translation, transform.rotation
        return {"x": translation.x, "y": translation.y,
                "yaw": math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))}

    def update_mapping_overlay(self):
        with self.mapping_lock:
            buffer, frame = self.mapping_tf, self.mapping_frame
            previous = self.mapping_overlay
        if buffer is None or not frame:
            return
        try:
            pose = self.transform_pose(buffer.lookup_transform(frame, "base_footprint", rospy.Time()).transform)
        except tf2_ros.TransformException:
            return

        with self.lock:
            recent = tuple(self.recent_scans)
        scan = previous["scan"] if previous else None
        for sample in reversed(recent):
            stamp = sample.header.stamp.to_nsec()
            if scan and stamp <= scan["stamp"]:
                break
            try:
                laser = self.transform_pose(buffer.lookup_transform(
                    frame, sample.header.frame_id.lstrip("/"), sample.header.stamp,
                ).transform)
            except tf2_ros.TransformException:
                continue
            scan = {
                **laser, "stamp": stamp, "angle_min": sample.angle_min,
                "angle_step": sample.angle_increment,
                "ranges": [value if math.isfinite(value) and sample.range_min <= value <= sample.range_max
                           else None for value in sample.ranges],
            }
            break
        with self.mapping_lock:
            if buffer is self.mapping_tf:
                self.mapping_overlay = {"pose": pose, "scan": scan}

    def mapping_status(self, after=-1, previous=""):
        self.update_mapping_overlay()
        with self.mapping_lock:
            online = all(p.poll() is None for p in self.mapping_processes)
            return {"active": bool(self.mapping_processes), "online": online,
                    "session": self.mapping_session, "revision": self.mapping_revision,
                    "overlay": self.mapping_overlay,
                    "grid": self.mapping_grid if after != self.mapping_revision or previous != self.mapping_session else None}

    def drive(self, linear, angular):
        if self.room_mode and abs(angular) > 0.2:
            with self.mapping_lock:
                if self.mapping_processes:
                    # The native matcher can lose alignment on fast virtual turns.
                    angular = math.copysign(0.2, angular)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.teleop.publish(cmd)


class Dashboard(BaseHTTPRequestHandler):
    def address_string(self):
        return "local"

    def log_message(self, *_):
        pass

    def reply(self, status, body, content_type="application/json", headers=None):
        if isinstance(body, dict):
            body = json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in (headers or {}).items():
            self.send_header(key, str(value))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        robot = self.server.robot
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            self.reply(200, robot.snapshot())
        elif path == "/api/base/odometry":
            self.reply(200, robot.wheel_feedback())
        elif path in ("/api/amr/telemetry", "/api/map/live"):
            data = robot.snapshot()
            pose = data["pose"]
            model_pose = {"position": {"x": pose["x"], "y": pose["y"]}, "yaw": pose["yaw"]} if pose else None
            if path == "/api/map/live":
                status = {"pose": model_pose, "navigation": robot.nav_status()}
                if robot.dock:
                    status["dock"] = robot.dock.status()
                if robot.relocation:
                    status["relocate_available"] = True
                if robot.camera_demo:
                    status["camera_obstacle_demo"] = True
                if robot.obstacle_wait:
                    status["obstacle_wait"] = robot.obstacle_wait.status()
                self.reply(200, status)
            else:
                self.reply(200, {"sensors": {"model_pose": model_pose},
                                 "freshness": {key: {"fresh": rospy.Time.now().to_sec() - item["seconds"] < 2}
                                               for key, item in data["sensors"].items()}})
        elif path in ("/api/health", "/api/navigation/status"):
            state = robot.nav_status()
            self.reply(200, state if path.endswith("status") else {"navigation": state})
        elif path == "/api/sensors":
            self.reply(200, robot.sensor_snapshot())
        elif path == "/api/sensors/depth":
            with robot.lock:
                msg, sequence = robot.depth_message, robot.sequence
            payload = None
            step = 0
            if msg and not msg.is_bigendian:
                if msg.encoding == "32FC1":
                    payload, step = bytes(msg.data), msg.step
                elif msg.encoding in ("16UC1", "mono16") and msg.step == msg.width * 2:
                    millimeters = array("H")
                    millimeters.frombytes(bytes(msg.data))
                    payload = array("f", (value * 0.001 if value else math.nan for value in millimeters)).tobytes()
                    step = msg.width * 4
            if payload is not None:
                self.reply(200, payload, "application/octet-stream", {
                    "X-Depth-Width": msg.width, "X-Depth-Height": msg.height,
                    "X-Depth-Step": step, "X-Depth-Sequence": sequence,
                })
            else:
                self.reply(503, {"error": "Chưa nhận ảnh depth float32"})
        elif path in ("/api/map", "/api/map/grid"):
            with robot.lock:
                image = robot.map
            if image:
                info, cells = image
                if path.endswith("grid"):
                    self.reply(200, cells, "application/octet-stream")
                else:
                    x, y = info["origin"]
                    self.reply(200, {**info, "obstacles": [], "bounds": {
                        "min_x": x, "max_x": x + info["width"] * info["resolution"],
                        "min_y": y, "max_y": y + info["height"] * info["resolution"],
                    }})
            else:
                self.reply(503, {"error": "Chưa nhận bản đồ từ ROS"})
        elif path == "/api/map/walls":
            self.reply(200, {"walls": [], "editable": False})
        elif path == "/api/navigation/graph" and robot.routes:
            with robot.command_lock:
                self.reply(200, robot.routes.model.graph())
        elif path == "/api/dock" and robot.dock:
            self.reply(200, robot.dock.status())
        elif path == "/api/obstacles/demo" and robot.camera_demo:
            try:
                self.reply(200, obstacle_request("status"))
            except RuntimeError as error:
                self.reply(503, {"error": str(error)})
        elif path == "/api/obstacles/barriers" and robot.camera_demo:
            try:
                self.reply(200, robot.gazebo_barriers("list"))
            except RuntimeError as error:
                self.reply(503, {"error": str(error)})
        elif path == "/api/slam/map":
            from urllib.parse import parse_qs, urlsplit
            query = parse_qs(urlsplit(self.path).query)
            self.reply(200, robot.mapping_status(int(query.get("after", [-1])[0]), query.get("previous", [""])[0]))
        elif path == "/api/version":
            self.reply(200, {"backend": "reeman-noetic",
                             **({"directed_routes": True, "docking": True} if robot.routes else {}),
                             **({"relocate": True} if robot.relocation else {})})
        else:
            root = Path(__file__).with_name("viewer")
            asset = (root / (path.lstrip("/") or "index.html")).resolve()
            if root not in asset.parents or not asset.is_file():
                self.reply(404, {"error": "Không tìm thấy"})
                return
            mime = mimetypes.guess_type(str(asset))[0] or "application/octet-stream"
            self.reply(200, asset.read_bytes(), mime)

    def do_POST(self):
        robot = self.server.robot
        try:
            length = int(self.headers.get("Content-Length", "0"))
            maximum = 65536 if self.path == "/api/navigation/graph" else 4096
            if not 0 <= length <= maximum:
                raise ValueError("Yêu cầu quá lớn")
            if length and self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("Cần gửi JSON")
            data = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(data, dict):
                raise ValueError("Cần gửi đối tượng JSON")
            if self.path == "/api/goal":
                with robot.command_lock:
                    robot.goal(*coordinates(data, ("x", "y", "yaw"), (1000, 1000, 2*math.pi)))
            elif self.path in ("/api/drive", "/api/amr/command"):
                linear, angular = coordinates(data, ("linear", "angular"), (0.9, 2.84))
                if linear or angular:
                    if robot.relocating:
                        raise ValueError("Đang đặt lại định vị robot")
                    if (robot.dock and robot.dock.status()["active"]
                            or robot.routes and robot.routes.status()[0] == "NAVIGATING"):
                        robot.cancel_navigation()
                    elif robot.sent and robot.action.get_state() in (0, 1):
                        robot.action.cancel_goal()
                robot.drive(linear, angular)
            elif self.path in ("/api/stop", "/api/navigation/cancel", "/api/amr/reset"):
                robot.cancel_navigation()
                self.reply(200, {"ok": True, **robot.nav_status()})
                return
            elif self.path in ("/api/navigation/plan", "/api/navigation/validate-goal"):
                x, y = coordinates(data["goal"], ("x", "y"), (1000, 1000))
                if self.path.endswith("validate-goal"):
                    valid = robot.free_map_point(x, y)
                    self.reply(200, {"ok": True, "valid": valid, "message": "Điểm nằm ngoài vùng trống" if not valid else ""})
                    return
                with robot.command_lock:
                    robot.check_route_idle()
                    robot.plan({"x": x, "y": y})
                self.reply(200, {"ok": True, **robot.nav_status()})
                return
            elif self.path == "/api/navigation/graph" and robot.routes:
                self.reply(200, robot.save_route_graph(data))
                return
            elif self.path == "/api/navigation/graph/plan" and robot.routes:
                robot.plan_defined_route(data["name"])
                self.reply(200, {"ok": True, **robot.nav_status()})
                return
            elif self.path == "/api/dock" and robot.dock:
                x, y, yaw = coordinates(data, ("x", "y", "yaw"), (1000, 1000, math.tau))
                self.reply(200, robot.save_dock({"x": x, "y": y, "yaw": yaw}))
                return
            elif self.path == "/api/dock/return" and robot.dock:
                self.reply(200, robot.return_to_dock())
                return
            elif self.path == "/api/obstacles/demo" and robot.camera_demo:
                enabled = data.get("active")
                if not isinstance(enabled, bool):
                    raise ValueError("Trạng thái vật cản cần là true hoặc false")
                self.reply(200, obstacle_request("on" if enabled else "off"))
                return
            elif self.path == "/api/obstacles/barriers" and robot.camera_demo:
                request = data["action"]
                if request == "add":
                    pose = coordinates(data, ("x", "y", "yaw"), (1000, 1000, math.tau))
                    response = robot.gazebo_barriers("add", pose=pose)
                elif request == "remove" and type(data.get("id")) is int:
                    response = robot.gazebo_barriers("remove", id=data["id"])
                else:
                    raise ValueError("Lệnh đặt hoặc gỡ vật cản không hợp lệ")
                self.reply(200, response)
                return
            elif self.path == "/api/relocate" and robot.relocation:
                pose = robot.relocate(*coordinates(data, ("x", "y", "yaw"), (1000, 1000, math.tau)))
                self.reply(200, {"pose": pose, "message": "Đã đặt lại định vị robot"})
                return
            elif self.path == "/api/navigation/start":
                robot.start_navigation()
                self.reply(200, {"ok": True, **robot.nav_status()})
                return
            elif self.path == "/api/navigation/obstacle-wait" and robot.obstacle_wait:
                self.reply(200, robot.obstacle_wait.set(data["seconds"]))
                return
            elif self.path == "/api/amr/speed":
                robot.speed, = coordinates(data, ("speed_mps",), (0.9,))
                self.reply(200, {"ok": True, "speed_mps": robot.speed})
                return
            elif self.path in ("/api/slam/start", "/api/slam/stop"):
                if self.path.endswith("start"):
                    robot.start_mapping()
                else:
                    robot.stop_mapping()
                self.reply(200, robot.mapping_status())
                return
            else:
                self.reply(404, {"error": "Không tìm thấy"})
                return
            self.reply(200, {"ok": True})
        except (ValueError, KeyError, TypeError) as error:
            self.reply(422, {"error": str(error)})
        except (RuntimeError, rospy.ServiceException) as error:
            self.reply(503, {"error": str(error)})


def obstacle_request(command, **data):
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(2)
            client.connect("/sim-state/camera-obstacle.sock")
            client.sendall(json.dumps({"command": command, **data}).encode("utf-8") + b"\n")
            with client.makefile("rb") as response:
                state = json.loads(response.readline(4096))
    except (OSError, ValueError) as error:
        raise RuntimeError("Chưa kết nối được bộ mô phỏng vật cản Gazebo") from error
    if state.get("invalid"):
        raise ValueError(state["error"])
    if state.get("error"):
        raise RuntimeError(state["error"])
    return state


def coordinates(data, fields, limits):
    values = tuple(float(data[field]) for field in fields)
    if any(not math.isfinite(v) or abs(v) > limit for v, limit in zip(values, limits)):
        raise ValueError("Giá trị nằm ngoài phạm vi")
    return values


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


if __name__ == "__main__":
    rospy.init_node("amr_sim_dashboard")
    with Server("/sim-state/dashboard.sock", Dashboard) as server:
        server.robot = Robot()
        server.timeout = 0.2
        while not rospy.is_shutdown():
            server.handle_request()
