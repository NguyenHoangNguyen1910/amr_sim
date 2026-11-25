"""Bridge Reeman's native docking request to a simulated rear contact."""

import math
from pathlib import Path
import tempfile
import threading

import rospy
import tf2_ros
import yaml
from driver_msgs.msg import BumperEvent, DockInfraRed, PowerSystemEvent
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Int32
from yocs_msgs.msg import NavigationControl, NavigationControlStatus
from yocs_msgs.srv import WaypointListService


DOCK_NAME = "charging_pile"
REAR_OFFSET = 0.27
APPROACH_DISTANCE = 0.60
DOCK_MOTION = ("ALIGNING", "REVERSING")
ACTIVE = ("NAVIGATING", *DOCK_MOTION)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def offset_pose(pose, distance):
    yaw = pose["yaw"]
    return {"x": pose["x"] + distance * math.cos(yaw),
            "y": pose["y"] + distance * math.sin(yaw), "yaw": yaw}


def orientation(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Docking:
    def __init__(self, file, buffer, navigation):
        self.file = file
        self.buffer = buffer
        self.navigation = navigation
        self.lock = threading.Lock()
        self.pose = self.read_pose()
        self.state = "IDLE"
        self.message = "Chọn vị trí dock sạc" if self.pose is None else "Đã đặt dock sạc"
        self.error = None
        self.contact_odom = None
        self.deadline = rospy.Time()
        self.velocity = rospy.Publisher("/input/auto_docking", Twist, queue_size=1)
        self.charger = rospy.Publisher("/mobile_base/events/charger", PowerSystemEvent, latch=True, queue_size=1)
        self.ir = rospy.Publisher("/mobile_base/sensors/dock_ir", DockInfraRed, queue_size=1)
        self.reload = rospy.ServiceProxy("/robot_api/request_waypoints", WaypointListService)
        self.subscribers = [
            rospy.Subscriber("/mobile_base/commands/set_docking", Int32, self.base_command, queue_size=4),
            rospy.Subscriber("/robot_api/nav_ctrl_status", NavigationControlStatus, self.nav_status, queue_size=8),
            rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper, queue_size=4),
        ]
        self.charger.publish(PowerSystemEvent(PowerSystemEvent.UNPLUGGED))
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def waypoints(self):
        return (yaml.safe_load(self.file.read_text()) or {}) if self.file.is_file() else {}

    def read_pose(self):
        waypoint = next((point for point in self.waypoints().get("waypoints", [])
                         if point.get("name") == DOCK_NAME and point.get("type") == "charge"), None)
        if waypoint is None:
            return None
        pose = waypoint["pose"]
        return {"x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose["theta"])}

    def status(self):
        with self.lock:
            pose = self.pose
            return {"state": self.state, "message": self.message, "error": self.error,
                    "active": self.state in ACTIVE, "charging": self.state == "CHARGING",
                    "pose": pose, "face": offset_pose(pose, -REAR_OFFSET) if pose else None,
                    "approach": offset_pose(pose, APPROACH_DISTANCE) if pose else None}

    def save(self, face, is_free):
        pose = offset_pose(face, REAR_OFFSET)
        for i in range(7):
            point = offset_pose(pose, i / 10)
            if not is_free(point["x"], point["y"]):
                raise ValueError("Cần chừa vùng trống dài 0,6 m trước dock")
        with self.lock:
            if self.state in ACTIVE:
                raise ValueError("Dừng về sạc trước khi đổi dock")
            config = self.waypoints()
            points = [p for p in config.get("waypoints", []) if p.get("name") != DOCK_NAME]
            points.append({"name": DOCK_NAME, "type": "charge",
                           "pose": {"x": pose["x"], "y": pose["y"], "theta": pose["yaw"]}})
            config["waypoints"] = points
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.file.parent,
                                             delete=False) as stream:
                temp = Path(stream.name)
                try:
                    yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
                    stream.flush()
                    temp.replace(self.file)
                finally:
                    try:
                        temp.unlink()
                    except FileNotFoundError:
                        pass
            if self.state == "CHARGING":
                self.charger.publish(PowerSystemEvent(PowerSystemEvent.UNPLUGGED))
            self.pose = pose
            self.state, self.message, self.error = "IDLE", "Đã lưu dock sạc", None

    def start(self):
        with self.lock:
            if self.pose is None:
                raise ValueError("Chọn vị trí dock trước khi về sạc")
            if self.state == "CHARGING":
                return
            if self.state in ACTIVE:
                raise ValueError("Robot đang về dock")
            if not self.navigation.ready or not self.navigation.control.get_num_connections():
                raise RuntimeError("Điều hướng Reeman chưa sẵn sàng")
        loaded = self.reload()
        if not loaded.success or not any(w.name == DOCK_NAME and w.type == "charge"
                                         for w in loaded.waypoints.waypoints):
            raise RuntimeError("Waypoint provider chưa nạp charging_pile")
        self.navigation.clear()
        with self.lock:
            self.state, self.message, self.error = "NAVIGATING", "Đang đến vị trí chờ trước dock", None
            self.deadline = rospy.Time.now() + rospy.Duration(120)
            self.navigation.control.publish(NavigationControl(
                control=NavigationControl.START, mode=NavigationControl.GOAL, goal_name=DOCK_NAME))

    def stop(self):
        with self.lock:
            if self.state not in ACTIVE:
                return
            self.finish("CANCELLED", "Đã dừng về dock", PowerSystemEvent.UNPLUGGED)
            self.navigation.control.publish(NavigationControl(
                control=NavigationControl.STOP, mode=NavigationControl.GOAL))

    def finish(self, state, message, event):
        self.velocity.publish(Twist())
        self.charger.publish(PowerSystemEvent(event))
        self.state, self.message = state, message

    def blocked(self, message):
        self.finish("BLOCKED", message, PowerSystemEvent.DOCKING_ERROR1)

    def base_command(self, msg):
        with self.lock:
            if msg.data == 1 and self.state == "NAVIGATING":
                self.state, self.message = "ALIGNING", "Đang căn hướng để lùi vào dock"
                self.deadline = rospy.Time.now() + rospy.Duration(35)
                self.charger.publish(PowerSystemEvent(PowerSystemEvent.DOCKING))
            elif msg.data == 0 and self.state in DOCK_MOTION:
                self.finish("CANCELLED", "Reeman đã dừng lùi vào dock", PowerSystemEvent.UNPLUGGED)

    def nav_status(self, msg):
        with self.lock:
            if self.state == "NAVIGATING" and msg.status < 0:
                self.blocked("Reeman không đến được vị trí chờ trước dock")

    def bumper(self, msg):
        if msg.state == BumperEvent.PRESSED:
            with self.lock:
                if self.state in DOCK_MOTION:
                    self.blocked("Dừng vì robot chạm vật cản khi lùi")

    def relative_pose(self):
        try:
            tf = self.buffer.lookup_transform("map", "base_footprint", rospy.Time())
        except tf2_ros.TransformException:
            return None
        stamp = tf.header.stamp
        if abs((rospy.Time.now() - stamp).to_sec()) > 0.5:
            return None
        yaw = self.pose["yaw"]
        dx = tf.transform.translation.x - self.pose["x"]
        dy = tf.transform.translation.y - self.pose["y"]
        return (dx * math.cos(yaw) + dy * math.sin(yaw),
                -dx * math.sin(yaw) + dy * math.cos(yaw),
                angle(orientation(tf.transform.rotation) - yaw), stamp)

    def odom_pose(self):
        try:
            tf = self.buffer.lookup_transform("odom", "base_footprint", rospy.Time())
        except tf2_ros.TransformException:
            return None
        if abs((rospy.Time.now() - tf.header.stamp).to_sec()) > 0.5:
            return None
        p = tf.transform.translation
        return p.x, p.y, orientation(tf.transform.rotation)

    def tick(self, _):
        with self.lock:
            publish_ir = self.ir.get_num_connections() > 0
            if self.pose is None:
                if publish_ir:
                    self.ir.publish(DockInfraRed(header=Header(stamp=rospy.Time.now(), frame_id="dock_ir_link"),
                                                 data=[0, 0, 0]))
                return
            if self.state in ACTIVE and rospy.Time.now() > self.deadline:
                if self.state == "NAVIGATING":
                    self.navigation.control.publish(NavigationControl(
                        control=NavigationControl.STOP, mode=NavigationControl.GOAL))
                self.blocked("Quá thời gian về dock")
                return
            if self.state == "CHARGING":
                current = self.odom_pose()
                if current is not None and self.contact_odom is not None:
                    x, y, heading = self.contact_odom
                    if math.hypot(current[0] - x, current[1] - y) > 0.11 or abs(angle(current[2] - heading)) > 0.2:
                        self.state, self.message = "IDLE", "Robot đã rời dock"
                        self.charger.publish(PowerSystemEvent(PowerSystemEvent.UNPLUGGED))
                if self.state == "CHARGING":
                    if publish_ir:
                        self.ir.publish(DockInfraRed(
                            header=Header(stamp=rospy.Time.now(), frame_id="dock_ir_link"),
                            data=[0, DockInfraRed.NEAR_CENTER, 0]))
                    return
            if self.state not in DOCK_MOTION and not publish_ir:
                return
            measured = self.relative_pose()
            if measured is None:
                if self.state in DOCK_MOTION:
                    self.velocity.publish(Twist())
                return
            forward, lateral, yaw, stamp = measured
            self.error = {"forward_m": forward, "lateral_m": lateral, "yaw_rad": yaw}
            if publish_ir:
                bits = [0, 0, 0]
                if -0.03 <= forward < 0.9 and abs(lateral) < 0.35 and abs(yaw) < 0.65:
                    near = forward < 0.36
                    bits[1] = ((DockInfraRed.NEAR_CENTER if near else DockInfraRed.FAR_CENTER)
                               if abs(lateral) < 0.06 else 0)
                    if lateral > 0.03:
                        bits[0] = DockInfraRed.NEAR_LEFT if near else DockInfraRed.FAR_LEFT
                    if lateral < -0.03:
                        bits[2] = DockInfraRed.NEAR_RIGHT if near else DockInfraRed.FAR_RIGHT
                self.ir.publish(DockInfraRed(header=Header(stamp=stamp, frame_id="dock_ir_link"), data=bits))
            if self.state not in DOCK_MOTION:
                return
            if forward < -0.045 or forward > 1.0 or abs(lateral) > 0.34:
                self.blocked("Robot lệch khỏi hành lang lùi vào dock")
                return
            if abs(forward) <= 0.025 and abs(lateral) <= 0.04 and abs(yaw) <= 0.08:
                self.contact_odom = self.odom_pose()
                if self.contact_odom is None:
                    self.velocity.publish(Twist())
                    return
                self.finish("CHARGING", "Đã tiếp xúc dock · đang sạc mô phỏng",
                            PowerSystemEvent.PLUGGED_TO_DOCKBASE)
                return
            target_yaw = max(-0.48, min(0.48, math.atan2(lateral, max(forward, 0.18)))) if forward > 0.045 else 0.0
            error = angle(target_yaw - yaw)
            command = Twist()
            command.angular.z = max(-0.30, min(0.30, 1.9 * error))
            if abs(error) < 0.18 and forward > 0.012:
                command.linear.x = -max(0.024, min(0.12, 0.5 * forward))
                self.state, self.message = "REVERSING", "Đang lùi vào mặt dock"
            elif forward <= 0.012 and abs(lateral) > 0.04:
                self.blocked("Đã tới mặt dock nhưng lệch tiếp điểm")
                return
            self.velocity.publish(command)
