"""Set a map pose through Reeman localization or the generated room's TF anchor."""

import math
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from yoyo_msgs.msg import Reloc

from docking import angle, orientation


ROOM_FRAME = "sim_local_map"
ROOM_CONFIG = "/sim-state/room-localization-config"


class Relocation:
    def __init__(self, buffer, room):
        self.buffer = buffer
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.room = room
        if room:
            self.broadcaster = tf2_ros.StaticTransformBroadcaster()
            self.anchor = (0.0, 0.0, 0.0)
            self.publish_anchor(self.anchor)
        else:
            self.request = rospy.Publisher("/robot_api/reloc", Reloc, queue_size=1)
            self.completed = threading.Event()
            self.waiting = False
            self.busy_seen = False
            self.readiness = rospy.Subscriber("/robot_api/localization_ready", Bool,
                                              self.on_readiness, queue_size=4)

    def on_readiness(self, message):
        if self.waiting:
            if not message.data:
                self.busy_seen = True
            elif self.busy_seen:
                self.completed.set()

    def pose_in(self, frame):
        try:
            tf = self.buffer.lookup_transform(frame, "base_footprint", rospy.Time())
        except tf2_ros.TransformException as error:
            raise RuntimeError("Chưa có pose robot từ Cartographer") from error
        stamp = tf.header.stamp
        if abs((rospy.Time.now() - stamp).to_sec()) > 1:
            raise RuntimeError("Pose robot đã quá cũ để Relocate")
        p = tf.transform.translation
        return {"x": p.x, "y": p.y, "yaw": orientation(tf.transform.rotation)}

    def publish_anchor(self, anchor):
        x, y, yaw = anchor
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "map"
        tf.child_frame_id = ROOM_FRAME
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw / 2)
        tf.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(tf)

    def apply_room(self, x, y, yaw):
        raw = self.pose_in(ROOM_FRAME)
        delta = angle(yaw - raw["yaw"])
        cosine, sine = math.cos(delta), math.sin(delta)
        anchor = (x - cosine * raw["x"] + sine * raw["y"],
                  y - sine * raw["x"] - cosine * raw["y"], delta)
        self.publish_anchor(anchor)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            try:
                current = self.pose_in("map")
            except RuntimeError:
                current = None
            if (current and math.hypot(current["x"] - x, current["y"] - y) < 0.06
                    and abs(angle(current["yaw"] - yaw)) < 0.08):
                self.anchor = anchor
                return current
            time.sleep(0.05)
        self.publish_anchor(self.anchor)
        raise RuntimeError("Chưa nhận pose mới trong hệ map")

    def apply_native(self, x, y, yaw):
        if not self.request.get_num_connections():
            raise RuntimeError("Node Relocate gốc của Reeman chưa sẵn sàng")
        self.completed.clear()
        self.busy_seen = False
        self.waiting = True
        try:
            self.request.publish(Reloc(x=x, y=y, z=yaw))
            if not self.completed.wait(12):
                raise RuntimeError("Reeman chưa xác nhận đã định vị lại trên bản đồ lưu")
        finally:
            self.waiting = False
        return self.pose_in("map")

    def apply(self, x, y, yaw):
        measured = self.apply_room(x, y, yaw) if self.room else self.apply_native(x, y, yaw)
        try:
            self.clear_costmaps()
        except rospy.ServiceException as error:
            rospy.logwarn("Relocate applied but move_base costmaps could not be cleared: %s", error)
        return measured
