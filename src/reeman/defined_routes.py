"""Save directed route models and submit them to the original Reeman nodes."""

import math
from pathlib import Path
import re
import tempfile
import threading
import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import Bool, String
from yocs_msgs.msg import NavigationControl, NavigationControlStatus
from yoyo_msgs.srv import LoadPlanService, MakePlanService


class RouteModel:
    def __init__(self, file):
        self.file = file
        self.document = ET.parse(file).getroot() if file.is_file() else ET.Element("Model")

    def graph(self):
        return {
            "points": [
                {"name": item.attrib["name"], "x": float(item.attrib["xPosition"]) / 1000,
                 "y": float(item.attrib["yPosition"]) / 1000,
                 "yaw": float(item.get("vehicleOrientationAngle") or 0) / 1000,
                 "type": item.get("type", "delivery")}
                for item in self.document.findall("point")
            ],
            "edges": [
                {"source": item.attrib["sourcePoint"], "destination": item.attrib["destinationPoint"]}
                for item in self.document.findall("path")
            ],
        }

    def save(self, data):
        if not isinstance(data.get("points"), list) or not isinstance(data.get("edges"), list):
            raise ValueError("Cần danh sách điểm và đoạn tuyến")
        points = {}
        for item in data["points"]:
            name = item["name"]
            if not re.fullmatch(r"[\w.-]{1,40}", name) or "---" in name or name in points:
                raise ValueError("Tên điểm phải hợp lệ và duy nhất")
            x, y = float(item["x"]), float(item["y"])
            yaw = float(item.get("yaw", 0))
            if any(not math.isfinite(v) or abs(v) > bound
                   for v, bound in ((x, 1000), (y, 1000), (yaw, math.tau))):
                raise ValueError("Tọa độ hoặc góc của điểm nằm ngoài phạm vi")
            points[name] = {"x": round(x * 1000), "y": round(y * 1000), "yaw": round(yaw * 1000)}

        edges = {}
        for item in data["edges"]:
            source, destination = item["source"], item["destination"]
            if source not in points or destination not in points:
                raise ValueError("Đoạn tuyến phải nối các điểm đã tạo")
            a, b = points[source], points[destination]
            distance = round(math.hypot(b["x"] - a["x"], b["y"] - a["y"]))
            if not distance or (source, destination) in edges:
                raise ValueError("Các đoạn tuyến phải khác nhau và có độ dài dương")
            edges[source, destination] = distance

        original_points = {item.get("name"): item for item in self.document.findall("point")}
        original_paths = {(item.get("sourcePoint"), item.get("destinationPoint")): item
                          for item in self.document.findall("path")}
        document = ET.Element("Model", self.document.attrib)
        for name, point in points.items():
            original = original_points.get(name)
            attributes = dict(original.attrib) if original is not None else {"type": "delivery"}
            attributes.update(name=name, xPosition=str(point["x"]), yPosition=str(point["y"]),
                              vehicleOrientationAngle=str(point["yaw"]))
            ET.SubElement(document, "point", attributes)
        for (source, destination), distance in edges.items():
            original = original_paths.get((source, destination))
            attrs = (dict(original.attrib) if original is not None else
                     {"name": source + "---" + destination, "pathWidth": "100", "maxVelocity": ""})
            if original is None or any(
                    (float(original_points[name].get("xPosition")), float(original_points[name].get("yPosition")))
                    != (points[name]["x"], points[name]["y"]) for name in (source, destination)):
                attrs["length"] = str(distance)
            attrs.update(sourcePoint=source, destinationPoint=destination)
            ET.SubElement(document, "path", attrs)
        for item in self.document:
            if item.tag in ("point", "path"):
                continue
            if item.tag == "node" and any(
                    (path.get("sourcePoint"), path.get("destinationPoint")) not in edges
                    for path in item.findall("path")):
                continue
            document.append(item)

        with tempfile.NamedTemporaryFile(dir=self.file.parent, delete=False) as stream:
            temporary = Path(stream.name)
            try:
                ET.ElementTree(document).write(stream, encoding="utf-8", xml_declaration=True)
                stream.flush()
                temporary.replace(self.file)
            finally:
                try:
                    temporary.unlink()
                except FileNotFoundError:
                    pass
        self.document = document
        return self.graph()


class DefinedRoutes:
    def __init__(self, model):
        self.model = model
        self.lock = threading.Lock()
        self.ready = False
        self.state = "IDLE"
        self.message = ""
        self.nodes = []
        self.has_plan = False
        self.control = rospy.Publisher("/robot_api/nav_ctrl", NavigationControl, queue_size=4)
        self.preview = rospy.ServiceProxy("/path_server/make_plan", MakePlanService)
        self.load = rospy.ServiceProxy("/path_server/load_path", LoadPlanService)
        self.subscribers = [
            rospy.Subscriber("/robot_api/localization_ready", Bool, self.localization, queue_size=1),
            rospy.Subscriber("/robot_api/nav_ctrl_status", NavigationControlStatus, self.update_status, queue_size=8),
            rospy.Subscriber("/robot_api/shortdij_trajectory", String, self.update_sequence, queue_size=4),
        ]

    def localization(self, msg):
        self.ready = msg.data

    def update_status(self, msg):
        with self.lock:
            if self.state != "NAVIGATING":
                return
            if msg.status == NavigationControlStatus.COMPLETED:
                self.state, self.message = "SUCCEEDED", "Đã đến đích theo tuyến"
            elif msg.status == NavigationControlStatus.CANCELLED:
                self.state, self.message = "CANCELLED", "Đã dừng tuyến"
            elif msg.status < 0:
                self.state, self.message = "BLOCKED", msg.status_desc or "Reeman không chạy được tuyến"

    def update_sequence(self, msg):
        if msg.data.startswith("short_dij:"):
            with self.lock:
                if self.state == "NAVIGATING":
                    self.nodes = msg.data.partition(":")[2].split()

    def status(self):
        with self.lock:
            return self.state, self.message, list(self.nodes)

    def plan(self, name):
        if not self.ready:
            raise RuntimeError("Đang chờ Cartographer định vị robot")
        point = next((p for p in self.model.graph()["points"] if p["name"] == name), None)
        if point is None:
            raise ValueError("Đích chưa có trong tuyến đã lưu")
        message = self.preview(name)
        path = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in message.path.poses]
        if not path:
            raise ValueError("Reeman không tìm thấy tuyến hợp lệ từ vị trí hiện tại đến điểm " + name)
        return {**point, "kind": "defined-route"}, path

    def start(self, name):
        if not self.ready or not self.control.get_num_connections():
            raise RuntimeError("Điều hướng tuyến Reeman chưa sẵn sàng")
        with self.lock:
            self.state, self.message = "NAVIGATING", "Đang chạy tuyến đến " + name
            self.nodes = []
        self.has_plan = True
        self.control.publish(NavigationControl(control=NavigationControl.START, mode=NavigationControl.PLAN,
                                               goal_name=name))

    def stop(self):
        with self.lock:
            if self.state != "NAVIGATING":
                return
            self.state, self.message = "CANCELLED", "Đã dừng tuyến"
        self.control.publish(NavigationControl(control=NavigationControl.STOP, mode=NavigationControl.PLAN))

    def clear(self):
        if self.has_plan:
            self.load(0, "")
            self.has_plan = False
