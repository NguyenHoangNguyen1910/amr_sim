"""Place and move physical obstacles in the Gazebo demo room."""

import json
import math
import multiprocessing
from pathlib import Path
import signal
import socketserver
import sys
import time


MODEL = "moving_depth_obstacle"
WORLD = "reeman_robot18"
X = 0.35
Y_MIN, Y_MAX = -1.35, 0.60
SIZE = (0.30, 0.46, 0.21)
PARKED_Z = -1.2
SPEED = 0.46
BARRIER_SIZE = (1.20, .24, .21)
BARRIER_SLOTS = range(1, 7)
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"


def barrier_name(slot):
    return f"test_barrier_{slot}"


def parked_pose(slot):
    return (-4 + 1.6 * (slot - 1), 3.3, PARKED_Z)


def add_obstacle(world, element, box):
    def add(name, size, position, color):
        model = element(world, "model", name=name)
        element(model, "pose", (*position, 0, 0, 0))
        link = element(model, "link", name=f"{name}_body")
        element(link, "gravity", "false")
        element(link, "kinematic", "true")
        box(link, name, size, (0, 0, 0, 0, 0, 0), color, visibility=1)

    add(MODEL, SIZE, (X, Y_MIN, PARKED_Z), (.65, .22, .80, 1))
    for slot in BARRIER_SLOTS:
        add(barrier_name(slot), BARRIER_SIZE, parked_pose(slot), (.89, .41, .18, 1))


def change_frame(pose, source, target):
    x, y, yaw = pose
    dx, dy = x - source[0], y - source[1]
    rotation = target[2] - source[2]
    cosine, sine = math.cos(rotation), math.sin(rotation)
    return (target[0] + cosine * dx - sine * dy,
            target[1] + sine * dx + cosine * dy, yaw + rotation)


def track_robot(position, stop):
    from gz.transport import Node
    from gz.msgs.pose_v_pb2 import Pose_V

    def receive(message):
        for model in message.pose:
            if model.name == "robot18":
                p, q = model.position, model.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
                position[:] = (time.monotonic(), p.x, p.y, yaw)
                break

    node = Node()
    if not node.subscribe(Pose_V, POSE_TOPIC, receive):
        raise RuntimeError("Không theo dõi được vị trí robot trong Gazebo")
    try:
        stop.wait()
    finally:
        node.unsubscribe(POSE_TOPIC)


class ObstacleServer(socketserver.UnixStreamServer):
    def __init__(self, path, robot_position):
        from gz.transport import Node
        from gz.msgs.boolean_pb2 import Boolean
        from gz.msgs.pose_pb2 import Pose

        self.node = Node()
        self.pose = Pose(name=MODEL)
        self.pose.position.x = X
        self.pose.orientation.w = 1
        self.boolean = Boolean
        self.started = None
        self.crossed = False
        self.error = None
        self.barriers = {}
        self.robot_position = robot_position
        super().__init__(str(path), ObstacleRequest)

    def world_robot(self):
        stamp, x, y, yaw = self.robot_position[:]
        if not stamp or time.monotonic() - stamp > .60:
            raise RuntimeError("Chưa có vị trí robot mới từ Gazebo")
        return x, y, yaw

    def apply_pose(self, pose):
        ok, reply = self.node.request(f"/world/{WORLD}/set_pose/blocking",
                                     pose, type(pose), self.boolean, 450)
        if not ok or not reply.data:
            raise RuntimeError("Gazebo chưa xác nhận vị trí vật cản thử nghiệm")

    def place(self, y, z):
        self.pose.position.y = y
        self.pose.position.z = z
        self.apply_pose(self.pose)

    def place_barrier(self, slot, x, y, z, yaw):
        pose = type(self.pose)(name=barrier_name(slot))
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.apply_pose(pose)

    def barriers_command(self, command, request):
        if command == "remove":
            slot = request["id"]
            if slot not in self.barriers:
                raise ValueError("Vật cản không còn trên bản đồ")
            self.place_barrier(slot, *parked_pose(slot), 0)
            del self.barriers[slot]
            return {"removed": slot}
        if not self.barriers and command == "list":
            return {"obstacles": [], "limit": len(BARRIER_SLOTS)}
        world_robot = self.world_robot()
        map_robot = request["robot"]
        if command == "add":
            slot = next((i for i in BARRIER_SLOTS if i not in self.barriers), None)
            if slot is None:
                raise ValueError("Đã đủ 6 vật cản, hãy gỡ một vật cản trước")
            x, y, yaw = change_frame(request["pose"], map_robot, world_robot)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            length, width, height = BARRIER_SIZE
            half_x = (abs(cosine) * length + abs(sine) * width) / 2
            half_y = (abs(sine) * length + abs(cosine) * width) / 2
            if abs(x) + half_x > 4.94 or abs(y) + half_y > 3.94:
                raise ValueError("Vật cản cần nằm trọn trong phòng mô phỏng")
            dx, dy = world_robot[0] - x, world_robot[1] - y
            along = max(0, abs(dx * cosine + dy * sine) - length / 2)
            across = max(0, abs(-dx * sine + dy * cosine) - width / 2)
            if math.hypot(along, across) < .46:
                raise ValueError("Chọn vị trí xa robot hơn để đặt vật cản")
            self.place_barrier(slot, x, y, height / 2, yaw)
            self.barriers[slot] = (x, y, yaw)
        obstacles = []
        for slot, pose in sorted(self.barriers.items()):
            x, y, yaw = change_frame(pose, world_robot, map_robot)
            obstacles.append({"id": slot, "x": x, "y": y, "yaw": yaw,
                              "length": BARRIER_SIZE[0], "width": BARRIER_SIZE[1]})
        return {"obstacles": obstacles, "limit": len(BARRIER_SLOTS)}

    def command(self, request):
        command = request["command"]
        if command in ("list", "add", "remove"):
            return self.barriers_command(command, request)
        if command not in ("on", "off", "status"):
            raise ValueError("Lệnh vật cản không hợp lệ")
        if command == "on" and self.started is None:
            self.place(Y_MIN, SIZE[2] / 2)
            self.started = time.monotonic()
            self.crossed = False
            self.error = None
        elif command == "off" and (self.started is not None or self.error is not None):
            self.place(Y_MIN, PARKED_Z)
            self.started = None
            self.error = None
        return {"active": self.started is not None, "error": self.error}

    def service_actions(self):
        if self.started is None or self.crossed:
            return
        y = min(Y_MIN + (time.monotonic() - self.started) * SPEED, Y_MAX)
        try:
            self.place(y, SIZE[2] / 2)
            self.crossed = y == Y_MAX
        except RuntimeError as error:
            self.started = None
            self.error = str(error)


class ObstacleRequest(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(1)
        try:
            request = json.loads(self.rfile.readline(2048))
            answer = self.server.command(request)
        except (ValueError, KeyError, TypeError) as error:
            answer = {"error": str(error), "invalid": True}
        except (OSError, RuntimeError) as error:
            answer = {"error": str(error)}
        try:
            self.wfile.write(json.dumps(answer).encode("utf-8") + b"\n")
        except OSError:
            pass


def stop_server(*_):
    raise SystemExit(0)


def serve(path):
    # Keep Gazebo's pose callbacks separate from its blocking service requests.
    context = multiprocessing.get_context("spawn")
    robot_position = context.Array("d", 4)
    stop = context.Event()
    tracker = context.Process(target=track_robot, args=(robot_position, stop), daemon=True)
    tracker.start()
    try:
        signal.signal(signal.SIGTERM, stop_server)
        with ObstacleServer(path, robot_position) as server:
            server.serve_forever(poll_interval=.08)
    finally:
        stop.set()
        tracker.join(2)
        path.unlink(missing_ok=True)


if __name__ == "__main__":
    serve(Path(sys.argv[1]))
