"""Verify the direct 8020 lab path in a disposable room."""
from bisect import bisect_left
from collections import deque
import json
import math
import sys
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32

URL = "http://reeman-gazebo:8020"


def request(path, body=None):
    headers = {}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    try:
        with urlopen(Request(URL + path, data=data, headers=headers), timeout=4) as reply:
            return json.load(reply)
    except HTTPError as error:
        raise RuntimeError(json.load(error)) from error


def wait_for(check, limit, reason):
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        value = check()
        if value:
            return value
        time.sleep(0.05)
    raise RuntimeError(reason)


def main():
    rospy.init_node("reeman_lab_compare", anonymous=True)
    lock = threading.Lock()
    streams = {key: deque(maxlen=1536) for key in ("clock", "odom", "scan", "imu", "camera")}

    def monitor(key, message_type):
        def receive(message):
            with lock:
                streams[key].append(message)
        topic = {"clock": "/clock", "odom": "/mobile/odom", "scan": "/scan",
                 "imu": "/imu", "camera": "/camera/scan"}[key]
        return rospy.Subscriber(topic, message_type, receive, queue_size=1)

    def snapshot():
        with lock:
            return {key: list(messages) for key, messages in streams.items()}

    def latest(key):
        with lock:
            return streams[key][-1]

    subscriptions = [monitor(key, msg) for key, msg in (
        ("clock", Clock), ("odom", Odometry), ("scan", LaserScan),
        ("imu", Imu), ("camera", LaserScan))]
    stop = rospy.Publisher("/mobile_base/commands/force_stop", Int32, queue_size=1)
    wait_for(lambda: all(snapshot().values()) and stop.get_num_connections(), 20,
             "missing scan, IMU, odom, clock, camera or force-stop subscriber")

    initial = request("/api/status")["pose"]["yaw"]
    a2 = "--a2" in sys.argv
    target = -math.pi / 2 if a2 else initial + 0.30
    last_command_time = None
    try:
        def turn():
            nonlocal last_command_time
            yaw = request("/api/status")["pose"]["yaw"]
            error = math.remainder(target - yaw, math.tau)
            if abs(error) < (0.07 if a2 else 0.05):
                return True
            velocity = math.copysign(min(0.45 if a2 else 0.2, max(0.13, abs(error))), error)
            last_command_time = time.monotonic()
            request("/api/drive", {"linear": 0, "angular": velocity})
            return False

        wait_for(turn, 28, "robot did not turn in the requested direction")
        before = snapshot()
        assert last_command_time is not None
        assert max(abs(msg.twist.twist.angular.z) for msg in before["odom"]) > 0.08
        stop.publish(Int32(data=1))
        # This must happen before Gazebo's 600 ms command watchdog would stop it.
        remaining = last_command_time + 0.6 - time.monotonic()
        wait_for(lambda: abs(latest("odom").twist.twist.angular.z) < 0.03,
                 remaining, "base did not stop before the command watchdog")
    finally:
        request("/api/drive", {"linear": 0, "angular": 0})

    samples = snapshot()
    scan_times = [msg.header.stamp.to_nsec() for msg in samples["scan"]]
    odom_times = [msg.header.stamp.to_nsec() for msg in samples["odom"]]
    assert len(scan_times) >= 12, "insufficient scan samples"
    assert all(a < b for a, b in zip(scan_times, scan_times[1:])), "scan time went backwards"
    assert all(len(scan.ranges) == 3200 and scan.header.frame_id == "laser_link"
               for scan in samples["scan"])
    assert all(imu.header.frame_id == "base_link" for imu in samples["imu"])
    overlap = [stamp for stamp in scan_times if odom_times[0] <= stamp <= odom_times[-1]]
    assert len(overlap) >= 10, "scan and odom are in different clock domains"
    nearest = []
    for stamp in overlap:
        index = bisect_left(odom_times, stamp)
        nearest.append(min(abs(stamp - t) for t in odom_times[max(0, index - 1):index + 1]))
    clock = samples["clock"][-1].clock.to_nsec()
    assert abs(clock - scan_times[-1]) < 500_000_000
    assert max(nearest) < 100_000_000, "scan has no close odom sample"
    print(f"PASS turn and force_stop; scans={len(scan_times)}; "
          f"max nearest wheel odom={max(nearest) / 1e6:.1f} ms; "
          f"last scan age={(clock - scan_times[-1]) / 1e6:.1f} ms")

    if not a2:
        return

    def near_front(scan):
        return sum(1.25 <= value <= 1.80 and
                   abs(scan.angle_min + index * scan.angle_increment) < 0.24
                   for index, value in enumerate(scan.ranges))

    time.sleep(0.35)
    samples = snapshot()
    camera_before = near_front(samples["camera"][-1])
    lidar_before = near_front(samples["scan"][-1])
    barrier = request("/api/obstacles/barriers", {"action": "add_a2", "height_cm": 12})
    slot = next(obj["id"] for obj in barrier["obstacles"] if obj.get("kind") == "a2")
    try:
        def sees_obstacle():
            camera_count = near_front(latest("camera"))
            if camera_count > camera_before + 4:
                return camera_count, near_front(latest("scan"))

        result = wait_for(sees_obstacle, 12, "camera did not detect the 12 cm obstacle")
        camera_after, lidar_after = result
        assert lidar_after <= lidar_before + 4, "2D LiDAR unexpectedly sees the low obstacle"
        print(f"PASS A2 12 cm front returns: camera {camera_before}->{camera_after}, "
              f"2D LiDAR {lidar_before}->{lidar_after}")
    finally:
        request("/api/obstacles/barriers", {"action": "remove", "id": slot})
    assert subscriptions


if __name__ == "__main__":
    main()
