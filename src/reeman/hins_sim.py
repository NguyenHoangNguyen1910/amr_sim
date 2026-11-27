"""Feed the original HINS-HE TCP range parser from Gazebo LaserScan data."""

import math
import os
import select
import socket
import struct
import io
import subprocess
import sys
import threading
import time
from xmlrpc.client import ServerProxy

FIRST_ANGLE = 20
LAST_ANGLE = 340
POINTS_PER_DEGREE = 10
PORT = 18090
SCAN_SOCKET = '\0reeman-hins-scan-' + os.environ.get('REEMAN_ROS_PORT', '11331')
SECTOR_BEAMS = (180 - FIRST_ANGLE) * POINTS_PER_DEGREE
HEADER_BYTES = struct.calcsize('>4s6H')
INVALID_RANGE_MM = 60000
TOKEN_BITS = 12
TOKEN_MASK = (1 << TOKEN_BITS) - 1
TOKEN_FIELD_COUNT = 1 + TOKEN_BITS
TOKEN_WIRE_HIGH = 8192
TOKEN_DECODE_THRESHOLD = 1000
MAX_PENDING_SCANS = 32
TIMING_SOCKET = "/sim-state/hins-timing.sock"
DIRECT_TIMING = struct.Struct("<QQ")


def range_mm(distance):
    return (round(distance * 1000)
            if math.isfinite(distance) and 0 <= distance <= 50 else INVALID_RANGE_MM)


def decode_token(intensities):
    if len(intensities) < TOKEN_FIELD_COUNT or intensities[0] < TOKEN_DECODE_THRESHOLD:
        return None
    return sum(1 << bit for bit in range(TOKEN_BITS)
               if intensities[bit + 1] >= TOKEN_DECODE_THRESHOLD)


def scan_packets(scan, token):
    """Encode the two sectors required by the stock parser for one scan."""
    sections = ((FIRST_ANGLE, 180), (180, LAST_ANGLE))
    for start, end in sections:
        count = (end - start) * POINTS_PER_DEGREE
        tick = (scan.header.stamp.to_nsec() // 1_000_000 + start) & 0xffff
        packet = bytearray(struct.pack('>4s6H', b'HISN', start, end, count,
                                       count, count, tick))
        for index in range(count):
            angle = math.radians(start - 180 + index / POINTS_PER_DEGREE)
            beam = round((angle - scan.angle_min) / scan.angle_increment)
            distance = scan.ranges[beam] if 0 <= beam < len(scan.ranges) else math.inf
            struct_pair = struct.pack('<HH', range_mm(distance), 0)
            packet.extend(struct_pair)
        if start == FIRST_ANGLE:
            # The native parser preserves these intensity fields even for
            # invalid ranges outside the simulated LiDAR's field of view.
            struct.pack_into('<H', packet, HEADER_BYTES + 2, TOKEN_WIRE_HIGH)
            for bit in range(TOKEN_BITS):
                if token & (1 << bit):
                    offset = HEADER_BYTES + 4 * (bit + 1) + 2
                    struct.pack_into('<H', packet, offset, TOKEN_WIRE_HIGH)
        yield bytes(packet)


def main(direct=False):
    import rospy
    from sensor_msgs.msg import LaserScan

    rospy.init_node('hins_gazebo_device')
    condition = threading.Condition()
    latest_scan = None
    scan_output = rospy.Publisher('/scan', LaserScan, queue_size=1)
    pending = {}
    pending_lock = threading.Lock()
    next_token = 0

    def remember_timing(token, timing):
        with pending_lock:
            if len(pending) == MAX_PENDING_SCANS:
                del pending[next(iter(pending))]
            pending[token] = timing

    def send_scan(connection, scan):
        nonlocal next_token
        token = next_token
        next_token = (next_token + 1) & TOKEN_MASK
        packets = b''.join(scan_packets(scan, token))
        remember_timing(token, (scan.header.stamp, scan.time_increment, scan.scan_time))
        connection.sendall(packets)

    def publish_decoded(payload):
        decoded = LaserScan().deserialize(payload)
        if len(decoded.ranges) != 2 * SECTOR_BEAMS:
            return
        token = decode_token(decoded.intensities)
        if token is None:
            return
        with pending_lock:
            timing = pending.pop(token, None)
        if timing is None:
            return
        decoded.header.stamp, decoded.time_increment, decoded.scan_time = timing
        decoded.intensities = ((0.0,) * TOKEN_FIELD_COUNT
                               + tuple(decoded.intensities[TOKEN_FIELD_COUNT:]))
        scan_output.publish(decoded)

    def relay_scans(receiver):
        while not rospy.is_shutdown():
            try:
                payload = receiver.recv(65536)
            except socket.timeout:
                continue
            publish_decoded(payload)

    if direct:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as timings:
            receiver.bind(SCAN_SOCKET)
            timings.bind(TIMING_SOCKET)
            timings.setblocking(False)
            try:
                while not rospy.is_shutdown():
                    ready, _, _ = select.select([timings, receiver], [], [], 0.5)
                    if not ready:
                        continue
                    # The plugin sends timing before putting data on the TCP stream.
                    # Drain timing first when both sockets become readable.
                    while True:
                        try:
                            stamp, token = DIRECT_TIMING.unpack(timings.recv(DIRECT_TIMING.size))
                        except BlockingIOError:
                            break
                        seconds, nanos = divmod(stamp, 1_000_000_000)
                        remember_timing(token, (rospy.Time(seconds, nanos), 0.0, 1.0 / 15))
                    if receiver in ready:
                        publish_decoded(receiver.recv(65536))
            finally:
                os.unlink(TIMING_SOCKET)
        return

    def reset_pending():
        with pending_lock:
            pending.clear()

    def update(scan):
        nonlocal latest_scan
        if scan.angle_increment > 0:
            with condition:
                latest_scan = scan
                condition.notify()

    subscriber = rospy.Subscriber('/sim/lidar_raw', LaserScan, update, queue_size=1)
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as receiver:
        receiver.bind(SCAN_SOCKET)
        receiver.settimeout(1)
        relay = threading.Thread(target=relay_scans, args=(receiver,), daemon=True)
        relay.start()
        try:
            serve_scans(rospy, condition, lambda: latest_scan, send_scan, reset_pending)
        finally:
            rospy.signal_shutdown('HINS device stopped')
            relay.join()
    subscriber.unregister()


def serve_scans(rospy, condition, get_scan, send_scan, reset_pending):
    with socket.create_server(('127.0.0.1', PORT)) as server:
        server.settimeout(1)
        while not rospy.is_shutdown():
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(1)
                reset_pending()
                previous_scan = None
                try:
                    while not rospy.is_shutdown():
                        with condition:
                            condition.wait_for(lambda: get_scan() is not previous_scan,
                                               timeout=1)
                            scan = get_scan()
                        if scan is None or scan is previous_scan:
                            continue
                        # The driver can buffer full scans before publishing.
                        send_scan(connection, scan)
                        previous_scan = scan
                except (ConnectionError, socket.timeout):
                    pass


def run_driver(command):
    import rospy
    rospy.init_node('hins_driver_startup', argv=[sys.argv[0]])
    # This driver combines ROS time and wall time when calculating its first stamp.
    # A private wall-clock ROS master keeps the vendor code and Gazebo time intact.
    params = rospy.get_param('/hins_he_driver')
    master_uri = 'http://127.0.0.1:18092'
    env = dict(os.environ, ROS_MASTER_URI=master_uri)
    master_env = dict(env, ROS_PACKAGE_PATH='/opt/ros/noetic/share/rosgraph')
    with subprocess.Popen(['/opt/ros/noetic/bin/rosmaster', '--core', '-p', '18092'],
                          env=master_env) as master:
        try:
            with ServerProxy(master_uri) as rpc:
                deadline = time.monotonic() + 60
                while not rospy.is_shutdown():
                    try:
                        rpc.setParam('/hins_driver_startup', '/hins_he_driver', params)
                        break
                    except OSError:
                        if master.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('HINS wall-clock master failed to start')
                        time.sleep(0.2)
                else:
                    return 0
            with subprocess.Popen([sys.executable, __file__, '--tap'], env=env) as tap:
                with subprocess.Popen(command, env=env) as driver:
                    try:
                        while not rospy.is_shutdown() and driver.poll() is None:
                            if tap.poll() is not None:
                                raise RuntimeError('HINS scan relay exited')
                            time.sleep(0.2)
                        return driver.returncode or 0
                    finally:
                        if driver.poll() is None:
                            driver.terminate()
                        if tap.poll() is None:
                            tap.terminate()
        finally:
            if master.poll() is None:
                master.terminate()


def tap_driver_scan():
    import rospy
    from sensor_msgs.msg import LaserScan

    rospy.init_node('hins_driver_scan_tap')
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
        sender.setblocking(False)

        def forward(message):
            if message._connection_header.get('callerid') == '/hins_he_driver':
                buffer = io.BytesIO()
                message.serialize(buffer)
                try:
                    sender.sendto(buffer.getvalue(), SCAN_SOCKET)
                except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
                    pass

        subscriber = rospy.Subscriber('/scan', LaserScan, forward, queue_size=1)
        rospy.spin()
        subscriber.unregister()


if __name__ == '__main__':
    if sys.argv[1:2] == ['--driver']:
        sys.exit(run_driver(sys.argv[2:]))
    elif sys.argv[1:2] == ['--tap']:
        tap_driver_scan()
    else:
        main(direct=sys.argv[1:] == ['--direct-relay'])
