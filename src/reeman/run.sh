#!/usr/bin/env bash
set -euo pipefail

project="$(cd -- "$(dirname -- "$0")" && pwd)"
harness="${REEMAN_HARNESS:-$project/harness}"
port="${REEMAN_HTTP_PORT:-8010}"
websocket_port="${REEMAN_GZ_PORT:-9002}"
device_default=0
if [[ "$port" == 8020 ]]; then device_default=1; fi
device_inputs="${REEMAN_DEVICE_INPUTS:-$device_default}"
openni_default=0
if [[ "$device_inputs" == 1 ]]; then openni_default="$device_default"; fi
openni_input="${REEMAN_OPENNI_INPUT:-$openni_default}"
direct_devices="${REEMAN_DIRECT_DEVICES:-}"
if [[ ! -f "$harness/run.sh" || ! -f "$harness/noetic.sh" ]]; then
    echo "Reeman harness not found at $harness" >&2
    exit 1
fi

# Stage the tested harness without modifying it or the backup.
compose_dir="${REEMAN_COMPOSE_DIR:-}"
stage="$(mktemp -d "${compose_dir:-/tmp}/amr-reeman-launch.XXXXXXXX")"
cp -- "$harness/"*.py "$harness/run.sh" "$harness/noetic.sh" "$stage/"
cp -- "$project/dashboard.py" "$project/dashboard.html" "$project/room_launch.py" "$stage/"
cp -- "$project/defined_routes.py" "$project/docking.py" "$project/relocation.py" "$project/obstacle_wait.py" "$stage/"
cp -- "$project/dynamic_obstacle.py" "$stage/"
if [[ "$port" == 8020 ]]; then
    cp -- "$project/room_layout.py" "$stage/"
fi
if [[ "$device_inputs" == 1 ]]; then
    cp -- "$project/sensor_input_launch.py" "$project/hins_sim.py" "$project/wit901_sim.py" "$stage/"
fi
if [[ "$openni_input" == 1 ]]; then
    [[ "$device_inputs" == 1 ]] || { echo "REEMAN_OPENNI_INPUT requires REEMAN_DEVICE_INPUTS=1" >&2; exit 2; }
    if [[ -n "$compose_dir" ]]; then
        ln -s -- "$compose_dir/openni" "$stage/openni"
    else
        backup="${REEMAN_BACKUP:-/mnt/d/Embedded_Backup/ubuntu_root_full}"
        install="$backup/home/reeman/reeman_ws/install"
        cp -- "$project/gazebo_depth.cpp" "$stage/"
        mkdir -p "$stage/openni/OpenNI2/Drivers"
        cp -L -- "$install/lib/libOpenNI2.so" "$stage/openni/"
        bwrap --ro-bind "$backup/usr" /usr --ro-bind "$backup/etc" /etc \
            --ro-bind "$install/include/astra_camera/include/openni2" /openni-headers \
            --symlink usr/bin /bin --symlink usr/lib /lib --symlink usr/lib64 /lib64 \
            --dev /dev --proc /proc --tmpfs /tmp --bind "$stage" /build --chdir /build \
            /usr/bin/g++ -std=c++17 -O2 -shared -fPIC -pthread -I/openni-headers \
            gazebo_depth.cpp -o openni/OpenNI2/Drivers/libGazeboDepth.so
    fi
    cp -- "$project/depth_device.py" "$stage/"
fi
cp -a -- "$project/../sim/visualize" "$stage/viewer"
if [[ -d "$harness/controls" ]]; then
    cp -a -- "$harness/controls" "$stage/"
fi
sed -i 's/\r$//' "$stage/run.sh" "$stage/noetic.sh"
/usr/bin/python3 - "$stage" "$websocket_port" "$device_inputs" "$openni_input" "$port" "$direct_devices" <<'PY'
from pathlib import Path
import os
import shlex
import socket
import sys

root = Path(sys.argv[1])
outer = root / "run.sh"
text = outer.read_text()
marker = 'printf \'Simulation logs: %s\\n\' "$state"'
if text.count(marker) != 1:
    raise SystemExit("Unrecognized Reeman runtime launcher")
text = text.replace(marker, marker + '\nln -s -- "$state" "$sim_dir/session"')
device_inputs = sys.argv[3] == "1"
openni_input = sys.argv[4] == "1"
hardware_cadence = device_inputs and sys.argv[5] == "8020"
direct_devices = set(sys.argv[6].split(",")) if sys.argv[6] not in ("", "bridge") else set()
if direct_devices - {"depth", "lidar", "imu"}:
    raise SystemExit("REEMAN_DIRECT_DEVICES accepts bridge or a list of depth,lidar,imu")
if direct_devices and (not hardware_cadence or not os.environ.get("REEMAN_COMPOSE_DIR")):
    raise SystemExit("Direct devices require the 8020 Compose profile")
if "depth" in direct_devices and not openni_input:
    raise SystemExit("Direct depth requires REEMAN_OPENNI_INPUT=1")

camera_demo = openni_input and sys.argv[5] == "8020"
marker = 'cp -a -- "$backup/home/reeman/data/maps" "$state/maps"'
if text.count(marker) != 1:
    raise SystemExit("Unrecognized Reeman map staging")
room_relocate = " --relocate" if sys.argv[5] == "8020" else ""
room_staging = """if [[ "$world" == room ]]; then
    if [[ ! -d "$maps" ]]; then
        mkdir -- "$maps"
        cp -- "$state/room.pgm" "$maps/default.pgm"
        sed 's|^image: .*|image: default.pgm|' "$state/room.yaml" > "$maps/default.yaml"
        printf '0 0 0\\n' > "$maps/initPose.txt"
    fi
    touch "$state/room-mode"
    /usr/bin/python3 "$sim_dir/room_launch.py" "$backup" "$state/bringup.launch"@ROOM_RELOCATE@
elif [[ ! -d "$maps" ]]; then
    cp -a -- "$backup/home/reeman/data/maps" "$maps"
fi"""
map_setup = 'maps="$state/maps"\n'
map_staging = room_staging.replace("@ROOM_RELOCATE@", room_relocate)
if sys.argv[5] == "8020":
    map_setup += """route_dir="${XDG_DATA_HOME:-$HOME/.local/share}/amr-sim/reeman-8020/$world"
if [[ "$world" == map ]]; then
    map_hash="$(sha256sum "$backup/home/reeman/data/maps/default.pgm")"
    route_dir="$route_dir/${map_hash%% *}"
fi
mkdir -p -- "$route_dir"
if [[ -n "$compose_dir" ]]; then
    ln -s -- "$route_dir" "$compose_dir/routes"
    maps="$route_dir/maps"
    if [[ ! -d "$maps" ]]; then
        map_seed="$(mktemp -d "$route_dir/.map-seed.XXXXXXXX")"
        maps="$map_seed/maps"
    fi
fi
"""
    map_staging += """
if [[ -n "$compose_dir" ]]; then
    if [[ -n "${map_seed:-}" ]]; then
        mv -T -- "$maps" "$route_dir/maps"
        rmdir -- "$map_seed"
    fi
    ln -s -- "$route_dir/maps" "$state/maps"
fi
for name in model.xml waypoint.yaml; do
    if [[ -f "$state/maps/$name" && ! -f "$route_dir/$name" ]]; then
        cp -n -- "$state/maps/$name" "$route_dir/$name"
    fi
    ln -sfn -- "/sim-routes/$name" "$state/maps/$name"
done"""
    mount = '--bind "$state/maps" /data/maps'
    if text.count(mount) != 1:
        raise SystemExit("Unrecognized Reeman map mount")
    text = text.replace(mount, '--bind "$route_dir" /sim-routes ' + mount)
text = text.replace(marker, map_setup + map_staging)
if device_inputs:
    marker = 'mkdir "$state/ros"'
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized Reeman ROS launch staging")
    command = '\n/usr/bin/python3 "$sim_dir/sensor_input_launch.py" "$backup" "$state/bringup.launch"'
    extra = '\ntouch "$state/native-slam-profile"' if hardware_cadence else ""
    if openni_input:
        command += " --openni"
    if "lidar" in direct_devices:
        address = socket.gethostbyname(socket.gethostname())
        command += " --direct-lidar-ip " + shlex.quote(address)
    if "imu" in direct_devices:
        command += " --direct-imu"
    text = text.replace(marker, marker + command + '\ntouch "$state/device-inputs"' + extra)
    bridge = root / "ros_bridge.py"
    bridge_text = bridge.read_text()
    for name, message in (("scan", "LaserScan"), ("imu", "Imu")):
        original = f'rospy.Publisher("/{name}", {message}'
        if bridge_text.count(original) != 1:
            raise SystemExit(f"Unrecognized {name} bridge publisher")
        topic = "/sim/lidar_raw" if name == "scan" else "/sim/imu_raw"
        bridge_text = bridge_text.replace(original, f'rospy.Publisher("{topic}", {message}')
    gazebo_bridge = root / "gz_bridge.py"
    gazebo_text = gazebo_bridge.read_text()
    replaced_topics = []
    if "lidar" in direct_devices:
        replaced_topics.append("/sim/scan")
    if "imu" in direct_devices:
        replaced_topics.append("/sim/imu")
    if openni_input:
        replaced_topics.extend(("/sim/rgbd/points", "/sim/rgbd/camera_info", "/sim/rgbd/image"))
        if "depth" in direct_devices:
            replaced_topics.append("/sim/rgbd/depth_image")
        else:
            bridge_text = bridge_text.replace("import rospy", "import rospy\nfrom depth_device import OpenNIDepthInput")
            bridge_text = bridge_text.replace("self.connection = connection", "self.connection = connection\n        self.openni_depth = OpenNIDepthInput()")
        marker = "    def publish(self, message, payload):\n"
        if bridge_text.count(marker) != 1:
            raise SystemExit("Unrecognized depth bridge dispatch")
        dispatch = "" if "depth" in direct_devices else (
            '        if message["kind"] == "depth":\n'
            '            self.openni_depth.write(message, payload)\n            return\n'
        )
        bridge_text = bridge_text.replace(marker, marker + dispatch +
            '        if message["kind"] not in self.handlers:\n            return\n')
    if replaced_topics:
        routes = tuple(f'("{topic}",' for topic in replaced_topics)
        lines = gazebo_text.splitlines(keepends=True)
        active_lines = [line for line in lines if not line.lstrip().startswith(routes)]
        if len(lines) - len(active_lines) != len(routes):
            raise SystemExit("Unrecognized Gazebo device routes")
        gazebo_bridge.write_text("".join(active_lines))
    disabled = set()
    if openni_input:
        disabled.update(("depth", "depth_points", "camera_info", "color"))
    if "lidar" in direct_devices:
        disabled.add("scan")
    if "imu" in direct_devices:
        disabled.add("imu")
    bridge_text = "".join(
        line for line in bridge_text.splitlines(keepends=True)
        if not any(line.lstrip().startswith(f'"{key}":') for key in disabled)
    )
    if hardware_cadence:
        marker = "scan_time=0.1,"
        if bridge_text.count(marker) != 1:
            raise SystemExit("Unrecognized LiDAR bridge period")
        bridge_text = bridge_text.replace(marker, "scan_time=1.0 / 15,")
    bridge.write_text(bridge_text)
if sys.argv[5] == "8020":
    bridge = root / "ros_bridge.py"
    bridge_text = bridge.read_text()
    # The dock emulator owns these topics on 8020. Keep the other base feedback in the bridge.
    charger = '        self.publishers["charger"].publish(PowerSystemEvent(event=PowerSystemEvent.UNPLUGGED))\n'
    ir_start = '        if self.publishers["dock_ir"].get_num_connections():\n'
    ir_end = '    def send_velocity(self, forward, yaw):'
    if bridge_text.count(charger) != 1 or bridge_text.count(ir_start) != 1 or bridge_text.count(ir_end) != 1:
        raise SystemExit("Unrecognized simulated dock feedback")
    first, end = bridge_text.index(ir_start), bridge_text.index(ir_end)
    bridge_text = (bridge_text[:first] + "\n" + bridge_text[end:]).replace(charger, "")
    bridge.write_text("".join(
        line for line in bridge_text.splitlines(keepends=True)
        if not line.lstrip().startswith(('"charger":', '"dock_ir":'))
    ))
if hardware_cadence:
    marker = '"$sim_dir/gz_bridge.py" "$state/bridge.sock"'
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized Gazebo bridge launcher")
    text = text.replace(marker, marker + ' --bulk-socket "$state/bridge-bulk.sock" --clock-hz 100')
if camera_demo:
    marker = '\n[[ -S "$state/bridge.sock" ]]'
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized Gazebo bridge setup")
    text = text.replace(marker, marker + """
if [[ "$world" == room ]]; then
    touch "$state/camera-obstacle-demo"
    /usr/bin/python3 "$sim_dir/dynamic_obstacle.py" "$state/camera-obstacle.sock" > "$state/camera-obstacle.log" 2>&1 &
    pids+=("$!")
fi""")
outer.write_text(text)
generator = root / "generate_world.py"
text = generator.read_text()
marker = '    plugin(world, "sensors", "Sensors",'
if text.count(marker) != 1:
    raise SystemExit("Unrecognized Gazebo world generator")
entry = f'    plugin(world, "websocket-server", "WebsocketServer", (("port", {int(sys.argv[2])}), ("publication_hz", 30)))\n'
if direct_devices:
    parameters = {"state_directory": os.environ["REEMAN_COMPOSE_DIR"] + "/state"}
    parameters.update({name: "true" if name in direct_devices else "false"
                       for name in ("depth", "lidar", "imu")})
    entry += f'''    direct = element(world, "plugin", filename="libreeman_direct_devices.so", name="reeman::DirectDevices")
    for key, value in {parameters!r}.items():
        element(direct, key, value)
'''
text = text.replace(marker, entry + marker)
if hardware_cadence:
    marker = "(0.05, 50, 0.001), 10, 0.0)"
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized Gazebo HINS LiDAR configuration")
    text = text.replace(marker, "(0.05, 50, 0.001), 15, 0.0)")
if camera_demo:
    marker = '    add_robot(world, spawn if spawn is not None else default_spawn)'
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized room model staging")
    text = text.replace(marker, """    if map_path is None:
        from dynamic_obstacle import add_obstacle
        add_obstacle(world, element, box)
""" + marker)
if sys.argv[5] == "8020":
    marker = "def add_demo(world):"
    if text.count(marker) != 1 or text.count("def demo_obstacles():") != 1:
        raise SystemExit("Unrecognized Gazebo room geometry")
    text = text.replace("def demo_obstacles():", "def basic_demo_obstacles():")
    text = text.replace(marker, """def demo_obstacles():
    from room_layout import ROOM_OBSTACLES
    return basic_demo_obstacles() + ROOM_OBSTACLES


""" + marker)
generator.write_text(text)
viewer = root / "viewer/app.js"
text = viewer.read_text().replace("this.world = 'amr_world';", "this.world = 'reeman_robot18';")
text = text.replace("this.objects.get('amr')", "this.objects.get('robot18')").replace("this.objects.has('amr')", "this.objects.has('robot18')")
viewer.write_text(text.replace("${host}:9002", "${host}:" + sys.argv[2]))
if sys.argv[5] == "8020":
    html = root / "viewer/index.html"
    html.write_text(html.read_text().replace("<body>", '<body data-map-default="open">', 1))
inner = root / "noetic.sh"
text = inner.read_text()
if hardware_cadence:
    marker = "/sim/ros_bridge.py /sim-state/bridge.sock"
    if text.count(marker) != 1:
        raise SystemExit("Unrecognized ROS bridge launcher")
    text = text.replace(marker, marker + " --bulk-socket /sim-state/bridge-bulk.sock")
marker = "bridge_pid=$!"
if text.count(marker) != 1:
    raise SystemExit("Unrecognized Reeman Noetic launcher")
if device_inputs:
    mode = " --direct-relay" if "lidar" in direct_devices else ""
    text = text.replace(marker, marker + f'\npython3 /sim/hins_sim.py{mode} > /sim-state/hins-device.log 2>&1 &\n'
                        'hins_pid=$!\n')
    marker_cleanup = "cleanup() {"
    text = text.replace(marker_cleanup, marker_cleanup + '\n    if [[ -n "${hins_pid:-}" ]]; then kill -TERM "$hins_pid" 2>/dev/null || true; fi')
inner.write_text(text.replace(
    marker, marker + "\npython3 /sim/dashboard.py > /sim-state/dashboard.log 2>&1 &\nmonitor_pid=$!"
))
PY

proxy_host=127.0.0.1
if [[ -n "$compose_dir" ]]; then
    ln -s -- "$stage" "$compose_dir/runtime"
    proxy_host=0.0.0.0
fi
runtime_pid=
proxy_pid=
cleanup() {
    for pid in "$runtime_pid" "$proxy_pid"; do
        [[ -z "$pid" ]] || kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
bash "$stage/run.sh" "$@" &
runtime_pid=$!
/usr/bin/python3 "$project/proxy.py" "$stage/session/dashboard.sock" "$port" "$proxy_host" &
proxy_pid=$!
wait -n "$runtime_pid" "$proxy_pid"
