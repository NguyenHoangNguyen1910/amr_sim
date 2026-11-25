#!/bin/bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /home/reeman/catkin_ws/install_isolated/setup.bash
source /home/reeman/reeman_ws/install/setup.bash
export YOYO_MODEL=robot18 LASER_MODEL=hins_he IMU_MODEL=wit901_1 D3_MODEL=max SOCKET_MODEL=none AGV_MODEL=0

if [[ "$1" == inspect ]]; then
    shift
    exec "$@"
fi

master_pid=
launch_pid=
bridge_pid=
monitor_pid=
cleanup() {
    if [[ -n "$monitor_pid" ]]; then
        kill -TERM "$monitor_pid" 2>/dev/null || true
    fi
    if [[ -n "$bridge_pid" ]]; then
        kill -INT "$bridge_pid" 2>/dev/null || true
    fi
    if [[ -n "$launch_pid" ]]; then
        kill -INT "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
    if [[ -n "$master_pid" ]]; then
        kill -TERM "$master_pid" 2>/dev/null || true
        wait "$master_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_master() {
    python3 - "$1" "$2" <<'PYTHON'
import os
import sys
import time
from xmlrpc.client import ServerProxy

master = ServerProxy(os.environ["ROS_MASTER_URI"])
deadline = time.monotonic() + 150
while time.monotonic() < deadline:
    try:
        os.kill(int(sys.argv[1]), 0)
    except ProcessLookupError:
        sys.exit("ROS process exited. See master.log and ros.log.")
    try:
        response = (master.getPid("/sim_supervisor") if sys.argv[2] == "pid"
                    else master.getParam("/sim_supervisor", "/use_sim_time"))
        if response[0] == 1:
            break
    except OSError:
        pass
    time.sleep(0.2)
else:
    sys.exit("Timed out waiting for the ROS stack. See master.log and ros.log.")
PYTHON
}

ROS_PACKAGE_PATH=/opt/ros/noetic/share/rosgraph \
    rosmaster --core --port 11311 > /sim-state/master.log 2>&1 &
master_pid=$!
wait_for_master "$master_pid" pid
roslaunch /sim-state/bringup.launch > /sim-state/ros.log 2>&1 &
launch_pid=$!
wait_for_master "$launch_pid" param
ROS_PACKAGE_PATH=/opt/ros/noetic/share/rosgraph:/opt/ros/noetic/share/roslaunch python3 /sim/ros_bridge.py /sim-state/bridge.sock > /sim-state/ros-bridge.log 2>&1 &
bridge_pid=$!

if [[ "$1" == smoke ]]; then
    plan=()
    if [[ "$2" == room ]]; then
        plan=(--plan)
    fi
    ROS_PACKAGE_PATH=/opt/ros/noetic/share/rosgraph:/opt/ros/noetic/share/roslaunch \
        python3 /sim/smoke.py "${plan[@]}"
elif [[ "$1" == sensors ]]; then
    ROS_PACKAGE_PATH=/opt/ros/noetic/share/rosgraph:/opt/ros/noetic/share/roslaunch \
        python3 /sim/sensor_monitor.py &
    monitor_pid=$!
    wait -n "$launch_pid" "$bridge_pid" "$monitor_pid"
else
    wait -n "$launch_pid" "$bridge_pid"
fi
