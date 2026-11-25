#!/bin/bash
set -euo pipefail

ready=/run/reeman-noetic-ready
rm -f -- "$ready"
previous="$(readlink /reeman-run/current || true)"
if [[ "$previous" == /reeman-run/session.* && "${previous#/reeman-run/}" != */* && -d "$previous" ]]; then
    find "$previous" -depth -delete
fi
session="$(mktemp -d /reeman-run/session.XXXXXXXX)"
launch_pid=
cleanup() {
    rm -f -- "$ready"
    if [[ -n "$launch_pid" ]]; then
        kill -TERM "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

install=/home/reeman/reeman_ws/install
for setup in /opt/ros/noetic/setup.bash /home/reeman/catkin_ws/install_isolated/setup.bash "$install/setup.bash"; do
    if [[ ! -r "$setup" ]]; then
        printf 'Missing Reeman runtime file %s\n' "$setup" >&2
        exit 1
    fi
done
mkdir -p -- "$session/openni/OpenNI2/Drivers"
cp -L -- "$install/lib/libOpenNI2.so" "$session/openni/"
/usr/bin/g++ -std=c++17 -O2 -shared -fPIC -pthread \
    -I"$install/include/astra_camera/include/openni2" \
    /gazebo_depth.cpp -o "$session/openni/OpenNI2/Drivers/libGazeboDepth.so"

ln -sfnT -- "$session" /reeman-run/current
touch -- "$ready"
deadline=$((SECONDS + 150))
until [[ -S "$session/state/bridge.sock" && -S "$session/state/bridge-bulk.sock" ]]; do
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for the Gazebo ROS bridge" >&2
        exit 1
    fi
    sleep 0.2
done

ln -sfnT -- "$session/runtime" /sim
ln -sfnT -- "$session/state" /sim-state
ln -sfnT -- "$session/routes" /sim-routes
mkdir -p -- /data
for entry in /backup-data/*; do
    name="${entry##*/}"
    if [[ "$name" != maps ]]; then
        ln -sfnT -- "$entry" "/data/$name"
    fi
done
ln -sfnT -- /sim-state/maps /data/maps
ln -sfnT -- /data /home/reeman/data

export HOME=/home/reeman ROS_HOME=/sim-state/ros PYTHONUNBUFFERED=1 LANG=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
cd /tmp
/bin/bash /sim/noetic.sh run "${REEMAN_WORLD:-room}" &
launch_pid=$!
wait "$launch_pid"
