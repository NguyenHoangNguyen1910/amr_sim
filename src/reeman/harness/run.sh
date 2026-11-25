#!/usr/bin/env bash
set -euo pipefail
sim_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
backup="${REEMAN_BACKUP:-/mnt/d/Embedded_Backup/ubuntu_root_full}"
runtime="${REEMAN_RUNTIME:-$backup}"
cache="${XDG_CACHE_HOME:-$HOME/.cache}/reeman-noetic"
if [[ "$backup" == /mnt/d/Embedded_Backup/ubuntu_root_full && -z "${REEMAN_RUNTIME:-}" && -f "$cache/.ready" ]]; then
    runtime="$cache"
fi
system_root="$backup"
if [[ -x "$runtime/usr/bin/python3" ]]; then
    system_root="$runtime"
fi
mode=run
world=map
gui=0
for option in "$@"; do
    case "$option" in
        --smoke) mode=smoke ;;
        --sensors) mode=sensors ;;
        --gui) gui=1 ;;
        --room) world=room ;;
        --map) world=map ;;
        *) echo "Unknown option: $option" >&2; exit 2 ;;
    esac
done
if [[ ! -f "$backup/opt/ros/noetic/setup.bash" ]]; then
    echo "ROS Noetic backup missing at $backup" >&2
    exit 1
fi
/usr/bin/python3 -c 'import gz.transport, gz.msgs'
if (( gui )); then
    controls_source="$sim_dir/controls"
    controls_build="${XDG_CACHE_HOME:-$HOME/.cache}/reeman-sim/controls"
    controls_library="$controls_build/libReemanControls.so"
    if [[ ! -f "$controls_library" || "$controls_source/CMakeLists.txt" -nt "$controls_library" ||
          "$controls_source/ReemanControls.cc" -nt "$controls_library" ||
          "$controls_source/ReemanControls.qml" -nt "$controls_library" ]]; then
        PATH=/usr/bin:/bin cmake -S "$controls_source" -B "$controls_build" -DCMAKE_BUILD_TYPE=Release
        PATH=/usr/bin:/bin cmake --build "$controls_build" --parallel 2
    fi
    export GZ_GUI_PLUGIN_PATH="$controls_build${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
fi
compose_dir="${REEMAN_COMPOSE_DIR:-}"
if [[ -n "$compose_dir" ]]; then
    state="$compose_dir/state"
    mkdir -- "$state"
else
    state="$(mktemp -d /tmp/reeman-sim.XXXXXXXX)"
fi
printf 'Simulation logs: %s\n' "$state"
export GZ_PARTITION="reeman-$(basename "${compose_dir:-$state}")"
export GZ_IP=127.0.0.1
export GZ_HOMEDIR="$state"
pub_args=()
launch_args=()
if [[ "$world" == map ]]; then
    pub_args=(--map "$backup/home/reeman/data/maps/default.yaml")
else
    pub_args=(--room)
    launch_args=(--room)
fi
/usr/bin/python3 "$sim_dir/generate_world.py" --output "$state/world.sdf" "${pub_args[@]}"
/usr/bin/python3 "$sim_dir/prepare_launch.py" "$backup" "$state/bringup.launch" "${launch_args[@]}"
mkdir "$state/ros"
cp -a -- "$backup/home/reeman/data/maps" "$state/maps"
cleanup() {
    if [[ -n "$gazebo_pid" ]]; then
        kill -INT -- "-$gazebo_pid" 2>/dev/null || true
    fi
    for pid in "$@"; do
        if [[ "$pid" != "$gazebo_pid" ]]; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    if [[ -n "$gazebo_pid" ]]; then
        for attempt in {1..50}; do
            kill -0 -- "-$gazebo_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-$gazebo_pid" 2>/dev/null || true
        for attempt in {1..20}; do
            kill -0 -- "-$gazebo_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL -- "-$gazebo_pid" 2>/dev/null || true
    fi
    wait "$@" 2>/dev/null || true
}
pids=()
gazebo_pid=
trap 'cleanup "${pids[@]}"' EXIT
trap 'exit 130' INT TERM
"${REEMAN_GAZEBO_PYTHON:-/usr/bin/python3}" -u "$sim_dir/gz_bridge.py" "$state/bridge.sock" > "$state/gz-bridge.log" 2>&1 &
pids+=("$!")
for attempt in {1..100}; do
    if ! kill -0 "${pids[0]}" 2>/dev/null; then
        cat -- "$state/gz-bridge.log" >&2
        exit 1
    fi
    if [[ -S "$state/bridge.sock" ]]; then
        break
    fi
    sleep 0.1
done
[[ -S "$state/bridge.sock" ]]
if [[ -z "$compose_dir" ]]; then
    bwrap --die-with-parent --unshare-net --unshare-pid --new-session \
        --ro-bind "$system_root/usr" /usr --ro-bind "$backup/opt" /opt \
        --ro-bind "$runtime/opt/ros/noetic" /opt/ros/noetic \
        --ro-bind "$backup/etc" /etc --ro-bind "$backup/home" /home \
        --ro-bind "$runtime/home/reeman/reeman_ws/install" /home/reeman/reeman_ws/install \
        --ro-bind "$runtime/home/reeman/catkin_ws/install_isolated" /home/reeman/catkin_ws/install_isolated \
        --ro-bind "$backup/var" /var --symlink usr/bin /bin \
        --symlink usr/lib /lib --symlink usr/lib64 /lib64 \
        --dev /dev --proc /proc --tmpfs /tmp \
        --ro-bind "$backup/home/reeman/data" /data \
        --bind "$state/maps" /data/maps --bind "$state/maps" /home/reeman/data/maps \
        --ro-bind "$sim_dir" /sim --bind "$state" /sim-state \
        --clearenv --setenv PATH /usr/local/bin:/usr/bin:/bin \
        --setenv HOME /home/reeman --setenv ROS_HOME /sim-state/ros \
        --setenv ROS_MASTER_URI http://127.0.0.1:11311 --setenv ROS_IP 127.0.0.1 \
        --setenv PYTHONUNBUFFERED 1 --setenv LANG C.UTF-8 --chdir /tmp \
        -- /bin/bash /sim/noetic.sh "$mode" "$world" &
    pids+=("$!")
fi
for attempt in {1..1500}; do
    kill -0 "${pids[@]}"
    if grep -Fq 'ROS sensor bridge connected' "$state/gz-bridge.log"; then
        break
    fi
    sleep 0.2
done
grep -Fq 'ROS sensor bridge connected' "$state/gz-bridge.log"
if [[ -n "$compose_dir" ]]; then
    setsid "$GZ_SIM_COMMAND" -r --headless-rendering -v 3 "$state/world.sdf" > "$state/gazebo.log" 2>&1 &
elif (( gui )); then
    setsid gz sim -r -v 3 "$state/world.sdf" > "$state/gazebo.log" 2>&1 &
else
    setsid gz sim -r -s --headless-rendering -v 3 "$state/world.sdf" > "$state/gazebo.log" 2>&1 &
fi
gazebo_pid=$!
pids+=("$gazebo_pid")
wait -n "${pids[@]}"
