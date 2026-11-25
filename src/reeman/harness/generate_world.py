#!/usr/bin/env python3
"""Generate the Robot18 Gazebo Jetty scene without writing to the backup."""

import argparse
import ast
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET


WHEEL_RADIUS = 0.0705
WHEEL_SEPARATION = 0.35
WHEEL_Z = 0.02
BODY_Z = WHEEL_RADIUS - WHEEL_Z
ROBOT_VISIBILITY = 2
SENSOR_VISIBILITY = 1
ULTRASONIC_VISIBILITY = 4
FLOOR_VISIBILITY = 0xFFFFFFFF ^ ULTRASONIC_VISIBILITY
SIDES = (("left", 0.165), ("center", 0.0), ("right", -0.165))


def element(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        if isinstance(value, (tuple, list)):
            node.text = " ".join(f"{v:.10g}" for v in value)
        else:
            node.text = str(value)
    return node


def plugin(parent, filename, class_name, settings=()):
    node = element(parent, "plugin", filename=f"gz-sim-{filename}-system",
                   name=f"gz::sim::systems::{class_name}")
    for key, value in settings:
        element(node, key, value)
    return node


def material(visual, color):
    entry = element(visual, "material")
    element(entry, "ambient", color)
    element(entry, "diffuse", color)


def shape(link, kind, name, dimensions, pose, color, robot=False):
    node = element(link, kind, name=f"{name}_visual" if kind == "visual" else name)
    element(node, "pose", pose)
    geometry = element(element(node, "geometry"), dimensions[0])
    for key, value in dimensions[1:]:
        element(geometry, key, value)
    if kind == "visual":
        if robot:
            element(node, "visibility_flags", ROBOT_VISIBILITY)
        material(node, color)
    return node


def box(link, name, size, pose, color, robot=False, collision=True, visibility=None):
    dimensions = ("box", ("size", size))
    if collision:
        shape(link, "collision", name, dimensions, pose, color, robot)
    visual = shape(link, "visual", name, dimensions, pose, color, robot)
    if visibility is not None:
        element(visual, "visibility_flags", visibility)


def static_box(world, name, size, pose, color, visibility=None):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    link = element(model, "link", name="body")
    box(link, name, size, pose, color, visibility=visibility)


def inertial(link, mass, diag, center=(0, 0, 0, 0, 0, 0)):
    entry = element(link, "inertial")
    element(entry, "pose", center)
    element(entry, "mass", mass)
    inertia = element(entry, "inertia")
    for key, value in zip(("ixx", "iyy", "izz"), diag):
        element(inertia, key, value)


def sensor(link, name, sensor_type, topic, frame, rate, pose=(0, 0, 0, 0, 0, 0)):
    entry = element(link, "sensor", name=name, type=sensor_type)
    element(entry, "pose", pose)
    element(entry, "always_on", "true")
    element(entry, "update_rate", rate)
    element(entry, "topic", topic)
    element(entry, "frame_id", frame)
    return entry


def scan_sensor(link, name, topic, frame, pose, h, v, distance, rate, noise,
                visibility=SENSOR_VISIBILITY):
    entry = sensor(link, name, "gpu_lidar", topic, frame, rate, pose)
    lidar = element(entry, "lidar")
    scan = element(lidar, "scan")
    for direction, settings in (("horizontal", h), ("vertical", v)):
        samples, minimum, maximum = settings
        axis = element(scan, direction)
        element(axis, "samples", samples)
        element(axis, "resolution", 1)
        element(axis, "min_angle", minimum)
        element(axis, "max_angle", maximum)
    limits = element(lidar, "range")
    for key, value in zip(("min", "max", "resolution"), distance):
        element(limits, key, value)
    element(lidar, "visibility_mask", visibility)
    if noise:
        noise_entry = element(lidar, "noise")
        element(noise_entry, "type", "gaussian")
        element(noise_entry, "mean", 0)
        element(noise_entry, "stddev", noise)


def add_robot(world, spawn):
    x, y, yaw = spawn
    robot = element(world, "model", name="robot18")
    element(robot, "pose", (x, y, BODY_Z + 0.001, 0, 0, yaw))
    base = element(robot, "link", name="base_link")
    inertial(base, 2.4, (0.019995, 0.019995, 0.03675), (0.01, 0, 0, 0, 0, 0))
    dark = (0.10, 0.13, 0.17, 1)
    blue = (0.13, 0.35, 0.60, 1)
    shape(base, "collision", "chassis", ("box", ("size", (0.54, 0.36, 0.36))),
          (0, 0, 0.15, 0, 0, 0), dark, True)
    shape(base, "visual", "chassis", ("box", ("size", (0.54, 0.45, 0.36))),
          (0, 0, 0.2, 0, 0, 0), dark, True)
    box(base, "laser_housing", (0.05, 0.05, 0.05), (0.22, 0, 0.22, 0, 0, 0),
        blue, robot=True, collision=False)
    box(base, "lidar3d_housing", (0.10, 0.10, 0.065), (0.04, 0, 0.39, 0, 0, 0),
        blue, robot=True, collision=False)
    box(base, "camera_housing", (0.027, 0.09, 0.025), (0.268, 0, 0.16, 0, 0, 0),
        blue, robot=True, collision=False)
    for side, y in (("left", WHEEL_SEPARATION / 2), ("right", -WHEEL_SEPARATION / 2)):
        link_name = f"wheel_{side}_link"
        link = element(robot, "link", name=link_name)
        element(link, "pose", (0, y, WHEEL_Z, -math.pi / 2, 0, 0))
        inertial(link, 0.01, (0.001, 0.001, 0.001))
        wheel = ("cylinder", ("radius", WHEEL_RADIUS), ("length", 0.0276))
        shape(link, "collision", "tire", wheel, (0, 0, 0, 0, 0, 0), dark, True)
        shape(link, "visual", "tire", wheel, (0, 0, 0, 0, 0, 0), dark, True)
        joint = element(robot, "joint", name=f"wheel_{side}_joint", type="revolute")
        element(joint, "parent", "base_link")
        element(joint, "child", link_name)
        axis = element(joint, "axis")
        element(axis, "xyz", (0, 0, 1))
    # Spherical casters preserve support while allowing lateral rolling during turns.
    for name, position in (("front", 0.17), ("back", -0.17)):
        caster = ("sphere", ("radius", 0.035))
        pose = (position, 0, 0.0351 - BODY_Z, 0, 0, 0)
        collision = shape(base, "collision", f"caster_{name}", caster, pose, dark, True)
        ode = element(element(element(collision, "surface"), "friction"), "ode")
        element(ode, "mu", 0.001)
        element(ode, "mu2", 0.001)
        shape(base, "visual", f"caster_{name}", caster, pose, dark, True)

    scan_sensor(base, "hins_he", "/sim/scan", "laser_link",
                (0.22, 0, 0.22, 0, 0, 0),
                (2401, -2 * math.pi / 3, 2 * math.pi / 3), (1, 0, 0),
                (0.05, 50, 0.001), 10, 0.0)
    scan_sensor(base, "lidar3d", "/sim/lidar3d", "lidar3d_link",
                (0.04, 0, 0.42, 0, 0, 0),
                (720, -math.pi, math.pi), (16, math.radians(-15), math.radians(15)),
                (0.12, 30, 0.01), 10, 0.01)
    for side, lateral in SIDES:
        horizontal_angle = 0.32 * (lateral / 0.165)
        # Ignore floor echoes at grazing angles while retaining the whole beam fan.
        scan_sensor(base, f"ultrasonic_{side}", f"/sim/ultrasonic/{side}",
                    f"ultrasonic_{side}_link", (0.275, lateral, 0.11, 0, 0, horizontal_angle),
                    (13, -0.26, 0.26), (9, -0.26, 0.26), (0.025, 2.5, 0.01), 10, 0.0,
                    visibility=ULTRASONIC_VISIBILITY)
        scan_sensor(base, f"cliff_{side}", f"/sim/cliff/{side}", f"cliff_{side}_link",
                    (0.27, lateral, 0.10, 0, math.pi / 2, 0),
                    (1, 0, 0), (1, 0, 0), (0.015, 0.8, 0.002), 10, 0.0)

    sensor(base, "jy901", "imu", "/sim/imu", "imu_link", 50)
    contact = sensor(base, "bumper", "contact", "/sim/bumper", "base_link", 50)
    contact_config = element(contact, "contact")
    element(contact_config, "collision", "chassis")
    element(contact_config, "topic", "/sim/bumper")

    rgbd = sensor(base, "dabai_max", "rgbd_camera", "/sim/rgbd",
                  "camera_depth_optical_frame", 10, (0.25, 0, 0.16, 0, 0, 0))
    camera = element(rgbd, "camera")
    element(camera, "horizontal_fov", math.radians(110))
    element(camera, "visibility_mask", SENSOR_VISIBILITY)
    image = element(camera, "image")
    element(image, "width", 320)
    element(image, "height", 160)
    element(image, "format", "R8G8B8")
    clip = element(camera, "clip")
    element(clip, "near", 0.2)
    element(clip, "far", 2.5)
    intrinsics = element(element(camera, "lens"), "intrinsics")
    element(intrinsics, "fx", 160 / math.tan(math.radians(55)))
    element(intrinsics, "fy", 80 / math.tan(math.radians(35.5)))
    element(intrinsics, "cx", 160)
    element(intrinsics, "cy", 80)
    element(intrinsics, "s", 0)

    plugin(robot, "diff-drive", "DiffDrive", (
        ("left_joint", "wheel_left_joint"), ("right_joint", "wheel_right_joint"),
        ("wheel_separation", WHEEL_SEPARATION), ("wheel_radius", WHEEL_RADIUS),
        ("topic", "/sim/cmd_vel"), ("odom_topic", "/sim/odom"),
        ("tf_topic", "/sim/odom_tf"), ("odom_publish_frequency", 50),
        ("frame_id", "odom"), ("child_frame_id", "base_footprint"),
    ))
    plugin(robot, "joint-state-publisher", "JointStatePublisher", (
        ("topic", "/sim/joint_states"), ("update_rate", 50),
        ("joint_name", "wheel_left_joint"), ("joint_name", "wheel_right_joint"),
    ))


def load_pgm(path):
    data = path.read_bytes()
    token = re.compile(rb"(?:\s|#[^\n]*\n)*([^\s#]+)")
    offset = 0
    header = []
    for _ in range(4):
        match = token.match(data, offset)
        if not match:
            raise ValueError(f"Invalid PGM header in {path}")
        header.append(match[1])
        offset = match.end()
    magic, cols, rows, scale = header
    cols, rows, scale = int(cols), int(rows), int(scale)
    if magic != b"P5" or not 0 < scale < 256 or not data[offset:offset + 1].isspace():
        raise ValueError(f"Expected an 8-bit P5 PGM in {path}")
    offset += 2 if data[offset:offset + 2] == b"\r\n" else 1
    raster = memoryview(data)[offset:]
    if cols < 1 or rows < 1 or len(raster) != cols * rows:
        raise ValueError(f"Unexpected PGM raster size in {path}")
    return cols, rows, scale, raster


def occupied_rectangles(cols, rows, occupied):
    active = {}
    for y in range(rows):
        runs = set()
        start = None
        for x, cell in enumerate(occupied[y * cols:(y + 1) * cols]):
            if cell and start is None:
                start = x
            elif not cell and start is not None:
                runs.add((start, x))
                start = None
        if start is not None:
            runs.add((start, cols))
        for run in sorted(active.keys() - runs):
            yield (*run, active.pop(run), y)
        for run in sorted(runs):
            active.setdefault(run, y)
    for run, start in sorted(active.items()):
        yield (*run, start, rows)


def add_map(world, yaml_path):
    # map_server's simple scalar/list YAML format needs no YAML dependency.
    metadata = dict(line.split(":", 1) for line in yaml_path.read_text().splitlines()
                    if line.strip() and not line.lstrip().startswith("#") and ":" in line)
    fields = {key.strip(): value.split("#", 1)[0].strip().strip("'\"")
              for key, value in metadata.items()}
    # The original absolute path belongs to the Focal backup. For that
    # standard map_server layout, prefer the co-located file.
    image = Path(fields["image"])
    if image.is_absolute():
        image = yaml_path.with_name(image.name)
    else:
        image = yaml_path.parent / image
    cols, rows, scale, raster = load_pgm(image)
    resolution = float(fields["resolution"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Map resolution must be positive")
    ox, oy, yaw = ast.literal_eval(fields["origin"])
    threshold = float(fields["occupied_thresh"])
    negate = bool(int(fields["negate"]))
    occupied = bytearray((pixel / scale if negate else 1 - pixel / scale) > threshold
                         for pixel in raster)

    model = element(world, "model", name="occupancy_map")
    element(model, "static", "true")
    element(model, "pose", (ox, oy, 0, 0, 0, yaw))
    link = element(model, "link", name="geometry")
    box(link, "map_floor", (cols * resolution + 6, rows * resolution + 6, 0.1),
        (cols * resolution / 2, rows * resolution / 2, -0.05, 0, 0, 0),
        (0.54, 0.55, 0.54, 1), visibility=FLOOR_VISIBILITY)
    for index, (x1, x2, y1, y2) in enumerate(occupied_rectangles(cols, rows, occupied)):
        size = ((x2 - x1) * resolution, (y2 - y1) * resolution, 1.4)
        pose = ((x1 + x2) * resolution / 2, (rows - (y1 + y2) / 2) * resolution,
                size[2] / 2, 0, 0, 0)
        box(link, f"wall_{index}", size, pose, (0.75, 0.76, 0.78, 1))

    pose_path = yaml_path.with_name("initPose.txt")
    if pose_path.is_file():
        x, y, heading = (float(value) for value in pose_path.read_text().split())
        return x, y, heading
    return ox + resolution * cols / 2, oy + resolution * rows / 2, 0


def demo_obstacles():
    wall = (0.72, 0.74, 0.78, 1)
    accent = (0.80, 0.30, 0.11, 1)
    return (
        ("wall_north", (10, 0.12, 1.4), (0, 4, 0.7), wall),
        ("wall_south", (10, 0.12, 1.4), (0, -4, 0.7), wall),
        ("wall_west", (0.12, 8, 1.4), (-5, 0, 0.7), wall),
        ("wall_east_north", (0.12, 3.5, 1.4), (5, 2.25, 0.7), wall),
        ("wall_east_south", (0.12, 3.5, 1.4), (5, -2.25, 0.7), wall),
        ("depth_target", (0.35, 0.50, 0.62), (1.45, 0, 0.31), accent),
        ("low_obstacle", (0.30, 0.65, 0.12), (1.0, 1.4, 0.06), accent),
        ("shelf", (0.43, 1.0, 0.084), (3.3, -1.5, 0.8), wall),
        ("shelf_leg_north", (0.09, 0.09, 0.76), (3.3, -1.065, 0.38), wall),
        ("shelf_leg_south", (0.09, 0.09, 0.76), (3.3, -1.935, 0.38), wall),
    )


def add_demo(world):
    floor = (0.50, 0.53, 0.54, 1)
    static_box(world, "floor", (10, 8, 0.12), (0, 0, -0.06, 0, 0, 0), floor,
               visibility=FLOOR_VISIBILITY)
    static_box(world, "catch_floor", (12, 12, 0.1), (0, 0, -0.65, 0, 0, 0), floor,
               visibility=FLOOR_VISIBILITY)
    for name, size, position, color in demo_obstacles():
        static_box(world, name, size, (*position, 0, 0, 0), color)
    return 0, 0, 0


def write_room_map(directory):
    origin_x, origin_y = -5.2, -4.2
    resolution = 0.05
    cols, rows = 208, 168
    pixels = bytearray(cols * rows)
    for row in range(rows):
        y = origin_y + (rows - row - 0.5) * resolution
        for col in range(cols):
            x = origin_x + (col + 0.5) * resolution
            if abs(x) < 5 and abs(y) < 4:
                pixels[row * cols + col] = 254
    for _, (width, length, height), (x, y, z), _ in demo_obstacles():
        if z - height / 2 > 0.5:
            continue
        col_min = math.floor((x - width / 2 - origin_x) / resolution)
        col_max = math.ceil((x + width / 2 - origin_x) / resolution)
        y_min = math.floor((y - length / 2 - origin_y) / resolution)
        y_max = math.ceil((y + length / 2 - origin_y) / resolution)
        for yi in range(max(0, y_min), min(rows, y_max)):
            row_start = (rows - 1 - yi) * cols
            for col in range(max(0, col_min), min(cols, col_max)):
                pixels[row_start + col] = 0
    (directory / "room.pgm").write_bytes(f"P5\n{cols} {rows}\n255\n".encode("ascii") + pixels)
    (directory / "room.yaml").write_text(
        f"image: room.pgm\nresolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="ascii",
    )


def build_world(map_path=None, spawn=None):
    sdf = ET.Element("sdf", version="1.12")
    world = element(sdf, "world", name="reeman_robot18")
    gui = element(world, "gui")
    teleop = element(gui, "plugin", filename="ReemanControls", name="Robot controls")
    element(element(teleop, "gz-gui"), "title", "Robot controls")
    physics = element(world, "physics", name="simulation", type="ignored")
    element(physics, "max_step_size", 0.002)
    element(physics, "real_time_factor", 1.0)
    for filename, class_name in (
        ("physics", "Physics"), ("user-commands", "UserCommands"),
        ("scene-broadcaster", "SceneBroadcaster"), ("imu", "Imu"), ("contact", "Contact"),
    ):
        plugin(world, filename, class_name)
    plugin(world, "sensors", "Sensors", (("render_engine", "ogre2"),))
    light = element(world, "light", name="ceiling", type="directional")
    element(light, "pose", (0, 0, 8, 0, 0, 0))
    element(light, "diffuse", (0.85, 0.85, 0.85, 1))
    element(light, "direction", (-0.25, 0.2, -1))
    default_spawn = add_map(world, map_path) if map_path is not None else add_demo(world)
    add_robot(world, spawn if spawn is not None else default_spawn)
    return sdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    environment = parser.add_mutually_exclusive_group()
    environment.add_argument("--map", type=Path, help="map_server YAML with an 8-bit P5 PGM")
    environment.add_argument("--room", action="store_true", help="emit a demo room map beside the SDF")
    parser.add_argument("--spawn", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    args = parser.parse_args()
    tree = ET.ElementTree(build_world(args.map, args.spawn))
    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    if args.room:
        write_room_map(args.output.parent)
    print(args.output)


if __name__ == "__main__":
    main()
