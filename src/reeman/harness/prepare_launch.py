#!/usr/bin/env python3
"""Build the simulation launch from the deployed ROS Noetic launch files."""
import argparse
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


def load(share, path):
    return ET.parse(share / path).getroot()


def matching(root, predicate):
    return next(child for child in root if predicate(child))


def make_launch(share, room):
    bringup = "yoyo_bringup/launch"
    root = load(share, f"{bringup}/yoyo_robot.launch")
    ET.SubElement(root, "param", name="/use_sim_time", value="true", type="bool")
    if room:
        mapping = matching(root, lambda e: e.get("name") == "map_server")
        mapping.set("args", "/sim-state/room.yaml")
        cartographer = matching(root, lambda e: "cartographer_ros" in e.get("file", ""))
        ET.SubElement(cartographer, "arg", name="configuration_basename", value="slam_2d.lua")
        initial_pose = matching(root, lambda e: e.get("name") == "set_initial_pose")
        root.remove(initial_pose)
    for element in list(root):
        include = element.get("file", "")
        replacement = None
        if "$(arg laser).launch" in include:
            replacement = load(share, f"{bringup}/includes/hins_he.launch")
            replacement.remove(matching(replacement, lambda e: e.get("pkg") == "hins_he_driver"))
        elif "$(arg imu).launch" in include or "secondary_serial" in include:
            replacement = ET.Element("launch")
        elif "robot_with_tf.launch" in include:
            replacement = load(share, f"{bringup}/robot_with_tf.launch")
            replacement.remove(matching(replacement, lambda e: e.get("name") == "mobile_base"))
        elif element.get("name") == "cam_3d_launcher":
            replacement = load(share, f"{bringup}/includes/pointcloud_max.launch")
            replacement.remove(matching(replacement, lambda e: "astra_camera" in e.get("file", "")))
            for child in replacement.findall("arg"):
                if child.get("name") == "model":
                    child.set("name", "camera_robot_model")
            for child in replacement.iter("param"):
                if child.get("value") == "$(arg model)":
                    child.set("value", "$(arg camera_robot_model)")
        elif "yoyo_robot_api)/launch/robot_api.launch" in include:
            replacement = load(share, "yoyo_robot_api/launch/robot_api.launch")
            replacement.remove(matching(replacement, lambda e: e.get("pkg") == "joy"))
        if replacement is not None:
            index = list(root).index(element)
            root.remove(element)
            for child in reversed(list(replacement)):
                if child.tag == "arg" and any(
                    e.get("name") == child.get("name") for e in root.findall("arg")
                ):
                    continue
                root.insert(index, deepcopy(child))
    transforms = [
        ("camera_depth_optical", "0 0 0 -1.570796326795 0 -1.570796326795",
         "camera_link", "camera_depth_optical_frame"),
        ("camera_color_optical", "0 0 0 -1.570796326795 0 -1.570796326795",
         "camera_link", "camera_color_optical_frame"),
        ("lidar3d_frame", "0.04 0 0.42 0 0 0", "base_link", "lidar3d_link"),
    ]
    for side, y, angle in (("left", .165, .32), ("center", 0.0, 0.0), ("right", -.165, -.32)):
        sonar = f"ultrasonic_{side}"
        cliff = f"cliff_{side}"
        transforms.extend([
            (f"{sonar}_frame", f"0.275 {y} 0.11 0 0 {angle}", "base_link", f"{sonar}_link"),
            (f"{cliff}_frame", f"0.27 {y} 0.10 0 1.570796326795 0", "base_link", f"{cliff}_link"),
        ])
    for name, pose, parent, frame in transforms:
        x, y, z, roll, pitch, yaw = pose.split()
        ET.SubElement(root, "node", {
            "pkg": "tf2_ros", "type": "static_transform_publisher", "name": name,
            "args": f"{x} {y} {z} {yaw} {pitch} {roll} {parent} {frame}",
        })
    return root


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("backup", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--room", action="store_true")
    args = parser.parse_args()
    share = args.backup / "home/reeman/reeman_ws/install/share"
    root = make_launch(share, args.room)
    ET.indent(root)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
