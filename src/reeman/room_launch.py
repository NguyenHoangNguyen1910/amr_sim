"""Prepare the original Cartographer nodes for a fresh simulated room."""

from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET


backup, target = map(Path, sys.argv[1:3])
relocate = sys.argv[3:] == ["--relocate"]
tree = ET.parse(target)
root = tree.getroot()
include = next(node for node in root.findall("include")
               if "cartographer_ros" in node.get("file", ""))
cartographer_share = backup / "home/reeman/catkin_ws/install_isolated/share/cartographer_ros"
cartographer = ET.parse(cartographer_share / "launch/robot_2d.launch").getroot()
if relocate:
    config = target.parent / "room-localization-config"
    shutil.copytree(cartographer_share / "configuration_files", config)
    lua = config / "slam_2d.lua"
    text = lua.read_text()
    marker = 'map_frame = "map"'
    if text.count(marker) != 1:
        raise ValueError("Unrecognized Cartographer map frame")
    lua.write_text(text.replace(marker, 'map_frame = "sim_local_map"'))
nodes = cartographer.findall("node")
index = list(root).index(include)
root.remove(include)
for node in nodes:
    if node.get("name") == "cartographer_node":
        args = node.attrib["args"].replace("$(arg configuration_basename)", "slam_2d.lua")
        if relocate:
            args = args.replace("$(find cartographer_ros)/configuration_files",
                                "/sim-state/room-localization-config")
        node.set("args", args.replace("-start_trajectory_with_default_topics true",
                                      "-start_trajectory_with_default_topics false"))
    else:
        ET.SubElement(node, "remap", {"from": "map", "to": "room_cartographer_grid"})
    root.insert(index, node)
    index += 1
tree.write(target, encoding="utf-8", xml_declaration=True)
