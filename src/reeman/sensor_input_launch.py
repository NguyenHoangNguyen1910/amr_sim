"""Wire the original Reeman LiDAR and IMU drivers to simulated devices."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def prepare(backup, target, openni=False):
    share = backup / "home/reeman/reeman_ws/install/share"
    launch_dir = share / "yoyo_bringup/launch/includes"
    tree = ET.parse(target)
    root = tree.getroot()
    lidar = ET.parse(launch_dir / "hins_he.launch").getroot().find("node[@pkg='hins_he_driver']")
    lidar.find("param[@name='laser_ip']").set("value", "127.0.0.1")
    lidar.remove(lidar.find("param[@name='port']"))
    ET.SubElement(lidar, "param", name="laser_port", type="int", value="18090")
    # The simulated device implements range data, not hardware configuration.
    lidar.find("param[@name='change_param']").set("value", "false")
    lidar.set("launch-prefix", "python3 /sim/hins_sim.py --driver")
    lidar.set("respawn", "true")
    lidar.set("respawn_delay", "2")
    imu = ET.parse(launch_dir / "wit901_1.launch").getroot().find("node[@pkg='jy901_imu']")
    imu.set("launch-prefix", "python3 /sim/wit901_sim.py")
    root.append(lidar)
    root.append(imu)
    if openni:
        camera = ET.parse(share / "astra_camera/launch/dabai_max.launch")
        node = camera.getroot().find(".//node[@pkg='astra_camera']")
        ET.SubElement(node, "env", name="LD_LIBRARY_PATH", value="/sim/openni:$(env LD_LIBRARY_PATH)")
        camera.write(target.parent / "gazebo_astra.launch", encoding="utf-8", xml_declaration=True)
        root.append(ET.Element("include", file="/sim-state/gazebo_astra.launch"))
        for element in root.findall("node"):
            if element.get("name") in ("camera_depth_optical", "camera_color_optical"):
                root.remove(element)
    tree.write(target, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    prepare(Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3:] == ["--openni"])
