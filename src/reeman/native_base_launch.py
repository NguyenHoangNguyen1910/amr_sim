"""Stage the deployed Reeman ROS launch with a different base serial endpoint."""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def matching(elements, predicate, description):
    found = [element for element in elements if predicate(element)]
    if len(found) != 1:
        raise ValueError("Expected exactly one {} in the original launch".format(description))
    return found[0]


def stage_launch(source, destination, base_port):
    main = ET.parse(source / "yoyo_robot.launch")
    robot = ET.parse(source / "robot_with_tf.launch")
    include = matching(
        main.getroot().findall("include"),
        lambda element: element.get("file", "").endswith("/robot_with_tf.launch"),
        "robot_with_tf include",
    )
    mobile_base = matching(
        robot.getroot().findall("node"),
        lambda element: element.get("name") == "mobile_base"
        and "yoyo_node/YoYoNodelet" in element.get("args", ""),
        "mobile_base nodelet",
    )
    if mobile_base.find("param[@name='device_port']") is not None:
        raise ValueError("Original mobile_base launch already overrides device_port")
    params = matching(mobile_base.findall("rosparam"),
                      lambda element: "driver_params_" in element.get("file", ""),
                      "mobile_base parameters")
    mobile_base.insert(
        list(mobile_base).index(params) + 1,
        ET.Element("param", {"name": "device_port", "type": "str", "value": base_port}),
    )
    # Resolve the sibling launch relative to the copy on the Noetic host.
    include.set("file", "$(dirname)/robot_with_tf_base_port.launch")
    destination.mkdir(parents=True, exist_ok=True)
    robot.write(destination / "robot_with_tf_base_port.launch", encoding="utf-8", xml_declaration=True)
    output = destination / "yoyo_robot_native.launch"
    main.write(output, encoding="utf-8", xml_declaration=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original", required=True, type=Path, help="Original yoyo_bringup/launch directory")
    parser.add_argument("--output", required=True, type=Path, help="Staging directory")
    parser.add_argument("--base-port", required=True, help="Serial device or PTY from a compatible base controller")
    args = parser.parse_args()
    if not args.base_port.startswith("/dev/"):
        parser.error("--base-port must be an absolute Linux device path under /dev")
    try:
        print(stage_launch(args.original, args.output, args.base_port))
    except (OSError, ValueError, ET.ParseError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    main()
