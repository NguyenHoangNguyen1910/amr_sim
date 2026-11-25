"""Create the Noetic image input from the Reeman Ubuntu tar archive."""

import argparse
from copy import copy
from pathlib import Path, PurePosixPath
import tarfile


RUNTIME_PATHS = (
    "usr",
    "etc/ld.so.cache",
    "etc/alternatives",
    "opt/ros/noetic",
    "home/reeman/catkin_ws/install_isolated",
    "home/reeman/reeman_ws/install",
)
UNUSED_SYSTEM_PATHS = ("usr/lib/modules", "usr/lib/firmware", "usr/src")
REQUIRED_FILES = {
    "usr/bin/python3",
    "etc/ld.so.cache",
    "opt/ros/noetic/setup.bash",
    "home/reeman/catkin_ws/install_isolated/setup.bash",
    "home/reeman/reeman_ws/install/setup.bash",
}


def in_tree(path, roots):
    return any(path == root or path.startswith(root + "/") for root in roots)


def collect_runtime(archive, destination):
    pending = REQUIRED_FILES.copy()
    count = 0
    with tarfile.open(archive, "r:") as original, tarfile.open(
        destination, "w", copybufsize=4 * 1024 * 1024
    ) as runtime:
        for entry in original:
            path = PurePosixPath(entry.name)
            name = str(path)
            if not in_tree(name, RUNTIME_PATHS) or in_tree(name, UNUSED_SYSTEM_PATHS):
                continue
            if ".." in path.parts:
                raise ValueError(f"Unexpected path in archive: {entry.name}")
            if not (entry.isfile() or entry.isdir() or entry.issym() or entry.islnk()):
                continue
            pending.discard(name)
            data = original.extractfile(entry) if entry.isfile() else None
            if entry.sparse is not None:
                # extractfile yields the expanded bytes, so use a regular tar header.
                entry = copy(entry)
                entry.type = tarfile.REGTYPE
                entry.sparse = None
                entry.pax_headers = {key: value for key, value in entry.pax_headers.items()
                                     if not key.startswith("GNU.sparse.")}
            runtime.addfile(entry, data)
            count += 1
    if pending:
        raise ValueError("Reeman runtime files missing: " + ", ".join(sorted(pending)))
    print(f"Staged {count:,} runtime entries", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    collect_runtime(args.archive, args.destination)
