#!/usr/bin/env python3
"""Convert a generated URDF to SDF and bake in the startup joint pose.

`gz sdf -p` drops the non-URDF <ros2_control> element that the
gazebo_ros2_control plugin needs, so this script re-embeds it inside the
model element.  It also writes <initial_position> into every joint axis so
the robot spawns directly in its control equilibrium (policy natural pose for
the arm, default angles for the legs) with no zero-joint startup sweep.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="source URDF")
    parser.add_argument("--output", required=True, help="generated SDF")
    parser.add_argument("--pose", required=True, help="startup_pose.json")
    args = parser.parse_args()

    result = subprocess.run(
        ["gz", "sdf", "-p", args.input], capture_output=True, text=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        return result.returncode
    sdf = result.stdout

    urdf = Path(args.input).read_text(encoding="utf-8")
    control_match = re.search(
        r"<ros2_control[^>]*>.*?</ros2_control>", urdf, re.S)
    if control_match is None:
        sys.stderr.write(f"no <ros2_control> element in {args.input}\n")
        return 1
    model_open = re.search(r"<model[^>]*>", sdf)
    if model_open is None:
        sys.stderr.write("no <model> element in converted SDF\n")
        return 1
    if "<ros2_control" not in sdf:
        sdf = sdf.replace(
            model_open.group(0), model_open.group(0) + "\n" + control_match.group(0), 1)

    root = ET.fromstring(sdf)
    pose = json.loads(Path(args.pose).read_text(encoding="utf-8"))
    for name, value in pose.items():
        joint = next((
            element for element in root.iter("joint")
            if element.get("name") == name and element.get("type") is not None
        ), None)
        if joint is None:
            sys.stderr.write(f"physical joint {name} not found in SDF\n")
            return 1
        axis = joint.find("axis")
        if axis is None:
            sys.stderr.write(f"physical joint {name} has no axis in SDF\n")
            return 1
        for existing in list(axis.findall("initial_position")):
            axis.remove(existing)
        initial = ET.Element("initial_position")
        initial.text = f"{float(value):g}"
        xyz = axis.find("xyz")
        insert_at = list(axis).index(xyz) + 1 if xyz is not None else 0
        axis.insert(insert_at, initial)

    if hasattr(ET, "indent"):
        ET.indent(root, space="  ")
    Path(args.output).write_text(
        ET.tostring(root, encoding="unicode"), encoding="utf-8")
    print(f"wrote {args.output} with {len(pose)} initial joint positions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
