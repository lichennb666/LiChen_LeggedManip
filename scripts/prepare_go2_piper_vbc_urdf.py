#!/usr/bin/env python3
"""Create an IsaacLab-only Go2+Piper URDF for physical gripper training.

The Gazebo URDF contains ROS/Gazebo plugins, transmissions and optional sensor
links.  IsaacLab's URDF importer does not need those sections.  This script
keeps the physical robot links/joints, including Piper's prismatic joint7 and
joint8, and rewrites package:// mesh references to paths visible inside the
IsaacLab container.

It intentionally writes a new file instead of modifying the Gazebo URDF.
"""

from __future__ import annotations

import argparse
import pathlib
import xml.etree.ElementTree as ET


SENSOR_TOKENS = (
    "camera",
    "d435",
    "livox",
    "mid360",
    "radar",
    "laser",
    "imu",
    "realsense",
)


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _should_remove_link(name: str) -> bool:
    lowered = name.lower()
    return any(token in lowered for token in SENSOR_TOKENS)


def _mesh_uri(value: str, mesh_root: pathlib.Path) -> str:
    """Resolve the two ROS package layouts used by the checked-in URDF."""
    if value.startswith("package://go2_piper_description/meshes/"):
        relative = value.split("/meshes/", 1)[1]
        return (mesh_root / relative).as_uri()
    if value.startswith("package://piper_l_description/meshes/dae/"):
        relative = value.split("/meshes/dae/", 1)[1]
        return (mesh_root / "piper_dae" / relative).as_uri()
    if value.startswith("model://go2_piper_description/meshes/"):
        relative = value.split("/meshes/", 1)[1]
        return (mesh_root / relative).as_uri()
    if value.startswith("file://"):
        # Keep already-portable paths.  The bad host-only RealSense path is
        # removed together with its camera link before this function is used.
        return value
    return value


def _remove_disconnected_links(root: ET.Element, root_link: str = "base") -> set[str]:
    """Remove link elements that are not part of any joint graph.

    The checked-in Gazebo description contains foot collision helper links
    (``*_calflower``, ``*_calflower1`` and ``*_foot``) without corresponding
    URDF joints.  Gazebo can still use those as standalone collision elements,
    but Isaac's URDF importer expects one connected articulation and otherwise
    selects the first orphan as the root.  The actual leg chain already has
    its foot geometry on the connected calf links, so these disconnected
    elements are safe to omit from the Isaac-only asset.
    """
    links = {
        elem.get("name")
        for elem in root
        if _local_name(elem.tag) == "link" and elem.get("name")
    }
    referenced = set()
    for elem in root:
        if _local_name(elem.tag) != "joint":
            continue
        for child_tag in ("parent", "child"):
            child = elem.find(child_tag)
            if child is not None and child.get("link"):
                referenced.add(child.get("link"))

    disconnected = {name for name in links if name != root_link and name not in referenced}
    if not disconnected:
        return set()

    for elem in list(root):
        kind = _local_name(elem.tag)
        if kind == "link" and elem.get("name") in disconnected:
            root.remove(elem)
        elif kind == "joint":
            endpoints = {
                child.get("link")
                for child_tag in ("parent", "child")
                for child in [elem.find(child_tag)]
                if child is not None and child.get("link")
            }
            if endpoints & disconnected:
                root.remove(elem)
    return disconnected


def prepare(input_path: pathlib.Path, output_path: pathlib.Path, mesh_root: pathlib.Path) -> dict[str, int]:
    tree = ET.parse(input_path)
    root = tree.getroot()

    links = {
        elem.get("name")
        for elem in root
        if _local_name(elem.tag) == "link" and elem.get("name")
    }
    removed_links = {name for name in links if _should_remove_link(name)}

    removed_sections = 0
    for elem in list(root):
        kind = _local_name(elem.tag)
        if kind in {"gazebo", "transmission", "ros2_control"}:
            root.remove(elem)
            removed_sections += 1
            continue
        if kind == "link" and elem.get("name") in removed_links:
            root.remove(elem)
            continue
        if kind == "joint" and (
            elem.find("parent") is None
            or elem.find("child") is None
            or elem.find("parent").get("link") in removed_links
            or elem.find("child").get("link") in removed_links
        ):
            root.remove(elem)
            continue

    # The checked-in Gazebo description uses piper_l_base/Link1..Link8 while
    # the existing LeggedManipLab WBC contract uses link0/link1..link8.  Keep
    # the joint names unchanged but normalize link names so the frozen WBC and
    # the new physical asset use the same frame lookup.
    link_rename = {"piper_l_base": "link0"}
    link_rename.update({f"Link{i}": f"link{i}" for i in range(1, 9)})
    for elem in root.iter():
        if _local_name(elem.tag) == "link" and elem.get("name") in link_rename:
            elem.set("name", link_rename[elem.get("name")])
        if _local_name(elem.tag) == "joint":
            for child_tag in ("parent", "child"):
                child = elem.find(child_tag)
                if child is not None and child.get("link") in link_rename:
                    child.set("link", link_rename[child.get("link")])

    # [VBC-ISAAC-ARTICULATION] Remove standalone Gazebo collision helpers so
    # Isaac sees exactly one connected articulation rooted at ``base``.
    removed_links |= _remove_disconnected_links(root, root_link="base")

    # Add a stable EE frame at the Piper finger-slide origin.  The old Isaac
    # USD had this helper body, while the Gazebo URDF did not.  Keeping it as a
    # real fixed link makes EE rewards/observations explicit without changing
    # the six arm joints or the two finger joints.
    if not any(
        _local_name(elem.tag) == "link" and elem.get("name") == "end_effector"
        for elem in root
    ):
        ee_link = ET.Element("link", {"name": "end_effector"})
        ET.SubElement(
            ee_link,
            "inertial",
        )
        inertial = ee_link.find("inertial")
        ET.SubElement(inertial, "mass", {"value": "0.0001"})
        ET.SubElement(
            inertial,
            "inertia",
            {"ixx": "1e-8", "ixy": "0", "ixz": "0", "iyy": "1e-8", "iyz": "0", "izz": "1e-8"},
        )
        ee_joint = ET.Element("joint", {"name": "end_effector_fixed", "type": "fixed"})
        ET.SubElement(ee_joint, "origin", {"xyz": "0 0 0.13503", "rpy": "1.5708 0 0"})
        ET.SubElement(ee_joint, "parent", {"link": "link6"})
        ET.SubElement(ee_joint, "child", {"link": "end_effector"})
        root.append(ee_link)
        root.append(ee_joint)

    mesh_count = 0
    for elem in root.iter():
        if _local_name(elem.tag) == "mesh" and elem.get("filename"):
            elem.set("filename", _mesh_uri(elem.get("filename"), mesh_root))
            mesh_count += 1

    # Remove empty or non-portable Gazebo-era tags that some exporters leave
    # nested under visual/collision blocks.  Do not remove URDF inertial data.
    for parent in root.iter():
        for child in list(parent):
            if _local_name(child.tag) in {"plugin", "sensor", "material"} and _local_name(parent.tag) == "gazebo":
                parent.remove(child)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    joint_names = {
        elem.get("name")
        for elem in root
        if _local_name(elem.tag) == "joint" and elem.get("name")
    }
    required = {"joint7", "joint8"}
    missing = required - joint_names
    if missing:
        raise RuntimeError(f"prepared URDF is missing physical gripper joints: {sorted(missing)}")

    return {
        "links_removed": len(removed_links),
        "sections_removed": removed_sections,
        "mesh_paths_rewritten": mesh_count,
        "joints": len(joint_names),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--mesh-root", type=pathlib.Path, required=True)
    args = parser.parse_args()
    stats = prepare(args.input.resolve(), args.output.resolve(), args.mesh_root.resolve())
    print(
        "[VBC] prepared IsaacLab URDF: "
        f"{args.output} | removed links={stats['links_removed']} "
        f"sections={stats['sections_removed']} meshes={stats['mesh_paths_rewritten']} "
        f"joints={stats['joints']} | joint7/joint8=OK"
    )


if __name__ == "__main__":
    main()
