#!/usr/bin/env python3
"""Derive a ROS 2 / gazebo_ros2_control URDF without touching the ROS 1 source."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WBC_JOINTS = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]
GRIPPER_JOINTS = ["joint7", "joint8"]
DEFAULT_CONTRACT = (
    Path(__file__).resolve().parents[2]
    / "go2_piper_wbc" / "config" / "policy_contract.yaml"
)


def _load_contract(contract_path: Path) -> dict:
    with contract_path.open("r", encoding="utf-8") as stream:
        contract = json.load(stream)
    names = [str(name) for name in contract["policy_joint_names"]]
    values = [float(value) for value in contract["default_angles"]]
    if names != WBC_JOINTS or len(values) != len(WBC_JOINTS):
        raise ValueError("policy contract joint order does not match generated ros2_control order")
    efforts = [float(value) for value in contract["effort_limits"]]
    if len(efforts) != len(WBC_JOINTS):
        raise ValueError("policy contract effort limit count does not match joint count")
    return {
        "initial_positions": dict(zip(names + GRIPPER_JOINTS, values + [0.04, -0.04])),
        "effort_limits": dict(zip(names, efforts)),
    }


def _remove_plugins_and_sensors(root: ET.Element) -> None:
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"transmission", "ros2_control"}:
                parent.remove(child)
            elif child.tag == "plugin":
                parent.remove(child)
            elif child.tag == "sensor":
                parent.remove(child)


def _remove_legacy_livox(root: ET.Element) -> None:
    # The legacy Go2Arm URDF already contains a Livox-looking visual named
    # laser_livox, but the course mapping stack expects the MID-360 TF layout
    # mid360_base_link -> livox_frame -> livox_imu_frame.  Remove the legacy
    # visual/link tree so Gazebo does not show two lidars after we add the
    # course-compatible MID-360 below.
    legacy_links = {"laser_livox", "livox_imu_link"}
    legacy_joints = {"laser_livox_joint", "livox_imu_joint"}
    for link in list(root.findall("link")):
        if link.get("name") in legacy_links:
            root.remove(link)
    for joint in list(root.findall("joint")):
        child = joint.find("child")
        parent = joint.find("parent")
        if (joint.get("name") in legacy_joints
                or (child is not None and child.get("link") in legacy_links)
                or (parent is not None and parent.get("link") in legacy_links)):
            root.remove(joint)
    for gazebo in list(root.findall("gazebo")):
        if gazebo.get("reference") in legacy_links:
            root.remove(gazebo)


def _repair_meshes(root: ET.Element) -> None:
    parent_map = {child: parent for parent in root.iter() for child in parent}
    collision_boxes = {
        "piper_l_base": "0.16 0.16 0.08",
        "Link1": "0.09 0.09 0.12",
        "Link2": "0.08 0.08 0.25",
        "Link3": "0.07 0.07 0.22",
        "Link4": "0.06 0.06 0.18",
        "Link5": "0.06 0.06 0.12",
        "Link6": "0.12 0.08 0.08",
        "Link7": "0.025 0.025 0.10",
        "Link8": "0.025 0.025 0.10",
    }
    for mesh in list(root.iter("mesh")):
        filename = mesh.get("filename", "")
        if "/go2_dae/" in filename:
            # Keep the original, complete Go2 DAE.  The similarly named MuJoCo
            # STL files are split into several material pieces (base_0..4,
            # hip_0..1, etc.); substituting only *_0 made Gazebo show fragments.
            mesh.set(
                "filename",
                f"package://go2_piper_description/meshes/go2_dae/{Path(filename).name}",
            )
        elif "piper_l_description" in filename:
            stem = Path(filename).stem
            geometry = parent_map.get(mesh)
            element = parent_map.get(geometry) if geometry is not None else None
            link = parent_map.get(element) if element is not None else None
            if (element is not None and element.tag == "collision" and link is not None
                    and link.get("name") in collision_boxes):
                geometry.remove(mesh)
                ET.SubElement(geometry, "box", {"size": collision_boxes[link.get("name")]})
                continue
            if element is not None and element.tag == "collision":
                if geometry is not None and geometry.tag == "geometry":
                    geometry.remove(mesh)
                    ET.SubElement(geometry, "box", {"size": "0.06 0.04 0.04"})
            else:
                mesh.set(
                    "filename",
                    f"package://go2_piper_description/meshes/piper_dae/{Path(filename).name}",
                )
        elif filename.startswith("file://"):
            geometry = parent_map.get(mesh)
            if geometry is not None and geometry.tag == "geometry":
                geometry.remove(mesh)
                ET.SubElement(geometry, "box", {"size": "0.09 0.025 0.025"})


def _prune_orphans_and_fix_materials(root: ET.Element) -> None:
    """The legacy export contains detached helper links and unnamed materials."""
    children: dict[str, list[str]] = {}
    joints = list(root.findall("joint"))
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and child is not None:
            children.setdefault(parent.get("link", ""), []).append(child.get("link", ""))
    reachable = {"base"}
    frontier = ["base"]
    while frontier:
        current = frontier.pop()
        for child in children.get(current, []):
            if child and child not in reachable:
                reachable.add(child)
                frontier.append(child)
    for link in list(root.findall("link")):
        if link.get("name") not in reachable:
            root.remove(link)
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or parent.get("link") not in reachable
                or child.get("link") not in reachable):
            root.remove(joint)
    material_index = 0
    for visual in root.iter("visual"):
        material = visual.find("material")
        if material is not None and not material.get("name"):
            material.set("name", f"inline_material_{material_index}")
            material_index += 1


def _repair_leg_contacts(root: ET.Element) -> None:
    """Connect each physical foot and restore the lower-calf contact shape."""
    for leg in ("FL", "FR", "RL", "RR"):
        calf_name = f"{leg}_calf"
        foot_name = f"{leg}_foot"
        calf = root.find(f"./link[@name='{calf_name}']")
        foot = root.find(f"./link[@name='{foot_name}']")
        if calf is None or foot is None:
            raise ValueError(f"legacy model is missing {calf_name} or {foot_name}")

        lower = ET.SubElement(calf, "collision", {"name": f"{leg}_lower_calf_collision"})
        ET.SubElement(lower, "origin", {"xyz": "0.02 0 -0.148", "rpy": "0 0.05 0"})
        geometry = ET.SubElement(lower, "geometry")
        ET.SubElement(geometry, "cylinder", {"length": "0.065", "radius": "0.011"})

        foot_collision = foot.find("collision")
        if foot_collision is None:
            raise ValueError(f"legacy model is missing {foot_name} collision")
        foot_collision.set("name", f"{leg}_foot_collision")

        joint = ET.SubElement(root, "joint", {"name": f"{leg}_foot_fixed", "type": "fixed"})
        ET.SubElement(joint, "origin", {"xyz": "0 0 -0.213", "rpy": "0 0 0"})
        ET.SubElement(joint, "parent", {"link": calf_name})
        ET.SubElement(joint, "child", {"link": foot_name})

        gazebo = ET.SubElement(root, "gazebo", {"reference": foot_name})
        ET.SubElement(gazebo, "mu1").text = "2.0"
        ET.SubElement(gazebo, "mu2").text = "2.0"
        ET.SubElement(gazebo, "kp").text = "1000000"
        ET.SubElement(gazebo, "kd").text = "100"
        ET.SubElement(gazebo, "maxVel").text = "0.01"
        ET.SubElement(gazebo, "minDepth").text = "0.001"


def _rotation_matrix_rpy(rpy: list[float]) -> list[list[float]]:
    """Return the inertial-frame orientation in the link frame."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _quaternion_wxyz_to_rpy(quaternion: list[float]) -> list[float]:
    w, x, y, z = quaternion
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def _align_piper_to_policy_model(root: ET.Element) -> None:
    """Match the Piper kinematics and rigid bodies used by the exported policy."""
    bodies = {
        "piper_l_base": {
            "mass": 0.1625485674036938,
            "pos": [-0.00979274765955341, 1.82905827587138e-6, 0.0410100126360189],
            "inertia": [0.000226596924525071, 0.000269444772561524,
                        0.000222318258878636, -7.33972270153965e-8,
                        2.13249977802622e-6, 8.15169009611054e-9],
        },
        "Link1": {"mass": 0.71, "pos": [0.000121505, 0.000104632, -0.00438597],
                  "quat": [0.682111, 0.730899, 0.0143111, -0.0175062],
                  "diag": [0.000489262, 0.000439887, 0.000404551]},
        "Link2": {"mass": 1.17, "pos": [0.198666, -0.0109269, 0.00142122],
                  "quat": [0.510131, 0.475585, 0.525075, 0.48773],
                  "diag": [0.0679032, 0.067745, 0.00111966]},
        "Link3": {"mass": 0.5, "pos": [-0.0202738, -0.133915, -0.000458683],
                  "quat": [0.706282, 0.705111, 0.0532202, 0.0339395],
                  "diag": [0.0138227, 0.0138032, 0.000244685]},
        "Link4": {"mass": 0.38, "pos": [-9.66636e-5, 0.000876064, -0.00496881],
                  "quat": [0.714689, -0.0948484, 0.0635223, 0.690064],
                  "diag": [0.000191586, 0.000185052, 0.000152863]},
        "Link5": {"mass": 0.383, "pos": [-4.10554e-5, -0.0566487, -0.00372058],
                  "quat": [0.709589, 0.704614, 0.00134613, -0.00132656],
                  "diag": [0.00166169, 0.00164328, 0.000185028]},
        "Link6": {"mass": 0.456991, "pos": [-0.000182345, 7.94104e-5, 0.0316214],
                  "quat": [0.999977, 6.30554e-5, 0.00678017, 0.000789386],
                  "diag": [0.000938039, 0.000723068, 0.000395388]},
        "Link7": {"mass": 0.025, "pos": [0.000651232, -0.049193, 0.00972259],
                  "quat": [0.477798, 0.572974, -0.518028, 0.418398],
                  "diag": [7.74531e-5, 7.36783e-5, 5.0886e-6]},
        "Link8": {"mass": 0.025, "pos": [0.000651232, -0.049193, 0.00972259],
                  "quat": [0.477798, 0.572974, -0.518028, 0.418398],
                  "diag": [7.74531e-5, 7.36783e-5, 5.0886e-6]},
    }
    for name, values in bodies.items():
        inertial = root.find(f"./link[@name='{name}']/inertial")
        if inertial is None:
            raise ValueError(f"missing Piper inertial for {name}")
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        origin.set("xyz", " ".join(str(value) for value in values["pos"]))
        origin.set("rpy", " ".join(str(value) for value in _quaternion_wxyz_to_rpy(
            values.get("quat", [1.0, 0.0, 0.0, 0.0]))))
        mass.set("value", str(values["mass"]))
        if "diag" in values:
            xx, yy, zz = values["diag"]
            xy = xz = yz = 0.0
        else:
            xx, yy, zz, xy, xz, yz = values["inertia"]
        for key, value in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
                              (xx, yy, zz, xy, xz, yz)):
            inertia.set(key, str(value))

    joint_models = {
        "joint1": ([0.0, 0.0, 0.123], [1.0, 0.0, 0.0, 0.0], [-2.618, 2.168]),
        "joint2": ([0.0, 0.0, 0.0], [0.0356735, -0.0356786, -0.706207, -0.706205], [0.0, 3.14]),
        "joint3": ([0.28503, 0.0, 0.0], [0.637536, 0.0, 0.0, -0.77042], [-2.967, 0.0]),
        "joint4": ([-0.021984, -0.25075, 0.0], [0.707105, 0.707108, 0.0, 0.0], [-1.745, 1.745]),
        "joint5": ([0.0, 0.0, 0.0], [0.707105, -0.707108, 0.0, 0.0], [-1.22, 1.22]),
        "joint6": ([8.8259e-5, -0.091, 0.0], [0.707105, 0.707108, 0.0, 0.0], [-2.0944, 2.0944]),
        "joint7": ([0.0, 0.0, 0.1358], [0.707105, 0.707108, 0.0, 0.0], [0.0, 0.05]),
        "joint8": ([0.0, 0.0, 0.1358], [0.707105, -0.707108, 0.0, 0.0], [-0.05, 0.0]),
    }
    for name, (position, quaternion, limits) in joint_models.items():
        joint = root.find(f"./joint[@name='{name}']")
        origin = joint.find("origin")
        origin.set("xyz", " ".join(str(value) for value in position))
        origin.set("rpy", " ".join(str(value) for value in _quaternion_wxyz_to_rpy(quaternion)))
        joint.find("axis").set("xyz", "0 0 1" if name != "joint7" else "0 0 -1")
        limit = joint.find("limit")
        limit.set("lower", str(limits[0]))
        limit.set("upper", str(limits[1]))

    tcp = ET.SubElement(root, "link", {"name": "end_effector"})
    inertial = ET.SubElement(tcp, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.1"})
    ET.SubElement(inertial, "inertia", {
        "ixx": "0.001", "ixy": "0", "ixz": "0",
        "iyy": "0.001", "iyz": "0", "izz": "0.001",
    })
    tcp_joint = ET.SubElement(root, "joint", {"name": "end_effector_fixed", "type": "fixed"})
    ET.SubElement(tcp_joint, "origin", {"xyz": "0 0 0.13", "rpy": "0 -1.57079632679 0"})
    ET.SubElement(tcp_joint, "parent", {"link": "Link6"})
    ET.SubElement(tcp_joint, "child", {"link": "end_effector"})


def _repair_gripper_contacts(root: ET.Element) -> None:
    """Create parallel finger pads in the policy model's Link6 frame.

    Link7/8 are mirrored about Link6.  Link7 local +y and Link8 local -y
    both point along Link6 +z (the gripper approach direction), while their
    prismatic motion separates them along Link6 +/-y.
    """
    for name in ("Link7", "Link8"):
        link = root.find(f"./link[@name='{name}']")
        if link is None:
            raise ValueError(f"missing gripper link {name}")
        for collision in list(link.findall("collision")):
            link.remove(collision)
        collision = ET.SubElement(link, "collision", {"name": f"{name}_finger_pad"})
        # Keep the collision pad close to the visible fingertip, but avoid a
        # large invisible swept volume.  Oversized pads made the cube move even
        # when the visual mesh still appeared clear in Gazebo.
        pad_y = "0.025" if name == "Link7" else "-0.025"
        ET.SubElement(collision, "origin", {"xyz": f"0 {pad_y} 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(collision, "geometry")
        # The visual mesh tapers to a narrow tip, but using that mesh directly
        # gives ODE only a few millimetres of effective contact after
        # tessellation.  This compact box keeps a usable contact patch while
        # reducing early contact during PREGRASP/DESCEND.
        ET.SubElement(geometry, "box", {"size": "0.015 0.050 0.035"})
        gazebo = ET.SubElement(root, "gazebo", {"reference": name})
        ET.SubElement(gazebo, "mu1").text = "20.0"
        ET.SubElement(gazebo, "mu2").text = "20.0"
        ET.SubElement(gazebo, "kp").text = "500000"
        ET.SubElement(gazebo, "kd").text = "50"
        ET.SubElement(gazebo, "maxVel").text = "0.005"
        ET.SubElement(gazebo, "minDepth").text = "0.001"


def _add_actuator_dynamics(
    root: ET.Element,
    effort_limits: dict[str, float],
    armature: float = 0.01,
    friction: float = 0.01,
    damping: float = 0.0,
) -> None:
    """Approximate Isaac Lab actuator dynamics in Gazebo Classic.

    SDFormat 1.7 has no joint-armature field.  For revolute joints, adding
    ``J*a*a^T`` to the child inertia reproduces generalized rotor inertia.
    For prismatic joints the generalized inertia has mass units, so the same
    approximation must increase child mass instead.  Treating a slider like a
    revolute joint leaves its translation numerically massless.
    """
    links = {link.get("name", ""): link for link in root.findall("link")}
    for name, effort_limit in effort_limits.items():
        joint = root.find(f"./joint[@name='{name}']")
        if joint is None:
            raise ValueError(f"missing controlled joint {name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"controlled joint {name} has no limit")
        limit.set("effort", str(effort_limit))
        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set("friction", str(friction))
        dynamics.set("damping", str(damping))

        child = joint.find("child")
        link = links.get("" if child is None else child.get("link", ""))
        inertial = None if link is None else link.find("inertial")
        inertia = None if inertial is None else inertial.find("inertia")
        if inertia is None:
            raise ValueError(f"child link of {name} has no inertia")
        if joint.get("type") == "prismatic":
            mass = inertial.find("mass")
            if mass is None:
                raise ValueError(f"child link of {name} has no mass")
            mass.set("value", f"{float(mass.get('value', '0')) + armature:.12g}")
            continue
        axis_element = joint.find("axis")
        axis = [float(value) for value in (
            "1 0 0" if axis_element is None else axis_element.get("xyz", "1 0 0")
        ).split()]
        origin = inertial.find("origin")
        rpy = [float(value) for value in (
            "0 0 0" if origin is None else origin.get("rpy", "0 0 0")
        ).split()]
        rotation = _rotation_matrix_rpy(rpy)
        # URDF inertia is expressed in the inertial frame. Rotate the joint
        # axis from the link frame using R^T before adding the rotor inertia.
        inertial_axis = [
            sum(rotation[row][column] * axis[row] for row in range(3))
            for column in range(3)
        ]
        components = (
            ("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2),
            ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2),
        )
        for key, row, column in components:
            value = float(inertia.get(key, "0"))
            value += armature * inertial_axis[row] * inertial_axis[column]
            inertia.set(key, f"{value:.12g}")


def _add_optical_frame(root: ET.Element) -> None:
    if root.find("./link[@name='d435_color_optical_frame']") is not None:
        return
    ET.SubElement(root, "link", {"name": "d435_color_optical_frame"})
    joint = ET.SubElement(root, "joint", {"name": "d435_color_optical_joint", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "-1.57079632679 0 -1.57079632679"})
    ET.SubElement(joint, "parent", {"link": "d435_camera_link"})
    ET.SubElement(joint, "child", {"link": "d435_color_optical_frame"})

    # A fixed front perception camera avoids wrist/self occlusion while the
    # Piper-mounted D435 visual geometry remains part of the robot model.
    ET.SubElement(root, "link", {"name": "perception_camera_link"})
    mount = ET.SubElement(root, "joint", {"name": "perception_camera_joint", "type": "fixed"})
    ET.SubElement(mount, "origin", {
        # Mount above the arm's safe pre-grasp envelope.  The steeper pitch
        # keeps the same tabletop workspace centred after raising the sensor.
        "xyz": "0.25 0.25 0.55", "rpy": "0 0.65 -0.75"})
    ET.SubElement(mount, "parent", {"link": "base"})
    ET.SubElement(mount, "child", {"link": "perception_camera_link"})
    ET.SubElement(root, "link", {"name": "perception_camera_optical_frame"})
    optical = ET.SubElement(root, "joint", {"name": "perception_camera_optical_joint", "type": "fixed"})
    ET.SubElement(optical, "origin", {"xyz": "0 0 0", "rpy": "-1.57079632679 0 -1.57079632679"})
    ET.SubElement(optical, "parent", {"link": "perception_camera_link"})
    ET.SubElement(optical, "child", {"link": "perception_camera_optical_frame"})

    # Livox MID-360 mapping stack.  The source Go2Arm_sim2sim model does not
    # contain this sensor, so create the full TF/visual structure here instead
    # of attaching Gazebo sensors to implicit or stale link names.
    if root.find("./link[@name='mid360_base_link']") is None:
        mount = ET.SubElement(root, "joint", {"name": "mid360_mount_joint", "type": "fixed"})
        ET.SubElement(mount, "origin", {"xyz": "0.2 0 0.12", "rpy": "0 0.785 0"})
        ET.SubElement(mount, "parent", {"link": "base"})
        ET.SubElement(mount, "child", {"link": "mid360_base_link"})

        mid360 = ET.SubElement(root, "link", {"name": "mid360_base_link"})
        inertial = ET.SubElement(mid360, "inertial")
        ET.SubElement(inertial, "mass", {"value": "0.265"})
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0.030", "rpy": "0 0 0"})
        ET.SubElement(inertial, "inertia", {
            "ixx": "0.00010", "ixy": "0", "ixz": "0",
            "iyy": "0.00010", "iyz": "0", "izz": "0.00010",
        })
        visual = ET.SubElement(mid360, "visual")
        ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh", {
            "filename": "package://livox_laser_simulation/meshes/livox_mid360_gazebo.dae"
        })
        collision = ET.SubElement(mid360, "collision")
        ET.SubElement(collision, "origin", {"xyz": "0 0 0.030", "rpy": "0 0 0"})
        collision_geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(collision_geometry, "box", {"size": "0.065 0.065 0.060"})

        scan_joint = ET.SubElement(root, "joint", {"name": "mid360_scan_joint", "type": "fixed"})
        ET.SubElement(scan_joint, "origin", {"xyz": "-0.012 0 0.047", "rpy": "0 0 0"})
        ET.SubElement(scan_joint, "parent", {"link": "mid360_base_link"})
        ET.SubElement(scan_joint, "child", {"link": "livox_frame"})
        livox = ET.SubElement(root, "link", {"name": "livox_frame"})
        livox_inertial = ET.SubElement(livox, "inertial")
        ET.SubElement(livox_inertial, "mass", {"value": "0.001"})
        ET.SubElement(livox_inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(livox_inertial, "inertia", {
            "ixx": "1e-7", "ixy": "0", "ixz": "0",
            "iyy": "1e-7", "iyz": "0", "izz": "1e-7",
        })

        imu_joint = ET.SubElement(root, "joint", {"name": "mid360_imu_joint", "type": "fixed"})
        ET.SubElement(imu_joint, "origin", {"xyz": "0.011 0.02329 -0.04412", "rpy": "0 0 0"})
        ET.SubElement(imu_joint, "parent", {"link": "livox_frame"})
        ET.SubElement(imu_joint, "child", {"link": "livox_imu_frame"})
        imu = ET.SubElement(root, "link", {"name": "livox_imu_frame"})
        imu_inertial = ET.SubElement(imu, "inertial")
        ET.SubElement(imu_inertial, "mass", {"value": "0.001"})
        ET.SubElement(imu_inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(imu_inertial, "inertia", {
            "ixx": "1e-7", "ixy": "0", "ixz": "0",
            "iyy": "1e-7", "iyz": "0", "izz": "1e-7",
        })


def _add_ros2_control(
    root: ET.Element,
    initial_positions: dict[str, float],
    demo_plugins: bool = False,
) -> None:
    control = ET.SubElement(root, "ros2_control", {"name": "Go2PiperSystem", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gazebo_ros2_control/GazeboSystem"
    for name in WBC_JOINTS + GRIPPER_JOINTS:
        joint = ET.SubElement(control, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "effort"})
        position = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(position, "param", {"name": "initial_value"}).text = str(initial_positions[name])
        ET.SubElement(joint, "state_interface", {"name": "velocity"})
        ET.SubElement(joint, "state_interface", {"name": "effort"})

    gazebo = ET.SubElement(root, "gazebo")
    plugin = ET.SubElement(gazebo, "plugin", {
        "name": "gazebo_ros2_control", "filename": "libgazebo_ros2_control.so"
    })
    ET.SubElement(plugin, "parameters").text = "/workspace/LeggedManip_Lab/ros2_ws/src/go2_piper_description/config/controllers.yaml"
    # The grasp plugin creates a real Gazebo fixed joint only after both Piper
    # fingers contact the dedicated box handle.  It never overwrites poses.
    grasp = ET.SubElement(gazebo, "plugin", {
        "name": "go2_piper_grasp", "filename": "libgo2_piper_grasp_plugin.so"
    })
    ros = ET.SubElement(grasp, "ros")
    ET.SubElement(ros, "namespace").text = "/"
    ET.SubElement(grasp, "target_model").text = "target_box"
    ET.SubElement(grasp, "target_link").text = "handle"
    ET.SubElement(grasp, "parent_link").text = "Link6"
    ET.SubElement(grasp, "left_finger").text = "Link7"
    ET.SubElement(grasp, "right_finger").text = "Link8"
    if demo_plugins:
        # This plugin deliberately overwrites base state and is only valid for
        # the ROS integration demo.  The physics model remains free-floating.
        stabilizer = ET.SubElement(gazebo, "plugin", {
            "name": "go2_piper_demo_base_stabilizer",
            "filename": "libgo2_piper_base_stabilizer.so",
        })
        ET.SubElement(stabilizer, "target_z").text = "0.28"


def _add_ros2_sensors(root: ET.Element) -> None:
    def add_mid360_ray(parent: ET.Element) -> ET.Element:
        ray = ET.SubElement(parent, "ray")
        scan = ET.SubElement(ray, "scan")
        horizontal = ET.SubElement(scan, "horizontal")
        ET.SubElement(horizontal, "samples").text = "1"
        ET.SubElement(horizontal, "resolution").text = "1"
        ET.SubElement(horizontal, "min_angle").text = "0"
        ET.SubElement(horizontal, "max_angle").text = "0"
        ray_range = ET.SubElement(ray, "range")
        ET.SubElement(ray_range, "min").text = "0.1"
        ET.SubElement(ray_range, "max").text = "70.0"
        ET.SubElement(ray_range, "resolution").text = "0.002"
        noise = ET.SubElement(ray, "noise")
        ET.SubElement(noise, "type").text = "gaussian"
        ET.SubElement(noise, "mean").text = "0.0"
        ET.SubElement(noise, "stddev").text = "0.0"
        return ray

    camera_gazebo = ET.SubElement(root, "gazebo", {"reference": "perception_camera_link"})
    sensor = ET.SubElement(camera_gazebo, "sensor", {"name": "d435_depth", "type": "depth"})
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "30"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = "1.211"
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "640"
    ET.SubElement(image, "height").text = "480"
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "5.0"
    plugin = ET.SubElement(sensor, "plugin", {"name": "d435_ros", "filename": "libgazebo_ros_camera.so"})
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = "/d435"
    ET.SubElement(plugin, "camera_name").text = "camera"
    ET.SubElement(plugin, "frame_name").text = "perception_camera_optical_frame"
    ET.SubElement(plugin, "min_depth").text = "0.1"
    ET.SubElement(plugin, "max_depth").text = "5.0"

    imu_gazebo = ET.SubElement(root, "gazebo", {"reference": "base"})
    imu_sensor = ET.SubElement(imu_gazebo, "sensor", {"name": "base_imu", "type": "imu"})
    ET.SubElement(imu_sensor, "always_on").text = "true"
    ET.SubElement(imu_sensor, "update_rate").text = "200"
    imu_plugin = ET.SubElement(imu_sensor, "plugin", {"name": "imu_ros", "filename": "libgazebo_ros_imu_sensor.so"})
    imu_ros = ET.SubElement(imu_plugin, "ros")
    ET.SubElement(imu_ros, "namespace").text = "/"
    ET.SubElement(imu_ros, "remapping").text = "~/out:=imu/data"
    ET.SubElement(imu_plugin, "frame_name").text = "base"
    ET.SubElement(imu_plugin, "initial_orientation_as_reference").text = "false"

    livox_imu_gazebo = ET.SubElement(root, "gazebo", {"reference": "livox_imu_frame"})
    livox_imu_sensor = ET.SubElement(livox_imu_gazebo, "sensor", {"name": "livox_imu", "type": "imu"})
    ET.SubElement(livox_imu_sensor, "always_on").text = "true"
    ET.SubElement(livox_imu_sensor, "update_rate").text = "200"
    livox_imu_plugin = ET.SubElement(livox_imu_sensor, "plugin", {
        "name": "livox_imu_ros", "filename": "libgazebo_ros_imu_sensor.so"})
    livox_imu_ros = ET.SubElement(livox_imu_plugin, "ros")
    ET.SubElement(livox_imu_ros, "namespace").text = "/"
    ET.SubElement(livox_imu_ros, "remapping").text = "~/out:=livox/imu"
    ET.SubElement(livox_imu_plugin, "frame_name").text = "livox_imu_frame"
    ET.SubElement(livox_imu_plugin, "initial_orientation_as_reference").text = "false"

    laser_gazebo = ET.SubElement(root, "gazebo", {"reference": "livox_frame"})
    scan_sensor = ET.SubElement(laser_gazebo, "sensor", {"name": "navigation_laser", "type": "ray"})
    ET.SubElement(scan_sensor, "pose").text = "0 0 0 0 -0.785 0"
    ET.SubElement(scan_sensor, "always_on").text = "true"
    ET.SubElement(scan_sensor, "visualize").text = "false"
    ET.SubElement(scan_sensor, "update_rate").text = "15"
    ray = ET.SubElement(scan_sensor, "ray")
    scan = ET.SubElement(ray, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = "720"
    ET.SubElement(horizontal, "resolution").text = "1"
    ET.SubElement(horizontal, "min_angle").text = "-3.14159265"
    ET.SubElement(horizontal, "max_angle").text = "3.14159265"
    scan_range = ET.SubElement(ray, "range")
    ET.SubElement(scan_range, "min").text = "0.15"
    ET.SubElement(scan_range, "max").text = "12.0"
    ET.SubElement(scan_range, "resolution").text = "0.01"
    ET.SubElement(ray, "noise", {"type": "gaussian"})
    scan_plugin = ET.SubElement(scan_sensor, "plugin", {
        "name": "navigation_laser_ros", "filename": "libgazebo_ros_ray_sensor.so"})
    scan_ros = ET.SubElement(scan_plugin, "ros")
    ET.SubElement(scan_ros, "namespace").text = "/"
    ET.SubElement(scan_ros, "remapping").text = "~/out:=scan"
    ET.SubElement(scan_plugin, "output_type").text = "sensor_msgs/LaserScan"
    ET.SubElement(scan_plugin, "frame_name").text = "livox_frame"

    cloud_sensor = ET.SubElement(laser_gazebo, "sensor", {"name": "livox_lidar", "type": "ray"})
    ET.SubElement(cloud_sensor, "pose").text = "0 0 0 0 0 0"
    ET.SubElement(cloud_sensor, "always_on").text = "true"
    ET.SubElement(cloud_sensor, "visualize").text = "false"
    ET.SubElement(cloud_sensor, "update_rate").text = "200"
    add_mid360_ray(cloud_sensor)
    cloud_plugin = ET.SubElement(cloud_sensor, "plugin", {
        "name": "gazebo_ros_lidar", "filename": "liblivox_laser_simulation.so"})
    add_mid360_ray(cloud_plugin)
    cloud_ros = ET.SubElement(cloud_plugin, "ros")
    ET.SubElement(cloud_ros, "namespace").text = "/"
    ET.SubElement(cloud_ros, "remapping").text = "~/out:=livox/lidar"
    ET.SubElement(cloud_plugin, "frame_name").text = "livox_frame"
    ET.SubElement(cloud_plugin, "point_rate").text = "200000"
    ET.SubElement(cloud_plugin, "publish_rate").text = "10"
    ET.SubElement(cloud_plugin, "downsample").text = "1"
    ET.SubElement(cloud_plugin, "noise_stddev").text = "0.02"
    ET.SubElement(cloud_plugin, "csv_file_name").text = "package://livox_laser_simulation/scan_mode/mid360.csv"

    # gazebo_ros_p3d is a ModelPlugin, so it must be attached to the model-level
    # <gazebo> element (a reference attribute is only for link/sensor plugins).
    odom_gazebo = ET.SubElement(root, "gazebo")
    odom_plugin = ET.SubElement(odom_gazebo, "plugin", {"name": "ground_truth_odom", "filename": "libgazebo_ros_p3d.so"})
    ros = ET.SubElement(odom_plugin, "ros")
    ET.SubElement(ros, "namespace").text = "/"
    ET.SubElement(ros, "remapping").text = "odom:=odom"
    ET.SubElement(odom_plugin, "body_name").text = "base"
    ET.SubElement(odom_plugin, "frame_name").text = "world"
    ET.SubElement(odom_plugin, "update_rate").text = "100"
    ET.SubElement(odom_plugin, "xyz_offset").text = "0 0 0"
    ET.SubElement(odom_plugin, "rpy_offset").text = "0 0 0"


def convert(
    input_path: Path,
    output_path: Path,
    contract_path: Path = DEFAULT_CONTRACT,
    startup_pose_path: Path | None = None,
    demo_plugins: bool = False,
) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()
    contract = _load_contract(contract_path)
    initial_positions = contract["initial_positions"]
    if startup_pose_path is not None:
        startup_pose = {
            str(name): float(value)
            for name, value in json.loads(
                startup_pose_path.read_text(encoding="utf-8")).items()
        }
        missing = [
            name for name in WBC_JOINTS + GRIPPER_JOINTS
            if name not in startup_pose
        ]
        if missing:
            raise ValueError(
                "startup pose is missing controlled joints: "
                + ", ".join(missing))
        initial_positions = {
            name: startup_pose[name] for name in WBC_JOINTS + GRIPPER_JOINTS
        }
    _remove_plugins_and_sensors(root)
    _remove_legacy_livox(root)
    _repair_meshes(root)
    _repair_leg_contacts(root)
    _prune_orphans_and_fix_materials(root)
    _align_piper_to_policy_model(root)
    _repair_gripper_contacts(root)
    _add_actuator_dynamics(root, contract["effort_limits"])
    _add_actuator_dynamics(
        root, {"joint7": 20.0, "joint8": 20.0},
        armature=0.1, friction=0.05, damping=20.0)
    _add_optical_frame(root)
    _add_ros2_control(
        root,
        initial_positions=initial_positions,
        demo_plugins=demo_plugins,
    )
    _add_ros2_sensors(root)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # Humble's spawn_entity.py parses file contents as unicode and lxml rejects
    # an encoding declaration in unicode input.
    tree.write(output_path, encoding="unicode", xml_declaration=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--startup-pose", type=Path)
    parser.add_argument(
        "--demo-plugins", action="store_true",
        help="include the non-physical demo base stabilizer",
    )
    args = parser.parse_args()
    convert(
        args.input, args.output, contract_path=args.contract,
        startup_pose_path=args.startup_pose,
        demo_plugins=args.demo_plugins,
    )


if __name__ == "__main__":
    main()
