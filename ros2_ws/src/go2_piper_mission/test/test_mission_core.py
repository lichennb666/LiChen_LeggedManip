import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from go2_piper_mission.mission_core import (
    grasp_retention,
    transformed_point,
    surface_point_to_object_center,
    visual_servo_tcp_target,
    waypoint_command,
    wrap_angle,
    yaw_from_quaternion,
)


def test_waypoint_command_is_in_body_frame() -> None:
    velocity, distance = waypoint_command(np.array([0, 0]), math.pi / 2, np.array([1, 0]))
    assert distance == 1.0
    np.testing.assert_allclose(velocity, [0.0, -0.1], atol=1e-6)


def test_angle_wrap() -> None:
    assert abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9


def test_yaw_from_quaternion() -> None:
    yaw = yaw_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert abs(yaw - math.pi / 2) < 1e-6


def test_transformed_finger_pad_point() -> None:
    # 90 degrees about x maps the finger's local +y axis to world +z.
    q = [math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)]
    point = transformed_point([0.4, -0.1, 0.5], q, [0.0, 0.04, 0.0])
    np.testing.assert_allclose(point, [0.4, -0.1, 0.54], atol=1e-6)


def test_visual_servo_preserves_tcp_pad_offset_and_bounds_step() -> None:
    target, error = visual_servo_tcp_target(
        [0.30, 0.0, 0.60], [0.40, 0.0, 0.55], [0.46, 0.0, 0.55], 0.02)
    np.testing.assert_allclose(error, [0.06, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(target, [0.32, 0.0, 0.60], atol=1e-6)


def test_visual_servo_uses_full_error_inside_step_limit() -> None:
    target, error = visual_servo_tcp_target(
        [0.30, 0.0, 0.60], [0.40, 0.0, 0.55], [0.405, -0.004, 0.553], 0.02)
    np.testing.assert_allclose(target, [0.305, -0.004, 0.603], atol=1e-6)
    np.testing.assert_allclose(error, [0.005, -0.004, 0.003], atol=1e-6)


def test_assisted_grasp_uses_post_close_fixed_joint_without_pose_helpers() -> None:
    source = (Path(__file__).parents[1] / "go2_piper_mission" / "mission_server.py").read_text()
    assert "def _reset_cube_pose" in source
    assert "def _publish_virtual_transport_cube" in source
    assert source.count("set_entity_state.call_async") == 2
    assert "reset_cube_on_start" in source
    assert "virtualize_physical_transport" in source
    assert "assisted_grasp" in source
    assert "double-contact fixed joint engaged after close" in source
    assert "/go2_piper/grasp/stabilize" not in source
    assert "/go2_piper/grasp/lift" not in source
    assert "if self._physical_grasp_enabled():" in source

    plugin = (Path(__file__).parents[2] / "go2_piper_gazebo_plugins" /
              "src" / "grasp_plugin.cpp").read_text()
    assert 'CreateJoint("fixed", robot_)' in plugin
    assert "left_contact || !right_contact" in plugin
    assert "SetWorldPose" not in plugin


def test_target_box_has_green_handle_link() -> None:
    model_path = (Path(__file__).parents[2] / "go2_piper_bringup" /
                  "models" / "target_box.sdf")
    root = ET.parse(model_path).getroot()
    model = root.find("model")
    assert model is not None and model.get("name") == "target_box"
    assert model.find("link[@name='body']") is not None
    handle = model.find("link[@name='handle']")
    assert handle is not None
    assert handle.find("collision[@name='grip_collision']") is not None
    assert model.find("joint[@name='body_to_handle']") is not None


def test_surface_depth_compensation_follows_camera_ray() -> None:
    center = surface_point_to_object_center([1.0, 1.0, 0.0], [0.0, 0.0, 0.0], 0.1)
    expected = 1.0 + 0.1 / math.sqrt(2.0)
    np.testing.assert_allclose(center, [expected, expected, 0.0], atol=1e-6)


def test_grasp_retention_accepts_gait_height_bob() -> None:
    retained, offset, aperture = grasp_retention(
        [0.40, 0.05, 0.52], [0.39, 0.05, 0.56], [0.032, -0.032],
        maximum_offset=0.09, minimum_aperture=0.04, minimum_height=0.10)
    assert retained
    assert offset < 0.05
    assert aperture > 0.06


def test_grasp_retention_rejects_closed_empty_gripper() -> None:
    retained, _, aperture = grasp_retention(
        [0.40, 0.05, 0.53], [0.40, 0.05, 0.56], [0.003, -0.002],
        maximum_offset=0.09, minimum_aperture=0.04, minimum_height=0.10)
    assert not retained
    assert aperture < 0.01


def test_grasp_retention_rejects_object_left_behind() -> None:
    retained, offset, _ = grasp_retention(
        [0.40, 0.05, 0.53], [0.55, 0.05, 0.61], [0.032, -0.032],
        maximum_offset=0.09, minimum_aperture=0.04, minimum_height=0.10)
    assert not retained
    assert offset > 0.09
