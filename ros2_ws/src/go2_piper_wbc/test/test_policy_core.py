import numpy as np
from go2_piper_wbc.policy_contract import indices_for_names, load_policy_contract
from go2_piper_wbc.policy_core import (
    ObservationHistory,
    pointing_pose_command,
    projected_gravity,
    rotation_matrix_xyzw,
    slew_pose_command,
    world_pose_to_mixed_command,
)


def test_observation_contract_is_210() -> None:
    history = ObservationHistory()
    obs = history.append((np.zeros(3), np.zeros(3), np.zeros(18), np.zeros(18),
                          np.zeros(18), np.zeros(3), np.zeros(7)))
    assert obs.shape == (210,)


def test_observation_history_repeats_first_sample_like_isaac_lab() -> None:
    history = ObservationHistory(depth=3)
    fields = tuple(np.arange(size, dtype=np.float32) + 10 * i
                   for i, size in enumerate(history.FIELD_SIZES))
    observation = history.append(fields)
    offset = 0
    for field in fields:
        size = field.size
        np.testing.assert_array_equal(
            observation[offset:offset + 3 * size], np.tile(field, 3))
        offset += 3 * size


def test_identity_gravity_points_down() -> None:
    np.testing.assert_allclose(projected_gravity(np.array([0, 0, 0, 1])), [0, 0, -1], atol=1e-6)


def test_world_pose_conversion_keeps_world_height() -> None:
    command = world_pose_to_mixed_command(
        np.array([1.5, 2.0, 0.6]), np.array([0, 0, 0, 1]),
        np.array([1.0, 2.0, 0.3]), np.array([0, 0, 0, 1]))
    np.testing.assert_allclose(command[:3], [0.5, 0.0, 0.6], atol=1e-6)
    np.testing.assert_allclose(command[3:], [1, 0, 0, 0], atol=1e-6)


def test_contract_uses_training_joint_defaults() -> None:
    contract = load_policy_contract()
    defaults = dict(zip(contract.joint_names, contract.default_angles))
    assert defaults["FR_hip_joint"] == np.float32(-0.1)
    assert defaults["FL_hip_joint"] == np.float32(0.1)
    assert defaults["RR_hip_joint"] == np.float32(-0.1)
    assert defaults["RL_hip_joint"] == np.float32(0.1)
    assert contract.action_clip == 10.0
    assert contract.physics_rate_hz / contract.policy_rate_hz == 4.0
    np.testing.assert_array_equal(
        contract.effort_limits[12:], [20.0, 20.0, 15.0, 7.0, 5.0, 5.0])


def test_named_mujoco_mapping_round_trip() -> None:
    contract = load_policy_contract()
    policy_names = list(contract.joint_names[:12])
    mujoco_names = [
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    ]
    policy_to_mujoco = indices_for_names(policy_names, mujoco_names)
    mujoco_to_policy = indices_for_names(mujoco_names, policy_names)
    sentinel = np.arange(12)
    np.testing.assert_array_equal(sentinel[policy_to_mujoco][mujoco_to_policy], sentinel)


def test_pointing_command_tool_x_axis_faces_target() -> None:
    command = pointing_pose_command(0.425, 0.05, 0.5, 0.34)
    quaternion_xyzw = np.array([command[4], command[5], command[6], command[3]])
    tool_x = rotation_matrix_xyzw(quaternion_xyzw)[:, 0]
    direction = np.array([0.425, 0.05, 0.16])
    direction /= np.linalg.norm(direction)
    np.testing.assert_allclose(tool_x, direction, atol=1e-6)


def test_pose_slew_limits_position_and_uses_shortest_quaternion_path() -> None:
    current = np.array([0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    target = np.array([0.5, 0.0, 0.5, -1.0, 0.0, 0.0, 0.0])
    result = slew_pose_command(current, target, position_step=0.02, angular_step=0.01)
    np.testing.assert_allclose(result[:3], [0.42, 0.0, 0.5], atol=1e-6)
    np.testing.assert_allclose(result[3:], [1.0, 0.0, 0.0, 0.0], atol=1e-6)
