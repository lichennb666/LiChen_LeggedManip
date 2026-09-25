import numpy as np

from go2_piper_wbc.piper_kinematics import (
    piper_tcp_position,
    piper_tcp_rotation,
    solve_piper_position_ik,
    solve_piper_pose_ik,
)


def test_zero_configuration_matches_policy_model() -> None:
    np.testing.assert_allclose(
        piper_tcp_position(np.zeros(6)), [0.184467, 0.0, 0.214647], atol=2.0e-4)


def test_tabletop_position_is_reachable() -> None:
    target = np.array([0.40, 0.0, 0.11], np.float32)
    solution, residual = solve_piper_position_ik(target, np.zeros(6), 100)
    assert residual < 1.0e-3
    np.testing.assert_allclose(piper_tcp_position(solution), target, atol=1.0e-3)


def test_pose_ik_preserves_reachable_tcp_pose() -> None:
    seed = np.array([0.0, 1.0, -0.6, 0.2, -0.25, 0.0], np.float32)
    target_position = piper_tcp_position(seed)
    target_rotation = piper_tcp_rotation(seed)
    solution, position_residual, orientation_residual = solve_piper_pose_ik(
        target_position, target_rotation, seed + 0.05, 80)
    assert position_residual < 1.0e-3
    assert orientation_residual < 0.08
    np.testing.assert_allclose(piper_tcp_position(solution), target_position, atol=1.0e-3)
