import numpy as np

from go2_piper_wbc.leg_kinematics import leg_foot_position, solve_leg_position_ik


def test_leg_ik_recovers_nominal_foot_position() -> None:
    nominal = np.array([-0.1, 0.8, -1.5], np.float32)
    target = leg_foot_position("FR", nominal)
    solution, residual = solve_leg_position_ik("FR", target, nominal + [0.05, -0.1, 0.1])
    assert residual < 5.0e-4
    np.testing.assert_allclose(leg_foot_position("FR", solution), target, atol=5.0e-4)
