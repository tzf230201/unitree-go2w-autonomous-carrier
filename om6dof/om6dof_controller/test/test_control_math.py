import math

import numpy as np
import pytest

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    clamp_positions,
    integrate_cylindrical_position,
    next_motion_mode,
    normalize_operation_mode,
    rotation_error,
    rotation_from_rotvec,
    step_toward,
    validated_control_command,
)


def test_operation_mode_aliases_and_cycle():
    assert normalize_operation_mode("auto") == MODE_AUTONOMOUS
    assert normalize_operation_mode("joint") == MODE_JOINT
    assert normalize_operation_mode("ik") == MODE_CARTESIAN
    assert normalize_operation_mode("silinder") == MODE_CYLINDRICAL
    assert next_motion_mode(MODE_JOINT) == MODE_CARTESIAN
    assert next_motion_mode(MODE_CARTESIAN) == MODE_CYLINDRICAL
    assert next_motion_mode(MODE_CYLINDRICAL) == MODE_JOINT
    with pytest.raises(ValueError):
        normalize_operation_mode("twist")


@pytest.mark.parametrize(
    "values",
    ([0.0] * 5, [0.0] * 7, [0.0, 0.0, math.nan, 0.0, 0.0, 0.0]),
)
def test_control_command_requires_six_finite_values(values):
    with pytest.raises(ValueError):
        validated_control_command(values)
    assert validated_control_command([0.0] * 6).shape == (6,)


def test_joint_clamp_and_pose_step():
    assert clamp_positions([-2, 0, 2], [-1, -1, -1], [1, 1, 1]) == [
        -1.0, 0.0, 1.0
    ]
    assert step_toward([0.0, 0.0], [1.0, -1.0], 0.1) == pytest.approx(
        [0.1, -0.1]
    )


def test_rotation_helpers_recover_rotation_vector():
    vector = np.array([0.1, -0.2, 0.05])
    assert rotation_error(rotation_from_rotvec(vector), np.eye(3)) == pytest.approx(
        vector, abs=1e-9
    )


def test_cylindrical_theta_does_not_drift_radius():
    result, theta = integrate_cylindrical_position(
        [0.212, 0.0, 0.3],
        radial_velocity=0.0,
        theta_velocity=math.pi / 2.0,
        vertical_velocity=0.0,
        dt=1.0,
        origin_xy=[0.012, 0.0],
        minimum_radius=0.03,
    )
    assert result == pytest.approx([0.012, 0.2, 0.3], abs=1e-9)
    assert theta == pytest.approx(math.pi / 2.0)


def test_cylindrical_min_radius_does_not_jump_on_entry():
    result, _ = integrate_cylindrical_position(
        [0.022, 0.0, 0.3],
        radial_velocity=0.0,
        theta_velocity=0.0,
        vertical_velocity=0.0,
        dt=0.02,
        origin_xy=[0.012, 0.0],
        minimum_radius=0.03,
        theta_hint=0.0,
    )
    assert result == pytest.approx([0.022, 0.0, 0.3])
