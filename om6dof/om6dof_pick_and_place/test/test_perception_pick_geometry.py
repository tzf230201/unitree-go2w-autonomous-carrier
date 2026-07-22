import numpy as np

from om6dof_pick_and_place.direct_pick_node import (
    DirectPickNode,
    approach_standoff_distances,
    axis_aligned_bbox_top_world,
    consecutive_detection_streak,
    direct_approach_direction,
    image_axis_tracking_target,
    linear_waypoints,
    optical_point_to_world,
    safe_approach_standoff_distances,
    stable_point_median,
    yaw_tracking_target,
)


def test_optical_point_transform_identity():
    point = optical_point_to_world(
        np.array([0.1, 0.2, 0.3]),
        np.array([1.0, 2.0, 3.0]),
        np.eye(3),
        np.array([0.01, 0.02, 0.03]),
        np.eye(3),
    )
    assert np.allclose(point, [1.11, 2.22, 3.33])


def test_stable_point_median_accepts_tight_cluster():
    samples = [
        np.array([0.10, 0.20, 0.30]),
        np.array([0.11, 0.19, 0.30]),
        np.array([0.09, 0.20, 0.31]),
    ]
    assert np.allclose(
        stable_point_median(samples, max_spread=0.02),
        [0.10, 0.20, 0.30],
    )


def test_stable_point_median_rejects_depth_jump():
    samples = [np.array([0.0, 0.0, 0.3]), np.array([0.0, 0.0, 0.8])]
    assert stable_point_median(samples, max_spread=0.03) is None


def test_linear_waypoints_advance_at_constant_height_and_bounded_step():
    points = linear_waypoints(
        np.array([0.20, 0.0, 0.10]),
        np.array([0.27, 0.0, 0.10]),
        max_step=0.015,
    )
    previous = np.array([0.20, 0.0, 0.10])
    assert np.allclose(points[-1], [0.27, 0.0, 0.10])
    assert all(np.isclose(point[2], 0.10) for point in points)
    for point in points:
        assert np.linalg.norm(point - previous) <= 0.015 + 1e-9
        previous = point


def test_direct_approach_direction_includes_vertical_component():
    direction = direct_approach_direction(
        np.array([0.10, -0.05, 0.25]),
        np.array([0.40, 0.10, 0.10]),
    )
    expected = np.array([0.30, 0.15, -0.15])
    expected /= np.linalg.norm(expected)
    assert np.allclose(direction, expected)
    assert direction[2] < 0.0


def test_bbox_top_world_uses_vertical_extent_after_rotation():
    top = axis_aligned_bbox_top_world(
        center_optical=np.array([0.0, 0.0, 0.40]),
        size_optical=np.array([0.04, 0.10, 0.06]),
        p_we=np.zeros(3),
        R_we=np.eye(3),
        t_ec=np.zeros(3),
        R_eo=np.eye(3),
    )
    assert np.allclose(top, [0.0, 0.0, 0.43])


def test_linear_waypoints_follow_diagonal_3d_ray():
    start = np.array([0.10, 0.00, 0.25])
    end = np.array([0.30, 0.10, 0.05])
    points = linear_waypoints(start, end, max_step=0.04)
    ray = end - start
    for point in points:
        progress = (point[0] - start[0]) / ray[0]
        assert np.allclose(point, start + progress * ray)


def test_visual_approach_distances_descend_and_end_at_final_standoff():
    assert np.allclose(
        approach_standoff_distances(0.16, 0.08, 0.04),
        [0.16, 0.12, 0.08],
    )


def test_visual_approach_distances_handle_non_even_step():
    assert np.allclose(
        approach_standoff_distances(0.15, 0.08, 0.04),
        [0.15, 0.11, 0.08],
    )


def test_adaptive_approach_skips_folded_near_base_waypoint():
    assert np.allclose(
        safe_approach_standoff_distances(
            0.406, [0.16, 0.12, 0.08], minimum_target_radius=0.28),
        [0.12, 0.08],
    )


def test_detection_streak_requires_strictly_consecutive_frames():
    streak = 0
    for detected in [True, True, False, True, True, True]:
        streak = consecutive_detection_streak(streak, detected)
    assert streak == 3


def test_yaw_tracking_ignores_target_inside_deadband():
    target, error, move = yaw_tracking_target(
        0.2, 0.001, 1.0, 0.5, -1.0, 0.05, 0.04, -2.6, 2.6)
    assert np.isclose(target, 0.2)
    assert abs(error) < 0.05
    assert move is False


def test_yaw_tracking_moves_right_and_limits_each_step():
    target, error, move = yaw_tracking_target(
        0.2, 0.5, 1.0, 1.0, -1.0, 0.02, 0.04, -2.6, 2.6)
    assert error > 0.0
    assert np.isclose(target, 0.16)
    assert move is True


def test_yaw_tracking_respects_joint_limit():
    target, _error, move = yaw_tracking_target(
        -2.59, 0.5, 1.0, 1.0, -1.0, 0.02, 0.20, -2.6, 2.6)
    assert np.isclose(target, -2.6)
    assert move is True


def test_vertical_tracking_moves_joint5_positive_for_object_below_image():
    target, error, move = image_axis_tracking_target(
        0.89, 0.14, 0.37, 0.35, 1.0, 0.045, 0.04, -1.8, 1.8)
    assert error > 0.0
    assert np.isclose(target, 0.93)
    assert move is True


def test_pick_preflight_rejects_unreachable_target():
    node = object.__new__(DirectPickNode)
    import threading
    node._run_lock = threading.Lock()
    node._busy = False
    node._object_world = lambda allow_stale=False: (
        np.zeros(3), np.array([0.6, 0.0, 0.0]), np.eye(3)
    )
    node._reachable = lambda _point: {
        "reachable": False,
        "radius": 0.6,
        "standoff_ok": False,
        "grasp_ok": False,
        "obj": [0.6, 0.0, 0.0],
    }
    node.get_parameter = lambda name: type("P", (), {"value": (
        [0.0, -1.0, -1.0] if name == "perception_world_min"
        else [1.0, 1.0, 1.0]
    )})()

    ready, message = node._pick_preflight()

    assert ready is False
    assert "outside arm workspace" in message
