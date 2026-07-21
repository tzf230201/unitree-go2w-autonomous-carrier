import numpy as np

from types import SimpleNamespace

from om6dof_perception.perception_node import (
    associated_detection,
    bbox_iou,
    estimate_bbox3d,
    project_bbox3d,
)


def _detection(bbox, score, class_name="bottle"):
    return SimpleNamespace(bbox=bbox, score=score, class_name=class_name)


def test_associated_detection_keeps_same_instance_over_higher_confidence():
    tracked = _detection((100, 100, 40, 80), 0.62)
    other = _detection((400, 100, 45, 90), 0.95)
    selected = associated_detection(
        [other, tracked], "bottle", reference_bbox=(102, 101, 40, 80))
    assert selected is tracked


def test_associated_detection_rejects_non_overlapping_instance():
    other = _detection((400, 100, 45, 90), 0.95)
    assert associated_detection(
        [other], "bottle", reference_bbox=(100, 100, 40, 80)) is None


def test_bbox_iou_handles_overlap_and_separation():
    assert bbox_iou((0, 0, 10, 10), (5, 0, 10, 10)) > 0.0
    assert bbox_iou((0, 0, 10, 10), (20, 0, 10, 10)) == 0.0


class Intrinsics:
    fx = 100.0
    fy = 100.0
    ppx = 50.0
    ppy = 50.0


def test_bbox3d_moves_target_behind_visible_surface():
    depth = np.full((100, 100), 2000, dtype=np.uint16)
    depth[30:70, 40:60] = 500

    box = estimate_bbox3d(
        depth, Intrinsics(), 0.001, (40, 30, 20, 40),
        min_thickness_m=0.025, max_thickness_m=0.12,
    )

    assert box is not None
    assert box["depth_inferred"] is True
    assert box["front_z"] == 0.5
    assert box["center"][2] > box["front_z"]
    assert np.isclose(box["center"][2],
                      box["front_z"] + box["size"][2] / 2.0)


def test_bbox3d_ignores_far_background_inside_yolo_roi():
    depth = np.full((100, 100), 1800, dtype=np.uint16)
    depth[25:75, 35:65] = 600

    box = estimate_bbox3d(
        depth, Intrinsics(), 0.001, (30, 20, 40, 60),
        depth_band_m=0.05,
    )

    assert box is not None
    assert box["front_z"] < 0.7
    assert box["back_z"] < 0.8


def test_bbox3d_returns_none_without_valid_depth():
    depth = np.zeros((100, 100), dtype=np.uint16)
    assert estimate_bbox3d(
        depth, Intrinsics(), 0.001, (30, 20, 40, 60)
    ) is None


def test_project_bbox3d_returns_front_and_back_faces():
    box = {
        "center": [0.0, 0.0, 0.55],
        "size": [0.1, 0.2, 0.1],
        "front_z": 0.5,
        "back_z": 0.6,
    }

    corners = project_bbox3d(box, Intrinsics())

    assert len(corners) == 8
    # The nearer face appears larger in the image than the inferred back.
    assert corners[1][0] - corners[0][0] > corners[5][0] - corners[4][0]
    assert corners[3][1] - corners[0][1] > corners[7][1] - corners[4][1]
