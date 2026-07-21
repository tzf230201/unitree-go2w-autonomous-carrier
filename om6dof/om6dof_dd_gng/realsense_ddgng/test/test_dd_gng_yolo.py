import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dd_gng_yolo import assign_node_labels, foreground_depth


def detection(bbox, score=0.8, class_name="bottle", class_id=39):
    return SimpleNamespace(
        bbox=bbox,
        score=score,
        class_name=class_name,
        class_id=class_id,
    )


def test_foreground_depth_ignores_far_background():
    depth = np.full((100, 100), 1800, dtype=np.uint16)
    depth[20:80, 30:70] = 500

    result = foreground_depth(depth, (25, 15, 50, 70), 0.001)

    assert np.isclose(result, 0.5)


def test_nodes_need_pixel_and_depth_overlap_for_label():
    nodes_uv = np.array([[50, 50], [50, 50], [90, 90]])
    nodes_xyz = np.array([
        [0.0, 0.0, 0.52],
        [0.0, 0.0, 1.20],
        [0.0, 0.0, 0.52],
    ])
    item = detection((30, 30, 40, 40))

    labels = assign_node_labels(
        nodes_uv, nodes_xyz, [item], [0.5], depth_tolerance_m=0.08
    )

    assert labels[0] is item
    assert labels[1] is None
    assert labels[2] is None


def test_highest_confidence_box_wins_overlap():
    low = detection((20, 20, 50, 50), score=0.4, class_name="cup", class_id=41)
    high = detection((30, 30, 50, 50), score=0.9)

    labels = assign_node_labels(
        np.array([[40, 40]]),
        np.array([[0.0, 0.0, 0.5]]),
        [low, high],
        [0.5, 0.5],
    )

    assert labels == [high]


def test_missing_roi_depth_falls_back_to_2d_intersection():
    item = detection((10, 10, 30, 30))
    labels = assign_node_labels(
        np.array([[20, 20]]),
        np.array([[0.0, 0.0, 2.0]]),
        [item],
        [None],
    )
    assert labels == [item]
