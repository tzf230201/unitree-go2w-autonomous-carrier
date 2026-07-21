import numpy as np

from om6dof_perception.yolox_detector import (
    YoloXDetector,
    resolve_coco_class,
)


def test_free_form_targets_resolve_to_coco_classes():
    assert resolve_coco_class("red cup on the table") == "cup"
    assert resolve_coco_class("glass jar with a black lid") == "bottle"
    assert resolve_coco_class("mobile phone") == "cell phone"
    assert resolve_coco_class("robot gripper") is None


def test_letterbox_preserves_source_aspect_ratio():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    padded, scale = YoloXDetector._letterbox(image)
    assert padded.shape == (640, 640, 3)
    assert scale == 1.0


def test_anchor_count_matches_yolox_output():
    grids, strides = YoloXDetector._anchors()
    assert grids.shape == (1, 8400, 2)
    assert strides.shape == (1, 8400, 1)
