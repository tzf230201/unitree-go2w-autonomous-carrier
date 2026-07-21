"""Small OpenCV-DNN YOLOX wrapper with no PyTorch dependency."""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)

CLASS_ALIASES = {
    "jar": "bottle",
    "glass jar": "bottle",
    "mug": "cup",
    "glass": "wine glass",
    "phone": "cell phone",
    "plant": "potted plant",
    "table": "dining table",
    "sofa": "couch",
}


def resolve_coco_class(description: str) -> Optional[str]:
    """Resolve a free-form GUI target to one COCO class name."""
    text = " ".join(description.lower().replace("_", " ").split())
    if not text:
        return None
    for name in sorted(COCO_CLASSES, key=len, reverse=True):
        if name in text:
            return name
    for alias in sorted(CLASS_ALIASES, key=len, reverse=True):
        if alias in text:
            return CLASS_ALIASES[alias]
    return None


@dataclass(frozen=True)
class Detection:
    bbox: tuple  # x, y, width, height in the source image
    score: float
    class_id: int
    class_name: str


class YoloXDetector:
    """YOLOX-S COCO detector using the ONNX model from OpenCV Zoo."""

    INPUT_SIZE = 640
    STRIDES = (8, 16, 32)

    def __init__(
        self,
        model_path: str,
        confidence: float = 0.35,
        nms_threshold: float = 0.5,
    ) -> None:
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {path}")
        self.model_path = str(path)
        self.confidence = float(confidence)
        self.nms_threshold = float(nms_threshold)
        self.net = cv2.dnn.readNet(self.model_path)
        self.grids, self.expanded_strides = self._anchors()

    @classmethod
    def _anchors(cls):
        grids = []
        strides = []
        for stride in cls.STRIDES:
            size = cls.INPUT_SIZE // stride
            xv, yv = np.meshgrid(np.arange(size), np.arange(size))
            grid = np.stack((xv, yv), axis=2).reshape(1, -1, 2)
            grids.append(grid)
            strides.append(np.full((1, size * size, 1), stride))
        return np.concatenate(grids, axis=1), np.concatenate(strides, axis=1)

    @classmethod
    def _letterbox(cls, bgr):
        height, width = bgr.shape[:2]
        scale = min(cls.INPUT_SIZE / height, cls.INPUT_SIZE / width)
        resized = cv2.resize(
            bgr,
            (int(width * scale), int(height * scale)),
            interpolation=cv2.INTER_LINEAR,
        )
        padded = np.full(
            (cls.INPUT_SIZE, cls.INPUT_SIZE, 3), 114, dtype=np.float32
        )
        padded[: resized.shape[0], : resized.shape[1]] = cv2.cvtColor(
            resized, cv2.COLOR_BGR2RGB
        )
        return padded, scale

    def detect(self, bgr) -> List[Detection]:
        padded, scale = self._letterbox(bgr)
        blob = np.transpose(padded, (2, 0, 1))[None, ...]
        self.net.setInput(blob)
        output = self.net.forward(self.net.getUnconnectedOutLayersNames())[0]
        dets = output[0].copy()
        dets[:, :2] = (
            dets[:, :2] + self.grids[0]
        ) * self.expanded_strides[0]
        dets[:, 2:4] = np.exp(dets[:, 2:4]) * self.expanded_strides[0]

        scores = dets[:, 4:5] * dets[:, 5:]
        class_ids = np.argmax(scores, axis=1)
        confidences = scores[np.arange(scores.shape[0]), class_ids]
        keep_mask = confidences >= self.confidence
        if not np.any(keep_mask):
            return []

        boxes = dets[keep_mask, :4]
        confidences = confidences[keep_mask]
        class_ids = class_ids[keep_mask]
        xywh = np.empty_like(boxes)
        xywh[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        xywh[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        xywh[:, 2:] = boxes[:, 2:]
        xywh /= scale

        indices = cv2.dnn.NMSBoxes(
            xywh.tolist(), confidences.tolist(),
            self.confidence, self.nms_threshold,
        )
        if indices is None or len(indices) == 0:
            return []
        height, width = bgr.shape[:2]
        results = []
        for index in np.asarray(indices).reshape(-1):
            x, y, w, h = (int(round(value)) for value in xywh[index])
            x = max(0, min(width - 1, x))
            y = max(0, min(height - 1, y))
            w = max(4, min(width - x, w))
            h = max(4, min(height - y, h))
            class_id = int(class_ids[index])
            results.append(Detection(
                bbox=(x, y, w, h),
                score=float(confidences[index]),
                class_id=class_id,
                class_name=COCO_CLASSES[class_id],
            ))
        return results

    def best(self, bgr, class_name: str) -> Optional[Detection]:
        matches = [
            detection for detection in self.detect(bgr)
            if detection.class_name == class_name
        ]
        return max(matches, key=lambda detection: detection.score, default=None)
