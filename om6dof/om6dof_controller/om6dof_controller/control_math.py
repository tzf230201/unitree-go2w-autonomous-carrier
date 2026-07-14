"""ROS-independent protocol and math helpers for :mod:`om6dof_controller`."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


MODE_AUTONOMOUS = "AUTONOMOUS"
MODE_JOINT = "JOINT"
MODE_CARTESIAN = "CARTESIAN"
MODE_CYLINDRICAL = "CYLINDRICAL"
MODE_READY = "READY"
MODE_STARTUP = "STARTUP"
MOTION_MODES = (MODE_JOINT, MODE_CARTESIAN, MODE_CYLINDRICAL)
OPERATION_MODES = (
    MODE_AUTONOMOUS,
    *MOTION_MODES,
    MODE_READY,
    MODE_STARTUP,
)


def normalize_operation_mode(value: str) -> str:
    normalized = str(value).strip().upper().replace("-", "_")
    aliases = {
        "AUTO": MODE_AUTONOMOUS,
        "AUTONOMY": MODE_AUTONOMOUS,
        "AUTONOMOUS": MODE_AUTONOMOUS,
        "OFF": MODE_AUTONOMOUS,
        "JOINT": MODE_JOINT,
        "JOINTS": MODE_JOINT,
        "CART": MODE_CARTESIAN,
        "IK": MODE_CARTESIAN,
        "CARTESIAN": MODE_CARTESIAN,
        "CYL": MODE_CYLINDRICAL,
        "CYLINDER": MODE_CYLINDRICAL,
        "CYLINDRICAL": MODE_CYLINDRICAL,
        "SILINDER": MODE_CYLINDRICAL,
        "SILINDRIS": MODE_CYLINDRICAL,
        "READY": MODE_READY,
        "STARTUP": MODE_STARTUP,
        "START": MODE_STARTUP,
    }
    try:
        return aliases[normalized]
    except KeyError as exc:
        expected = ", ".join(OPERATION_MODES)
        raise ValueError(
            f"unknown operation mode '{value}'; expected {expected}"
        ) from exc


def next_motion_mode(current: str) -> str:
    mode = normalize_operation_mode(current)
    if mode not in MOTION_MODES:
        return MODE_JOINT
    return MOTION_MODES[(MOTION_MODES.index(mode) + 1) % len(MOTION_MODES)]


def validated_control_command(values: Sequence[float]) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError("control_cmd must contain exactly six finite values")
    return result.copy()


def validated_joint_positions(values: Sequence[float], label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 6:
        raise ValueError(f"{label} must contain six joint positions")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


def clamp_positions(
    values: Sequence[float], lower: Sequence[float], upper: Sequence[float]
) -> list[float]:
    return [
        max(float(lo), min(float(hi), float(value)))
        for value, lo, hi in zip(values, lower, upper)
    ]


def step_toward(
    current: Sequence[float], target: Sequence[float], maximum_step: float
) -> list[float]:
    step = max(0.0, float(maximum_step))
    result = []
    for present, goal in zip(current, target):
        error = float(goal) - float(present)
        result.append(float(present) + max(-step, min(step, error)))
    return result


def limit_norm(values: Sequence[float], maximum: float) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    limit = max(0.0, float(maximum))
    norm = float(np.linalg.norm(vector))
    if norm > limit and norm > 0.0:
        return vector * (limit / norm)
    return vector


def rotation_from_rotvec(rotvec: Sequence[float]) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=float)
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.eye(3)
    axis = vector / theta
    skew = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return (
        np.eye(3)
        + math.sin(theta) * skew
        + (1.0 - math.cos(theta)) * (skew @ skew)
    )


def rotation_error(target: np.ndarray, present: np.ndarray) -> np.ndarray:
    relative = np.asarray(target, dtype=float) @ np.asarray(
        present, dtype=float
    ).T
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.zeros(3)
    sine = math.sin(angle)
    if abs(sine) < 1e-7:
        axis = np.sqrt(np.maximum(0.0, (np.diag(relative) + 1.0) / 2.0))
        if float(np.linalg.norm(axis)) < 1e-9:
            return np.zeros(3)
        axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array([
        relative[2, 1] - relative[1, 2],
        relative[0, 2] - relative[2, 0],
        relative[1, 0] - relative[0, 1],
    ]) / (2.0 * sine)
    return axis * angle


def integrate_cylindrical_position(
    position: Sequence[float],
    radial_velocity: float,
    theta_velocity: float,
    vertical_velocity: float,
    dt: float,
    origin_xy: Sequence[float],
    minimum_radius: float,
    theta_hint: Optional[float] = None,
) -> Tuple[np.ndarray, float]:
    """Integrate radius and angle exactly, avoiding Euler radial drift."""
    current = np.asarray(position, dtype=float)
    origin = np.asarray(origin_xy, dtype=float)
    if current.shape != (3,) or origin.shape != (2,):
        raise ValueError("position must have 3 values and origin_xy 2 values")
    dx = float(current[0] - origin[0])
    dy = float(current[1] - origin[1])
    radius = math.hypot(dx, dy)
    theta = (
        float(theta_hint)
        if radius < minimum_radius and theta_hint is not None
        else math.atan2(dy, dx)
    )
    safe_dt = max(0.0, float(dt))
    radial_floor = float(minimum_radius) if radius >= minimum_radius else radius
    new_radius = max(radial_floor, radius + float(radial_velocity) * safe_dt)
    new_theta = theta + float(theta_velocity) * safe_dt
    return np.array([
        float(origin[0]) + new_radius * math.cos(new_theta),
        float(origin[1]) + new_radius * math.sin(new_theta),
        float(current[2]) + float(vertical_velocity) * safe_dt,
    ]), new_theta
