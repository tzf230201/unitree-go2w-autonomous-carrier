"""
Forward / inverse kinematics for OpenManipulator-X.

Coordinate convention
---------------------
World frame: +x forward, +y left, +z up. Base of robot at origin.

Joint angles (q1, q2, q3, q4), all in radians:
  q1 — base yaw about +z (CCW from above is positive)
  q2 — shoulder, rotation about local +y. q2=0 → arm horizontal forward.
       Positive q2 tilts arm DOWN (per right-hand rule about +y).
  q3 — elbow, rotation about local +y at joint3. q3=0 → link3 collinear with
       the forward direction of joint3's frame.
  q4 — wrist pitch, rotation about local +y at joint4.

End-effector pitch (user-facing): angle of EE vector above the horizontal,
positive = pointing up. With the chosen joint conventions:

    pitch = -(q2 + q3 + q4)

so commanding a positive pitch wants the arm tip up.

Geometry (from open_manipulator_description URDF)
-------------------------------------------------
  base_z   = 0.0765  m   (joint2 height above base)
  L1_x     = 0.024   m   (joint2 → joint3, forward)
  L1_z     = 0.128   m   (joint2 → joint3, up)
  L2       = 0.124   m   (joint3 → joint4, along link3's x)
  L3       = 0.126   m   (joint4 → EE tip, along link4's x)

The bent link2 is treated as a single vector (L1_x, L1_z); we precompute its
length L1 = sqrt(L1_x² + L1_z²) and offset angle β = atan2(L1_x, L1_z) so the
IK simplifies to a 2-link planar problem from joint2 to joint4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class OMXGeom:
    base_z: float = 0.0765
    L1_x: float = 0.024
    L1_z: float = 0.128
    L2: float = 0.124
    L3: float = 0.126

    @property
    def L1(self) -> float:
        return math.hypot(self.L1_x, self.L1_z)

    @property
    def beta(self) -> float:
        """Tilt of link2 from +z axis at q2 = 0."""
        return math.atan2(self.L1_x, self.L1_z)


DEFAULT_GEOM = OMXGeom()


def forward_kinematics(
    q1: float, q2: float, q3: float, q4: float, geom: OMXGeom = DEFAULT_GEOM
) -> Tuple[float, float, float, float]:
    """Return (x, y, z, pitch) of the end-effector.

    pitch = elevation angle of the EE forward vector above the horizontal,
    positive = up.
    """
    L1, L2, L3 = geom.L1, geom.L2, geom.L3
    beta = geom.beta

    # planar (r-z) computation
    s_a = math.sin(beta + q2)
    c_a = math.cos(beta + q2)
    s_b = math.sin(q2 + q3)
    c_b = math.cos(q2 + q3)
    s_c = math.sin(q2 + q3 + q4)
    c_c = math.cos(q2 + q3 + q4)

    r = L1 * s_a + L2 * c_b + L3 * c_c
    z = geom.base_z + L1 * c_a - L2 * s_b - L3 * s_c

    x = r * math.cos(q1)
    y = r * math.sin(q1)
    pitch = -(q2 + q3 + q4)
    return x, y, z, pitch


class IKError(Exception):
    pass


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    pitch: float,
    elbow_up: bool = True,
    geom: OMXGeom = DEFAULT_GEOM,
) -> Tuple[float, float, float, float]:
    """Solve for (q1, q2, q3, q4) reaching (x, y, z) with given EE pitch.

    Raises IKError if target is unreachable.
    """
    L1, L2, L3 = geom.L1, geom.L2, geom.L3
    beta = geom.beta

    # 1) Base yaw
    q1 = math.atan2(y, x)
    r_e = math.hypot(x, y)

    # 2) Subtract last link to find wrist (joint4 position) in r-z
    gamma = -pitch  # cumulative tilt q2+q3+q4
    rw = r_e - L3 * math.cos(gamma)
    zw = z - geom.base_z + L3 * math.sin(gamma)

    # 3) Two-link IK: shoulder→wrist with link A (length L1, x-axis tilt β)
    #    and link B (length L2, x-axis tilt π/2 from +z when q3=0).
    dist2 = rw * rw + zw * zw
    cos_delta = (dist2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if cos_delta < -1.0 - 1e-6 or cos_delta > 1.0 + 1e-6:
        raise IKError(
            f"target unreachable: |wrist|={math.sqrt(dist2):.4f} m, "
            f"reach ∈ [{abs(L1-L2):.4f}, {L1+L2:.4f}]"
        )
    cos_delta = max(-1.0, min(1.0, cos_delta))
    delta = math.acos(cos_delta)
    if not elbow_up:
        delta = -delta

    A = L1 + L2 * math.cos(delta)
    B = L2 * math.sin(delta)
    alpha_A = math.atan2(rw, zw) - math.atan2(B, A)

    q2 = alpha_A - beta
    q3 = delta - math.pi / 2.0 + beta
    q4 = gamma - q2 - q3

    return q1, q2, q3, q4


def workspace_limits(geom: OMXGeom = DEFAULT_GEOM) -> Tuple[float, float]:
    """Approximate (r_min, r_max) of EE reach in horizontal plane."""
    L1, L2, L3 = geom.L1, geom.L2, geom.L3
    return abs(L1 - L2) - L3, L1 + L2 + L3


# ----- "home" / safe poses (joint angles, radians) -----
# All measured in the kinematic frame (NOT raw motor ticks).
POSE_HOME_UP = (0.0, -DEFAULT_GEOM.beta, -math.pi / 2.0 + DEFAULT_GEOM.beta, 0.0)
"""Arm pointing straight up. q2 = -β cancels the link-2 bend so it stands
vertical; q3 = -π/2 + β puts link-3 vertical too; q4 = 0 keeps the EE
collinear with the arm."""

POSE_READY_FORWARD = (0.0, 0.0, 0.0, 0.0)
"""Arm horizontal forward (matches all-zero joints of OM-X URDF)."""
