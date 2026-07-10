"""Velocity-IK helper for the 6-DOF Chain arm.

PyKDL provides FK + Jacobian; we apply a damped Moore-Penrose pseudoinverse to
get joint velocities from a desired EE twist. Good for joystick jogging where
each tick wants a small Δq from a button-derived velocity command — no need
for full position IK (which can fail near singularities or pick weird
elbow-up/down branches).

URDF is rendered from `om_chain_bringup/urdf/om_chain.urdf.xacro` at startup
via the `xacro` CLI. We don't depend on `kdl_parser_py` (not packaged for
Humble) — instead we walk the URDF with `urdf_parser_py` and build a
`PyKDL.Chain` joint-by-joint.
"""

from __future__ import annotations

import math
import subprocess
from pathlib import Path
from typing import List, Tuple

import numpy as np
import PyKDL
from ament_index_python.packages import get_package_share_directory
from urdf_parser_py.urdf import URDF


def _kdl_chain_from_urdf(urdf_str: str, base_link: str, tip_link: str) -> PyKDL.Chain:
    """Build a PyKDL.Chain from a URDF string between `base_link` and `tip_link`.

    Mirrors the conventions in ROS `kdl_parser` exactly:
      * F_parent_jnt = URDF joint origin (parent_link → joint frame)
      * for a moving joint, the rotation/translation axis is expressed in the
        PARENT frame, i.e. axis_parent = F_parent_jnt.M * urdf_axis, and the
        joint's reference point is F_parent_jnt.p
      * the KDL Segment tip frame is F_parent_jnt

    Getting this wrong (e.g. axis not rotated by the origin, or putting the
    origin after the joint) makes FK agree only at q = 0 and silently diverge
    elsewhere — which is exactly what happened before this fix.
    """
    robot = URDF.from_xml_string(urdf_str)
    joint_chain = robot.get_chain(base_link, tip_link, joints=True, links=False)
    chain = PyKDL.Chain()
    q_min: List[float] = []
    q_max: List[float] = []
    for jname in joint_chain:
        uj = robot.joint_map[jname]
        origin = uj.origin
        xyz = origin.xyz if (origin and origin.xyz) else [0.0, 0.0, 0.0]
        rpy = origin.rpy if (origin and origin.rpy) else [0.0, 0.0, 0.0]
        F_parent_jnt = PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))

        if uj.type == "fixed":
            kdl_j = PyKDL.Joint(uj.name, PyKDL.Joint.Fixed)
        else:
            ax = uj.axis if getattr(uj, "axis", None) else [0.0, 0.0, 1.0]
            axis_parent = F_parent_jnt.M * PyKDL.Vector(*ax)
            if uj.type in ("revolute", "continuous"):
                jtype = PyKDL.Joint.RotAxis
            elif uj.type == "prismatic":
                jtype = PyKDL.Joint.TransAxis
            else:
                raise ValueError(f"unsupported URDF joint type: {uj.type}")
            kdl_j = PyKDL.Joint(uj.name, F_parent_jnt.p, axis_parent, jtype)
            # record actuated-joint limits (continuous joints → ±π fallback)
            lim = getattr(uj, "limit", None)
            if lim is not None and lim.lower is not None and lim.upper is not None:
                q_min.append(float(lim.lower))
                q_max.append(float(lim.upper))
            else:
                q_min.append(-math.pi)
                q_max.append(+math.pi)

        chain.addSegment(PyKDL.Segment(uj.name, kdl_j, F_parent_jnt))
    return chain, np.array(q_min), np.array(q_max)


def render_chain_urdf(
    pkg: str = "open_manipulator_6dof_description",
    xacro_rel: str = "urdf/open_manipulator_6dof.urdf.xacro",
    extra_args: List[str] | None = None,
) -> str:
    """Render the rig's URDF xacro to a string.

    Default = the open-source 6dof description: the single canonical
    kinematics source, shared with move_group and Servo."""
    share = Path(get_package_share_directory(pkg))
    xacro_path = share / xacro_rel
    if not xacro_path.exists():
        raise FileNotFoundError(f"xacro not found: {xacro_path}")
    cmd = ["xacro", str(xacro_path)]
    if extra_args:
        cmd.extend(extra_args)
    out = subprocess.run(cmd, capture_output=True, check=True)
    return out.stdout.decode()


class IKSolver:
    """Velocity-resolved IK for the arm group, joint1..joint6."""

    def __init__(
        self,
        base_link: str = "world",
        tip_link: str = "end_effector_link",
        urdf_pkg: str = "open_manipulator_6dof_description",
        damping: float = 0.1,
        xacro_rel: str = "urdf/open_manipulator_6dof.urdf.xacro",
    ) -> None:
        urdf_str = render_chain_urdf(urdf_pkg, xacro_rel)
        self.chain, self.q_min, self.q_max = _kdl_chain_from_urdf(
            urdf_str, base_link, tip_link
        )
        self.n_joints = self.chain.getNrOfJoints()
        if self.n_joints != 6:
            raise RuntimeError(
                f"expected 6 actuated joints between {base_link} and {tip_link}, "
                f"got {self.n_joints} — URDF or chain config wrong"
            )
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
        self.damping = float(damping)
        self.base_link = base_link
        self.tip_link = tip_link
        self.n_segments = self.chain.getNrOfSegments()

    # ---------------- kinematics ----------------
    def _jnt(self, q: np.ndarray) -> PyKDL.JntArray:
        ja = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            ja[i] = float(q[i])
        return ja

    def fk(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return (xyz, rpy) of tip_link in base_link frame for joint vector q."""
        ja = self._jnt(q)
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(ja, frame)
        xyz = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        rpy = np.array(list(frame.M.GetRPY()))
        return xyz, rpy

    def fk_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return (xyz, R) where R is the 3×3 rotation matrix of tip_link in
        the base frame. Needed for orientation jogging about the tool frame."""
        ja = self._jnt(q)
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(ja, frame)
        xyz = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        M = frame.M
        R = np.array([
            [M[0, 0], M[0, 1], M[0, 2]],
            [M[1, 0], M[1, 1], M[1, 2]],
            [M[2, 0], M[2, 1], M[2, 2]],
        ])
        return xyz, R

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """Return the 6×N geometric Jacobian (linear rows on top, angular below)."""
        ja = self._jnt(q)
        J = PyKDL.Jacobian(self.n_joints)
        self.jac_solver.JntToJac(ja, J)
        out = np.zeros((6, self.n_joints))
        for r in range(6):
            for c in range(self.n_joints):
                out[r, c] = J[r, c]
        return out

    # ---------------- velocity IK ----------------
    def damped_pinv(self, J: np.ndarray, damping: float | None = None) -> np.ndarray:
        """Damped Moore-Penrose pseudoinverse. Falls back to standard pinv if
        damping is 0."""
        lam = float(self.damping if damping is None else damping)
        if lam <= 0.0:
            return np.linalg.pinv(J)
        m = J.shape[0]
        return J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(m))

    def velocity_ik(self, q: np.ndarray, twist: np.ndarray) -> np.ndarray:
        """Returns q_dot for the desired EE twist in base_link frame.

        twist layout: [vx, vy, vz, wx, wy, wz] (m/s, rad/s)
        """
        J = self.jacobian(q)
        J_pinv = self.damped_pinv(J)
        return J_pinv @ twist

    def position_only_velocity_ik(
        self, q: np.ndarray, lin_vel: np.ndarray
    ) -> np.ndarray:
        """Same idea as `velocity_ik` but only constrains the linear part of
        the EE velocity — orientation is allowed to drift freely. Matches
        ROBOTIS open_manipulator_friends' `PositionOnlySRJacobian` solver.

        For pure translation jogging this is dramatically nicer than full 6D
        IK: fewer joints have to coordinate, so the motion looks like "pure
        Cartesian translation" and the wrist doesn't spin trying to maintain
        a fixed orientation it never had to begin with.
        """
        J_lin = self.jacobian(q)[:3, :]   # 3 × N (top half of Jacobian)
        lam_sq = float(self.damping) ** 2
        J_pinv = J_lin.T @ np.linalg.inv(J_lin @ J_lin.T + lam_sq * np.eye(3))
        return J_pinv @ lin_vel

    def velocity_ik_priority(
        self, q: np.ndarray, lin_vel: np.ndarray, ang_vel: np.ndarray
    ) -> np.ndarray:
        """Task-priority velocity IK: EE POSITION is the primary task,
        rotation runs in its null space. Guarantees (to first order) that a
        rotation command can never translate the EoE — if the arm cannot
        rotate further while holding the tip, rotation simply stalls instead
        of dragging the tip away. This matches how tip-centred rotation
        should feel on a non-spherical-wrist arm."""
        J = self.jacobian(q)
        J_pos, J_rot = J[:3, :], J[3:, :]
        lam_sq = float(self.damping) ** 2
        Jp_pinv = J_pos.T @ np.linalg.inv(
            J_pos @ J_pos.T + lam_sq * np.eye(3))
        q_pos = Jp_pinv @ lin_vel
        N = np.eye(self.n_joints) - Jp_pinv @ J_pos
        JrN = J_rot @ N
        JrN_pinv = JrN.T @ np.linalg.inv(
            JrN @ JrN.T + lam_sq * np.eye(3))
        q_rot = JrN_pinv @ (ang_vel - J_rot @ q_pos)
        return q_pos + N @ q_rot

    def ee_to_base_angular(self, q: np.ndarray, w_ee: np.ndarray) -> np.ndarray:
        """Rotate an angular velocity vector from EE (tool) frame into the
        base frame so the user feels "roll" = around tool axis, "pitch" = tool
        nodding, etc. independently of the arm's current orientation."""
        ja = self._jnt(q)
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(ja, frame)
        M = frame.M  # rotation matrix EE→base
        v = PyKDL.Vector(float(w_ee[0]), float(w_ee[1]), float(w_ee[2]))
        out = M * v
        return np.array([out.x(), out.y(), out.z()])

    def manipulability(self, q: np.ndarray) -> float:
        """Yoshikawa manipulability index: sqrt(det(J·J^T)). Goes to 0 at
        singular configurations; a useful early-warning signal for jogging."""
        J = self.jacobian(q)
        return float(np.sqrt(max(0.0, np.linalg.det(J @ J.T))))

    # ---------------- self-collision (lightweight capsule model) ----------------
    def link_points(self, q: np.ndarray) -> np.ndarray:
        """Return the 3D position of every segment frame along the chain
        (base, after seg0, after seg1, … tip), in base frame. Consecutive
        points are the link "skeleton" used as capsule segments."""
        ja = self._jnt(q)
        pts = [np.zeros(3)]  # base_link origin
        for seg in range(1, self.n_segments + 1):
            frame = PyKDL.Frame()
            self.fk_solver.JntToCart(ja, frame, seg)
            pts.append(np.array([frame.p.x(), frame.p.y(), frame.p.z()]))
        return np.array(pts)

    @staticmethod
    def _segment_distance(p1, p2, p3, p4) -> float:
        """Shortest distance between segment [p1,p2] and segment [p3,p4]."""
        d1 = p2 - p1
        d2 = p4 - p3
        r = p1 - p3
        a = float(d1 @ d1); e = float(d2 @ d2); f = float(d2 @ r)
        EPS = 1e-9
        if a <= EPS and e <= EPS:
            return float(np.linalg.norm(p1 - p3))
        if a <= EPS:
            s = 0.0
            t = min(max(f / e, 0.0), 1.0)
        else:
            c = float(d1 @ r)
            if e <= EPS:
                t = 0.0
                s = min(max(-c / a, 0.0), 1.0)
            else:
                b = float(d1 @ d2)
                denom = a * e - b * b
                s = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > EPS else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t = 0.0; s = min(max(-c / a, 0.0), 1.0)
                elif t > 1.0:
                    t = 1.0; s = min(max((b - c) / a, 0.0), 1.0)
        cp1 = p1 + d1 * s
        cp2 = p3 + d2 * t
        return float(np.linalg.norm(cp1 - cp2))

    def self_collides(
        self,
        q: np.ndarray,
        radius: float = 0.025,
        skip_adjacent: int = 2,
    ) -> bool:
        """Approximate self-collision test: each link is a capsule (skeleton
        segment + `radius`). Two links collide if their segment distance is
        below 2·radius. Links within `skip_adjacent` of each other are ignored
        (neighbours always "touch" at the shared joint)."""
        pts = self.link_points(q)
        n = len(pts) - 1  # number of skeleton segments
        thr = 2.0 * radius
        for i in range(n):
            for j in range(i + 1 + skip_adjacent, n):
                # skip zero-length segments (fixed joints with no offset)
                if np.linalg.norm(pts[i + 1] - pts[i]) < 1e-6:
                    continue
                if np.linalg.norm(pts[j + 1] - pts[j]) < 1e-6:
                    continue
                d = self._segment_distance(pts[i], pts[i + 1], pts[j], pts[j + 1])
                if d < thr:
                    return True
        return False

    # ---------------- position-target IK (ROBOTIS-style) ----------------
    def solve_position_ik(
        self,
        q_init: np.ndarray,
        target_pos: np.ndarray,
        max_iter: int = 10,
        param: float = 0.002,
        gamma: float = 0.5,
        wn_pos: float = 1.0 / 0.3,
        tol: float = 1e-12,
    ) -> Tuple[np.ndarray, bool]:
        """Position-only Levenberg-Marquardt IK, a direct port of ROBOTIS
        open_manipulator_friends' `inverseSolverUsingPositionOnlySRJacobian`.

        Solves for joint angles that put the tip at `target_pos` (orientation
        is left free). Unlike single-step velocity IK, this converges to an
        absolute target each call, so jogging never accumulates drift.

        The Levenberg-Marquardt damping `lambda = error² + param` is adaptive:
        large far from the target (stable through singularities), shrinking as
        it converges (accurate). Mirrors Sugihara's method.

        Returns (q_solution, converged).
        """
        q = np.array(q_init, dtype=float)
        We = wn_pos * np.eye(3)
        Wn = np.eye(self.n_joints)

        xyz, _ = self.fk(q)
        pos_err = np.asarray(target_pos, dtype=float) - xyz
        pre_Ek = float(pos_err.T @ We @ pos_err)

        for _ in range(max_iter):
            J_pos = self.jacobian(q)[:3, :]                 # 3 × N
            lam = pre_Ek + param
            sr = J_pos.T @ We @ J_pos + lam * Wn            # N × N
            gerr = J_pos.T @ We @ pos_err                   # N
            try:
                dq = np.linalg.solve(sr, gerr)
            except np.linalg.LinAlgError:
                return q, False

            q_new = np.clip(q + dq, self.q_min, self.q_max)
            xyz, _ = self.fk(q_new)
            pos_err = np.asarray(target_pos, dtype=float) - xyz
            new_Ek = float(pos_err.T @ We @ pos_err)

            if new_Ek < tol:
                return q_new, True
            elif new_Ek < pre_Ek:
                q = q_new
                pre_Ek = new_Ek
            else:
                # step worsened the error → take a smaller (rolled-back) step
                q = np.clip(q + (1.0 - gamma) * dq, self.q_min, self.q_max)
                xyz, _ = self.fk(q)
                pos_err = np.asarray(target_pos, dtype=float) - xyz
                pre_Ek = float(pos_err.T @ We @ pos_err)

        # ran out of iterations — return best-so-far (jogging deltas are tiny,
        # so even a non-converged result is a good incremental move)
        return q, False

    # ---------------- full 6D pose IK (ROBOTIS-style) ----------------
    @staticmethod
    def orientation_difference(R_target: np.ndarray, R_present: np.ndarray) -> np.ndarray:
        """ROBOTIS `orientationDifference`: 0.5·Σ (col_present × col_target).
        A first-order rotation-error vector used by the SR-Jacobian solver."""
        return 0.5 * (
            np.cross(R_present[:, 0], R_target[:, 0])
            + np.cross(R_present[:, 1], R_target[:, 1])
            + np.cross(R_present[:, 2], R_target[:, 2])
        )

    def solve_pose_ik(
        self,
        q_init: np.ndarray,
        target_pos: np.ndarray,
        target_R: np.ndarray,
        max_iter: int = 20,
        param: float = 0.002,
        gamma: float = 0.5,
        wn_pos: float = 1.0 / 0.3,
        wn_ang: float = 1.0 / (2.0 * math.pi),
        tol: float = 1e-12,
    ) -> Tuple[np.ndarray, bool]:
        """Full 6D pose Levenberg-Marquardt IK — port of ROBOTIS
        `inverseSolverUsingSRJacobian`. Solves joints so the tool reaches
        `target_pos` AND `target_R`. Used for orientation jogging where we
        hold the EoE position and rotate the tool frame in place.
        """
        q = np.array(q_init, dtype=float)
        We = np.diag([wn_pos, wn_pos, wn_pos, wn_ang, wn_ang, wn_ang])
        Wn = np.eye(self.n_joints)
        tgt_pos = np.asarray(target_pos, dtype=float)

        def pose_err(qq: np.ndarray) -> np.ndarray:
            xyz, R = self.fk_pose(qq)
            ep = tgt_pos - xyz
            eo = self.orientation_difference(target_R, R)
            return np.concatenate([ep, eo])

        err = pose_err(q)
        pre_Ek = float(err.T @ We @ err)

        for _ in range(max_iter):
            J = self.jacobian(q)                      # 6 × N
            lam = pre_Ek + param
            sr = J.T @ We @ J + lam * Wn
            gerr = J.T @ We @ err
            try:
                dq = np.linalg.solve(sr, gerr)
            except np.linalg.LinAlgError:
                return q, False

            q_new = np.clip(q + dq, self.q_min, self.q_max)
            err = pose_err(q_new)
            new_Ek = float(err.T @ We @ err)

            if new_Ek < tol:
                return q_new, True
            elif new_Ek < pre_Ek:
                q = q_new
                pre_Ek = new_Ek
            else:
                q = np.clip(q + (1.0 - gamma) * dq, self.q_min, self.q_max)
                err = pose_err(q)
                pre_Ek = float(err.T @ We @ err)

        return q, False
