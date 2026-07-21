"""Thin synchronous wrapper around the MoveGroup and GripperCommand actions.

`pymoveit2` is not available on PyPI for our Python/arch combination and the
official `moveit_py` bindings only ship from Iron onward. So we drive
move_group directly through the standard `moveit_msgs/action/MoveGroup`
goal + `control_msgs/action/GripperCommand` goal.

This wrapper is intentionally small — enough to:
  - send a *named target* goal (group_state from the SRDF, e.g. "home", "rest")
  - send a *joint values* goal (list of 6 floats for the arm)
  - send a *position* or *pose* goal (geometry_msgs/Pose in `world` frame)
  - command the gripper to an absolute prismatic position

Async control flow: every method blocks until the action finishes; result is
returned as a bool (success/failure) so the caller's state machine can branch.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


DEFAULT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@dataclass
class JointTarget:
    """A named pose (from SRDF group_state) or explicit joint values."""
    name: Optional[str] = None
    values: Optional[List[float]] = None


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class MoveItClient:
    """Synchronous helpers driving move_group and the gripper action."""

    def __init__(
        self,
        node: Node,
        arm_group: str = "arm",
        gripper_group: str = "gripper",
        ee_link: str = "end_effector_link",
        reference_frame: str = "world",
        arm_joint_names: Iterable[str] = DEFAULT_ARM_JOINTS,
        planning_time: float = 5.0,
        num_planning_attempts: int = 10,
        max_velocity_scaling: float = 0.3,
        max_acceleration_scaling: float = 0.3,
        position_tolerance: float = 0.03,
        orientation_tolerance: float = 0.20,
        joint_tolerance: float = 0.01,
        gripper_action_name: str = "/gripper_controller/gripper_cmd",
    ) -> None:
        self.node = node
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.ee_link = ee_link
        self.reference_frame = reference_frame
        self.arm_joint_names = list(arm_joint_names)
        self.planning_time = planning_time
        self.num_planning_attempts = num_planning_attempts
        self.vel_scale = max_velocity_scaling
        self.acc_scale = max_acceleration_scaling
        self.pos_tol = position_tolerance
        self.ori_tol = orientation_tolerance
        self.joint_tol = joint_tolerance

        self._move_client = ActionClient(node, MoveGroup, "/move_action")
        self._grip_client = ActionClient(node, GripperCommand, gripper_action_name)

    # ---------------- lifecycle ----------------
    def wait_for_move_server(self, timeout_sec: float = 30.0) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("MoveGroup action server not available")
            return False
        return True

    def wait_for_servers(self, timeout_sec: float = 30.0) -> bool:
        if not self.wait_for_move_server(timeout_sec=timeout_sec):
            return False
        if not self._grip_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error("GripperCommand action server not available")
            return False
        return True

    # ---------------- helpers ----------------
    def _workspace(self) -> WorkspaceParameters:
        ws = WorkspaceParameters()
        ws.header.frame_id = self.reference_frame
        ws.min_corner.x = -1.0
        ws.min_corner.y = -1.0
        ws.min_corner.z = -0.5
        ws.max_corner.x = 1.0
        ws.max_corner.y = 1.0
        ws.max_corner.z = 1.5
        return ws

    def _new_plan_request(self, group: str) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.workspace_parameters = self._workspace()
        req.group_name = group
        req.num_planning_attempts = self.num_planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        return req

    def _wait_future(self, future: Future, timeout_sec: float = 60.0) -> bool:
        """Block until `future` is done. Must be called from a thread that is
        NOT the executor's main spin thread. Works under MultiThreadedExecutor
        because other threads keep processing rclpy callbacks (action client
        responses) while we sleep here."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return future.done()

    def _send_move_goal(self, plan_req: MotionPlanRequest) -> bool:
        goal = MoveGroup.Goal()
        goal.request = plan_req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 1

        self.node.get_logger().info(
            f"Planning {plan_req.group_name} (vel={self.vel_scale} acc={self.acc_scale})"
        )
        send_future = self._move_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=10.0):
            self.node.get_logger().error("Timed out waiting for MoveGroup goal acceptance")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected")
            return False

        result_future = handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=120.0):
            self.node.get_logger().error("Timed out waiting for MoveGroup result")
            return False
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("MoveGroup returned no result")
            return False
        # MoveItErrorCodes.SUCCESS = 1
        code = result.result.error_code.val
        ok = (code == 1)
        if not ok:
            self.node.get_logger().error(f"MoveGroup failed, error code {code}")
        return ok

    # ---------------- public API ----------------
    def move_to_named_pose(
        self,
        name: str,
        named_poses: Optional[dict] = None,
        group: Optional[str] = None,
    ) -> bool:
        """Move to an SRDF group_state by name. Caller must supply the
        `named_poses` dict {state_name: {joint_name: value, ...}} because the
        MoveGroup action server does NOT resolve SRDF names server-side."""
        if not named_poses or name not in named_poses:
            self.node.get_logger().error(
                f"Named pose '{name}' not in supplied named_poses dict "
                f"({list((named_poses or {}).keys())})"
            )
            return False
        joint_map = named_poses[name]
        names = list(joint_map.keys())
        values = [float(joint_map[n]) for n in names]
        return self.move_to_joint_values(values, joint_names=names, group=group)

    def move_to_joint_values(
        self,
        values: List[float],
        joint_names: Optional[List[str]] = None,
        group: Optional[str] = None,
    ) -> bool:
        group = group or self.arm_group
        names = joint_names if joint_names is not None else self.arm_joint_names
        if len(values) != len(names):
            raise ValueError(
                f"values has {len(values)} entries but {len(names)} joint names"
            )
        req = self._new_plan_request(group)
        gc = Constraints()
        for n, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = self.joint_tol
            jc.tolerance_below = self.joint_tol
            jc.weight = 1.0
            gc.joint_constraints.append(jc)
        req.goal_constraints.append(gc)
        return self._send_move_goal(req)

    def move_to_pose(
        self,
        pose: Pose,
        ee_link: Optional[str] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> bool:
        link = ee_link or self.ee_link
        pos_tol = self.pos_tol if position_tolerance is None else float(position_tolerance)
        ori_tol = self.ori_tol if orientation_tolerance is None else float(orientation_tolerance)
        req = self._new_plan_request(self.arm_group)
        gc = Constraints()

        # position constraint = small sphere at the target position
        pc = PositionConstraint()
        pc.header.frame_id = self.reference_frame
        pc.link_name = link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives.append(sphere)
        ps = PoseStamped()
        ps.header.frame_id = self.reference_frame
        ps.pose = pose
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        gc.position_constraints.append(pc)

        # orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.reference_frame
        oc.link_name = link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        gc.orientation_constraints.append(oc)

        req.goal_constraints.append(gc)
        return self._send_move_goal(req)

    def move_to_position(
        self,
        pose: Pose,
        ee_link: Optional[str] = None,
        position_tolerance: Optional[float] = None,
    ) -> bool:
        """Move the tool point without constraining its orientation.

        This is useful for a staged grasp: first place the gripper in front
        of the object, then issue a second pose goal that aligns orientation
        at exactly the same position.
        """
        link = ee_link or self.ee_link
        pos_tol = self.pos_tol if position_tolerance is None else float(position_tolerance)
        req = self._new_plan_request(self.arm_group)
        gc = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = self.reference_frame
        pc.link_name = link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        gc.position_constraints.append(pc)

        req.goal_constraints.append(gc)
        return self._send_move_goal(req)

    def move_to_xyz_rpy(
        self,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> bool:
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation = quat_from_rpy(roll, pitch, yaw)
        return self.move_to_pose(
            p,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )

    def set_gripper(self, position: float, max_effort: float = 5.0) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.node.get_logger().info(f"Gripper → {position:+.4f}")
        send_future = self._grip_client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_sec=5.0):
            self.node.get_logger().error("Timed out waiting for Gripper goal acceptance")
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("GripperCommand goal rejected")
            return False
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=10.0):
            self.node.get_logger().error("Timed out waiting for Gripper result")
            return False
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("GripperCommand returned no result")
            return False
        # The reached_goal flag isn't strictly required for success (the gripper
        # may stall at an object before reaching the commanded position).
        return True
