"""Backward-compatible IK import.

Kinematics are owned by :mod:`om6dof_controller`.  Keep this module as a
small compatibility bridge for out-of-tree programs that still import the
old teleop path; new code should import ``om6dof_controller.ik_solver``.
"""

from om6dof_controller.ik_solver import (  # noqa: F401
    IKSolver,
    _kdl_chain_from_urdf,
    render_chain_urdf,
)

__all__ = ["IKSolver", "render_chain_urdf"]
