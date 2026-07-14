import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64MultiArray, String

from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
)
from om6dof_controller.controller_node import (
    DEFAULT_JOINT_LOWER,
    DEFAULT_JOINT_UPPER,
    DEFAULT_READY_JOINT_POSITIONS,
    OM6DOFController,
)


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message, **_kwargs):
        self.infos.append(str(message))

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))

    def error(self, message, **_kwargs):
        self.errors.append(str(message))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Future:
    def __init__(self):
        self.callback = None
        self.response = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self.response

    def finish(self, response):
        self.response = response
        self.callback(self)


class _SwitchClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = _Future()
        self.futures.append(future)
        return future


class _UnavailableListClient:
    def service_is_ready(self):
        return False


class _IdentityIK:
    def fk_pose(self, q):
        values = np.asarray(q, dtype=float)
        return values[:3].copy(), np.eye(3)

    def velocity_ik_priority(self, _q, linear, angular):
        return np.concatenate([linear, angular])

    def ee_to_base_angular(self, _q, angular):
        return np.asarray(angular, dtype=float)

    def self_collides(self, _q, _radius):
        return False

    def manipulability(self, _q):
        return 1.0


def _controller(remote_enabled=False):
    node = object.__new__(OM6DOFController)
    node.lock = threading.RLock()
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.arm_controller = "arm_controller"
    node.remote_controller = "forward_position_controller"
    node.motion_mode = MODE_JOINT if remote_enabled else MODE_AUTONOMOUS
    node.remote_enabled = remote_enabled
    node.arm_controller_active = not remote_enabled
    node.remote_controller_active = remote_enabled
    node.switch_in_progress = False
    node.switch_target = None
    node.pending_manual_mode = MODE_JOINT
    node.remote_enabled_on_start = False
    node.startup_switch_attempted = False
    node.controller_list_future = None
    node.next_controller_poll = 0.0

    initial = [0.1, -1.9, 1.8, 0.2, 2.0, -0.1]
    node.joint_positions = dict(zip(node.joint_names, initial))
    node.last_joint_state = time.monotonic()
    node.startup_pose = list(initial)
    node.command_positions = list(initial) if remote_enabled else None
    node.control_velocity = np.zeros(6)
    node.last_control_cmd = 0.0
    node.last_tick = time.monotonic() - 0.02

    node.pose_target = None
    node.pose_operation = None
    node.pose_target_until = 0.0
    node.post_pose_mode = MODE_JOINT
    node.ready_pending_on_enable = False
    node.ready_pending_mode = MODE_JOINT

    node.joint_state_timeout = 1.0
    node.control_cmd_timeout = 0.3
    node.max_joint_velocity = 1.2
    node.joint_lower = [value + 0.02 for value in DEFAULT_JOINT_LOWER]
    node.joint_upper = [value - 0.02 for value in DEFAULT_JOINT_UPPER]
    node.ready_pose = list(DEFAULT_READY_JOINT_POSITIONS)
    node.pose_target_velocity = 0.5
    node.pose_target_tolerance = 0.01
    node.pose_target_timeout = 20.0

    node.max_cartesian_linear_velocity = 0.1
    node.max_cartesian_angular_velocity = 1.0
    node.max_cylindrical_theta_velocity = 0.5
    node.cylindrical_origin_xy = np.array([0.012, 0.0])
    node.cylindrical_min_radius = 0.03
    node.ik_position_gain = 4.0
    node.ik_rotation_gain = 3.0
    node.ik_tool_frame_rotation = True
    node.ik_max_target_lead = 0.04
    node.ik_max_joint_following_error = 0.30
    node.ik_manipulability_warning_threshold = 1.0e-6
    node.ik_self_collision = False
    node.ik_collision_radius = 0.025
    node.ik_collision_blocked = False
    node.ik = _IdentityIK()
    node.ik_target_pos = None
    node.ik_target_rotation = None
    node.cylindrical_theta_hint = None

    node.command_pub = _Publisher()
    node.operation_state_pub = _Publisher()
    node.remote_state_pub = _Publisher()
    node.switch_client = _SwitchClient()
    node.list_client = _UnavailableListClient()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.get_parameter = lambda name: SimpleNamespace(
        value={"switch_timeout_seconds": 2.0}[name]
    )
    return node


def test_joint_mode_switches_only_after_success_and_schedules_ready():
    node = _controller(remote_enabled=False)

    node._on_operation_mode(String(data="JOINT"))

    assert node.remote_enabled is False
    request = node.switch_client.requests[-1]
    assert request.activate_controllers == ["forward_position_controller"]
    assert request.deactivate_controllers == ["arm_controller"]
    assert request.strictness == SwitchController.Request.STRICT
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=True))
    assert node.remote_enabled is True
    assert node.pose_operation == MODE_READY
    assert node.pose_target == pytest.approx(DEFAULT_READY_JOINT_POSITIONS)
    assert node.command_pub.messages[-1].data == pytest.approx(node.startup_pose)


def test_failed_switch_never_arms_ready():
    node = _controller(remote_enabled=False)
    node._on_operation_mode(String(data="JOINT"))
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=False))
    assert node.remote_enabled is False
    assert node.pose_target is None
    assert node._logger.errors


def test_coordinate_mode_requires_manual_ownership():
    node = _controller(remote_enabled=False)
    node._on_operation_mode(String(data="CARTESIAN"))
    assert node.motion_mode == MODE_AUTONOMOUS
    assert not node.switch_client.requests
    assert any("request JOINT first" in msg for msg in node._logger.warnings)


def test_mode_change_clears_old_command():
    node = _controller(remote_enabled=True)
    node.control_velocity = np.ones(6)
    node.last_control_cmd = time.monotonic()
    node._on_operation_mode(String(data="CARTESIAN"))
    assert node.motion_mode == MODE_CARTESIAN
    assert node.last_control_cmd == 0.0
    assert node.control_velocity == pytest.approx(np.zeros(6))


def test_joint_velocity_stream_integrates_then_watchdog_holds_feedback():
    node = _controller(remote_enabled=True)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.5, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    moved = node.command_pub.messages[-1].data
    assert moved[0] > before[0]

    node.last_control_cmd = time.monotonic() - 1.0
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data == pytest.approx(node.startup_pose)


def test_cartesian_velocity_resolves_to_joint_positions():
    node = _controller(remote_enabled=True)
    node.motion_mode = MODE_CARTESIAN
    node._seed_ik_anchor_locked(node.command_positions)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.02, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data[0] > before[0]


def test_cylindrical_velocity_resolves_to_joint_positions():
    node = _controller(remote_enabled=True)
    node.motion_mode = MODE_CYLINDRICAL
    node._seed_ik_anchor_locked(node.command_positions)
    before = list(node.command_positions)
    node._on_control_cmd(Float64MultiArray(data=[0.02, 0, 0, 0, 0, 0]))
    node.last_tick = time.monotonic() - 0.02
    node._tick()
    assert node.command_pub.messages[-1].data[0] > before[0]


def test_ready_and_startup_are_transient_joint_pose_operations():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="READY"))
    assert node.pose_operation == MODE_READY
    assert node.motion_mode == MODE_JOINT
    node._on_operation_mode(String(data="STARTUP"))
    assert node.pose_target == pytest.approx(node.startup_pose)
    assert node.motion_mode == MODE_JOINT


def test_nonzero_joint_command_interrupts_pose_but_zero_does_not():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="READY"))

    node._on_control_cmd(Float64MultiArray(data=[0.0] * 6))
    assert node.pose_operation == MODE_READY
    assert node.last_control_cmd == 0.0

    node._on_control_cmd(Float64MultiArray(data=[0.2, 0, 0, 0, 0, 0]))
    assert node.pose_target is None
    assert node.pose_operation is None
    assert node.motion_mode == MODE_JOINT
    assert node.control_velocity == pytest.approx([0.2, 0, 0, 0, 0, 0])
    assert node.last_control_cmd > 0.0
    assert any("interrupted" in message for message in node._logger.infos)


def test_autonomous_restores_trajectory_controller():
    node = _controller(remote_enabled=True)
    node._on_operation_mode(String(data="AUTONOMOUS"))
    request = node.switch_client.requests[-1]
    assert request.activate_controllers == ["arm_controller"]
    assert request.deactivate_controllers == ["forward_position_controller"]
    node.switch_client.futures[-1].finish(SimpleNamespace(ok=True))
    assert node.remote_enabled is False
    assert node.motion_mode == MODE_AUTONOMOUS


@pytest.mark.parametrize(
    "values",
    ([0.0] * 5, [0.0] * 7, [0, 0, np.nan, 0, 0, 0]),
)
def test_invalid_control_command_is_rejected(values):
    node = _controller(remote_enabled=True)
    node._on_control_cmd(Float64MultiArray(data=values))
    assert node.last_control_cmd == 0.0
    assert node._logger.warnings
