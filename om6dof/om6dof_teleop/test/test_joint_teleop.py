import inspect
import math
import struct
import threading

import pytest
from std_msgs.msg import Bool, String

import om6dof_teleop.joint_teleop as joint_teleop
from om6dof_controller.control_math import (
    MODE_AUTONOMOUS,
    MODE_CARTESIAN,
    MODE_CYLINDRICAL,
    MODE_JOINT,
    MODE_READY,
    MODE_STARTUP,
)
from om6dof_teleop.joint_teleop import (
    BTN_B,
    BTN_F1,
    BTN_F3,
    BTN_LEFT,
    BTN_R1,
    BTN_R2,
    BTN_RIGHT,
    BTN_SELECT,
    BTN_UP,
    BTN_X,
    BTN_Y,
    JointTeleop,
    _decode_lowstate_remote,
)


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message, **_kwargs):
        self.infos.append(str(message))

    def warn(self, message, **_kwargs):
        self.warnings.append(str(message))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _remote_payload(keys, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
    raw = bytearray(40)
    raw[0:2] = b"\x55\x51"
    raw[2:4] = int(keys).to_bytes(2, "little")
    struct.pack_into("<f", raw, 4, lx)
    struct.pack_into("<f", raw, 8, rx)
    struct.pack_into("<f", raw, 12, ry)
    struct.pack_into("<f", raw, 20, ly)
    return list(raw)


def _adapter(remote_enabled=False):
    """Build the pure adapter state without starting a ROS graph."""
    node = object.__new__(JointTeleop)
    node.lock = threading.RLock()
    node.remote_enabled = remote_enabled
    node.control_mode = MODE_JOINT
    node.f1_destination = MODE_READY if remote_enabled else None
    node.mode_request_pending_until = 0.0
    node.remote_waiting_for_neutral = True
    node.keys = 0
    node.axes = (0.0, 0.0, 0.0, 0.0)
    node.lowstate_keys = 0
    node.lowstate_axes = (0.0, 0.0, 0.0, 0.0)
    node.event_keys = 0
    node.event_axes = (0.0, 0.0, 0.0, 0.0)
    node.last_lowstate_remote = 0.0
    node.last_event_remote = 0.0
    node.last_remote_action = {}

    node.remote_timeout = 0.5
    node.lowstate_preference_timeout = 0.1
    node.action_debounce = 0.0
    node.mode_request_timeout = 2.0
    node.deadzone = 0.08
    node.joint_velocity = 0.5
    node.cartesian_linear_speed = 0.05
    node.cartesian_angular_speed = 0.5
    node.cylindrical_theta_speed = 0.25
    node.button_pairs = [
        (15, 13),
        (14, 12),
        (8, 11),
        (10, 9),
        (-1, -1),
        (0, 4),
    ]
    node.joint_axes = [-1, -1, -1, -1, 3, -1]
    node.joint_signs = [1.0] * 6

    node.operation_pub = _Publisher()
    node.control_pub = _Publisher()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.gripper_commands = []
    node._command_gripper = node.gripper_commands.append
    return node


def _sample(node, keys, source="lowstate", axes=(0.0, 0.0, 0.0, 0.0)):
    node._process_remote_sample(keys, axes, source)


def test_lowstate_decoder_preserves_unitree_axis_order_and_buttons():
    keys, axes = _decode_lowstate_remote(
        _remote_payload(0xA55A, lx=0.1, ly=-0.2, rx=0.3, ry=-0.4)
    )
    assert keys == 0xA55A
    assert axes == pytest.approx((0.1, -0.2, 0.3, -0.4))


@pytest.mark.parametrize("payload", ([0] * 39, [0] * 41))
def test_lowstate_decoder_rejects_wrong_payload_length(payload):
    with pytest.raises(ValueError, match="exactly 40 bytes"):
        _decode_lowstate_remote(payload)


def test_lowstate_decoder_rejects_nonfinite_axes():
    raw = bytearray(_remote_payload(0))
    struct.pack_into("<f", raw, 12, math.inf)
    with pytest.raises(ValueError, match="non-finite"):
        _decode_lowstate_remote(raw)


def test_joint_mode_maps_six_remote_controls_to_six_velocities():
    node = _adapter(remote_enabled=True)
    keys = BTN_RIGHT | BTN_UP | BTN_Y | BTN_B | BTN_R2
    velocity = node._remote_joint_velocity(keys, (0.0, 0.0, 0.0, 1.0))
    assert velocity == pytest.approx([0.5] * 6)


def test_joint_axis_deadzone_is_zero_and_rescaled_outside_zone():
    node = _adapter(remote_enabled=True)
    assert node._deadzone(0.079) == 0.0
    assert node._deadzone(-0.079) == 0.0
    assert node._deadzone(0.54) == pytest.approx(0.5)
    assert node._deadzone(-0.54) == pytest.approx(-0.5)


def test_cartesian_and_cylindrical_modes_have_distinct_second_coordinate():
    node = _adapter(remote_enabled=True)
    keys = BTN_Y | BTN_LEFT | BTN_UP | BTN_X | BTN_R1
    axes = (0.0, 0.0, 0.0, 1.0)

    node.control_mode = MODE_CARTESIAN
    cartesian = node._remote_coordinate_velocity(keys, axes)
    assert cartesian == pytest.approx([0.05, 0.05, 0.05, 0.5, 0.5, 0.5])

    node.control_mode = MODE_CYLINDRICAL
    cylindrical = node._remote_coordinate_velocity(keys, axes)
    assert cylindrical == pytest.approx([0.05, 0.25, 0.05, 0.5, 0.5, 0.5])


def test_f3_requests_joint_then_waits_for_confirmed_remote_state(monkeypatch):
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: 100.0)
    node = _adapter(remote_enabled=False)

    _sample(node, BTN_F3)

    assert [msg.data for msg in node.operation_pub.messages] == [MODE_JOINT]
    assert node.remote_enabled is False
    assert node.mode_request_pending_until == pytest.approx(102.0)

    node._on_remote_state(Bool(data=True))
    assert node.remote_enabled is True
    assert node.control_mode == MODE_JOINT
    assert node.f1_destination == MODE_READY
    assert node.remote_waiting_for_neutral is True


def test_duplicate_f3_from_both_unitree_sources_is_one_request(monkeypatch):
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: 100.0)
    node = _adapter(remote_enabled=False)

    _sample(node, BTN_F3, "lowstate")
    _sample(node, BTN_F3, "event")

    assert [msg.data for msg in node.operation_pub.messages] == [MODE_JOINT]


def test_f3_has_priority_over_mode_buttons_pressed_in_same_sample(monkeypatch):
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: 100.0)
    node = _adapter(remote_enabled=True)

    _sample(node, BTN_F3 | BTN_SELECT | BTN_F1)

    assert [msg.data for msg in node.operation_pub.messages] == [
        MODE_AUTONOMOUS
    ]
    assert node.control_mode == MODE_JOINT


def test_select_cycles_only_the_three_motion_modes(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: now[0])
    node = _adapter(remote_enabled=True)

    expected = [MODE_CARTESIAN, MODE_CYLINDRICAL, MODE_JOINT]
    for mode in expected:
        _sample(node, BTN_SELECT)
        assert node.operation_pub.messages[-1].data == mode
        assert node.control_mode == mode
        now[0] += 0.01
        _sample(node, 0)
        now[0] += 0.01

    assert [msg.data for msg in node.operation_pub.messages] == expected


def test_f1_alternates_from_ready_to_startup_and_back_after_confirmation(
    monkeypatch,
):
    now = [100.0]
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: now[0])
    node = _adapter(remote_enabled=True)

    _sample(node, BTN_F1)
    assert node.operation_pub.messages[-1].data == MODE_STARTUP
    node._on_operation_state(String(data=MODE_STARTUP))

    now[0] += 0.01
    _sample(node, 0)
    now[0] += 0.01
    _sample(node, BTN_F1)
    assert node.operation_pub.messages[-1].data == MODE_READY
    assert node.control_mode == MODE_JOINT


def test_confirmed_autonomous_state_disables_remote_output():
    node = _adapter(remote_enabled=True)
    node.mode_request_pending_until = 123.0

    node._on_operation_state(String(data=MODE_AUTONOMOUS.lower()))

    assert node.remote_enabled is False
    assert node.mode_request_pending_until == 0.0


def test_tick_publishes_zero_while_neutral_latched_or_input_stale(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: now[0])
    node = _adapter(remote_enabled=True)
    node.lowstate_keys = BTN_RIGHT
    node.last_lowstate_remote = 100.0

    node._tick()
    assert node.control_pub.messages[-1].data == pytest.approx([0.0] * 6)

    node.remote_waiting_for_neutral = False
    now[0] = 101.0
    node._tick()
    assert node.control_pub.messages[-1].data == pytest.approx([0.0] * 6)


def test_tick_publishes_current_mode_velocity_only_when_fresh_and_armed(
    monkeypatch,
):
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: 100.0)
    node = _adapter(remote_enabled=True)
    node.remote_waiting_for_neutral = False
    node.lowstate_keys = BTN_RIGHT
    node.lowstate_axes = (0.0, 0.0, 0.0, 0.0)
    node.last_lowstate_remote = 100.0

    node._tick()

    assert node.control_pub.messages[-1].data == pytest.approx(
        [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def test_gripper_buttons_remain_separate_from_arm_control_topics(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(joint_teleop.time, "monotonic", lambda: now[0])
    node = _adapter(remote_enabled=True)

    _sample(node, joint_teleop.BTN_L1)
    now[0] += 0.01
    _sample(node, 0)
    now[0] += 0.01
    _sample(node, joint_teleop.BTN_L2)

    assert node.gripper_commands == ["open", "close"]
    assert node.operation_pub.messages == []


def test_adapter_has_no_kinematics_controller_manager_or_final_publisher():
    source = inspect.getsource(JointTeleop)
    assert "/om6dof/operation_mode" in source
    assert "/om6dof/control_cmd" in source
    assert "controller_manager" not in source
    assert "SwitchController" not in source
    assert "/forward_position_controller/commands" not in source
    assert not hasattr(JointTeleop, "_coordinate_step_locked")
    assert not hasattr(JointTeleop, "_request_controller_mode")
