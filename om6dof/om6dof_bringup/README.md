# om6dof_bringup

Canonical hardware and ros2_control bringup for OM6DOF. This package is the
only component allowed to own U2D2 and the Dynamixel bus.

```text
MoveIt ------> arm_controller ---------------------------+
                                                          |
command sources --> om6dof_controller                    |
                     | conversion / IK / switching        |
                     +--> forward_position_controller ----+
                                                          v
                                             controller_manager
                                                   |
                                      dynamixel_hardware_interface
                                                   |
                                                  U2D2
```

No teleop, pick-and-place, or MoveIt node should open the serial port directly.

## Controllers

`config/controllers.yaml` defines:

| Controller | Plugin | Boot state |
|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | active |
| `arm_controller` | `JointTrajectoryController` | active |
| `forward_position_controller` | `ForwardCommandController` | inactive |
| `gripper_controller` | `GripperActionController` | active |

The two arm controllers claim the same six `position` command interfaces and
are mutually exclusive:

- `arm_controller` is the autonomous/MoveIt trajectory path.
- `forward_position_controller` is the final position-command path for remote
  JOINT, CARTESIAN, and CYLINDRICAL modes. Conversion and coordinate IK live in
  `om6dof_controller`; the hardware interface still sees only joint positions.
- `gripper_controller` remains independent and active in either state.

`om6dof_controller` asks `/controller_manager/switch_controller` to exchange
the two arm controllers atomically and is the sole publisher to
`/forward_position_controller/commands`. Teleop and other command sources use
`/om6dof/operation_mode` plus `/om6dof/control_cmd`; they never create another
hardware interface.

## Real hardware

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_bringup hardware.launch.py
```

Options:

```bash
ros2 launch om6dof_bringup hardware.launch.py \
  port_name:=/dev/ttyUSB0 baud_rate:=1000000
```

The default device uses the stable FTDI `/dev/serial/by-id/...` path.

## Fake hardware

```bash
ros2 launch om6dof_bringup hardware.launch.py use_fake_hardware:=true
```

## MoveIt wrapper

`real.launch.py` can start hardware and ordinary MoveIt together:

```bash
ros2 launch om6dof_bringup real.launch.py
```

When `om6dof-hardware.service` already owns U2D2, start only MoveIt:

```bash
ros2 launch om6dof_bringup real.launch.py start_hardware:=false
```

MoveIt uses `arm_controller`. Remote control must be disabled before executing
a plan.

## Verification

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo --once /joint_states
```

Expected initial states:

```text
joint_state_broadcaster     active
arm_controller              active
gripper_controller          active
forward_position_controller inactive
```

If `ros2_control_node` exits, `hardware.launch.py` shuts down its parent launch
so systemd can restart the complete hardware owner instead of leaving only
`robot_state_publisher` alive.

## Ownership rule

Never launch two copies of `hardware.launch.py` against the same U2D2. A second
copy cannot share the serial port and must not be used as a controller-switch
mechanism. Controller switching always occurs inside the one canonical
`controller_manager`.
