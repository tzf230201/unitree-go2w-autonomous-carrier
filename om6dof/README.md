# OM6DOF ROS 2 stack

Independent ROS 2 Humble stack for the custom six-axis OpenMANIPULATOR-derived arm.

- `om6dof_description`: URDF/Xacro, meshes, and visualization
- `om6dof_bringup`: Dynamixel ros2_control and hardware controllers
- `om6dof_moveit_config`: MoveIt 2 configuration, move_group, and RViz
- `om6dof_pick_and_place`: pick-and-place applications
- `dynamixel_hardware_interface`: vendored ros2_control hardware plugin
- `dynamixel_interfaces`: vendored Dynamixel ROS interfaces
- `DynamixelSDK`: vendored upstream SDK, including the `dynamixel_sdk` ROS package

The stack is self-contained and no longer depends on `open_manipulator_friends_ros2`.

## Local modifications to vendored `dynamixel_hardware_interface`

The vendored copy is **not** pristine upstream ROBOTIS code — it carries local
patches that fix a recurring `-3001 SYNC_READ_FAIL` on this rig. These must be
re-applied (or preserved) if `dynamixel_hardware_interface` is ever re-vendored
from upstream.

**Root cause:** `ProcessReadCommunication()` used the ros2_control cycle
`period_ms` directly as the serial rx timeout. The first controller cycle
after activation has `period_ms` ≈ 0, so the read was guaranteed to time out
even though the servos answered fine.

1. `dynamixel_hardware_interface/src/dynamixel/dynamixel.cpp` (~line 1799):
   clamp the packet timeout to a 5 ms floor —
   `port_handler->setPacketTimeout(period_ms < 5.0 ? 5.0 : period_ms);`
2. `dynamixel_hardware_interface/include/dynamixel_hardware_interface/dynamixel/dynamixel.hpp`
   (~line 218/245): added `use_fast_read_protocol_` member (default `true`)
   and `SetUseFastReadProtocol(bool)` setter, so FastSyncRead can be disabled
   in favor of plain SyncRead from the start.
3. `dynamixel_hardware_interface/src/dynamixel_hardware_interface.cpp`
   (~line 162): reads an optional `use_fast_read_protocol` hardware
   parameter from the URDF/xacro and forwards it to
   `Dynamixel::SetUseFastReadProtocol()`.

`om6dof_bringup/urdf/om6dof.ros2_control.xacro` sets `use_fast_read_protocol`
to `0` to use this opt-out.

Also required (outside this repo, not a source patch): the U2D2/FTDI adapter's
`latency_timer` must be `1` via udev rule
(`/etc/udev/rules.d/99-u2d2-latency.rules`), otherwise `-3001` recurs
regardless of the timeout floor above.
