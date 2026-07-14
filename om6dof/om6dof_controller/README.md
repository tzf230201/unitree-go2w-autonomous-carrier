# om6dof_controller

High-level command converter between generic OM6DOF jog commands and the
existing ros2_control `forward_position_controller`. This package does not
open U2D2 and is not a ros2_control plugin; `om6dof_bringup` remains the single
hardware owner. It is the sole publisher of final six-joint targets on
`/forward_position_controller/commands`.

## Command contract

The two canonical arm-command topics are:

| Topic | Type | Meaning |
|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | Select command interpretation |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | Exactly six finite velocity values |

`operation_mode` accepts:

- `JOINT`: `[q1_dot .. q6_dot]` in rad/s. From autonomous ownership this first
  switches controllers and ramps the arm to READY.
- `CARTESIAN`: `[vx, vy, vz, roll_dot, pitch_dot, yaw_dot]`; translation is in
  the world/base frame, rotation is in the tool frame.
- `CYLINDRICAL`: `[radius_dot, theta_dot, z_dot, roll_dot, pitch_dot, yaw_dot]`.
- `AUTONOMOUS`: restore `arm_controller` for MoveIt.
- `READY` or `STARTUP`: transient joint-space pose request; effective mode
  returns to JOINT after reaching the pose.

The command stream expires after 0.3 seconds. On timeout the target is reseeded
from measured joint feedback, so an old velocity never continues moving.

Linear velocities and `radius_dot` use m/s. Joint and angular velocities,
including `theta_dot`, use rad/s. `operation_mode` is a discrete request;
`control_cmd` is a velocity stream and should normally be published at 20--50
Hz. The gripper is intentionally outside this six-axis command contract and
continues to use `/gripper_controller/gripper_cmd`.

Confirmed read-only state is published on:

- `/om6dof/operation_mode/state`
- `/om6dof/remote_enabled/state`

Both state topics use reliable, transient-local QoS. `remote_enabled` becomes
true only after `forward_position_controller` owns the arm interfaces; the
operation-mode state reports the mode actually accepted by the controller.

## Starting

Start `om6dof_bringup` first, then:

```bash
ros2 launch om6dof_controller controller.launch.py
```

The complete hardware + controller + Go2W adapter can instead be started with:

```bash
ros2 launch om6dof_teleop full_stack.launch.py
```

## Examples

Enable remote ownership and enter READY:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

Joint jog:

```bash
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0, 0, 0, 0, 0]}"
```

Cartesian +X:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: CARTESIAN}"
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0, 0, 0, 0, 0]}"
```

Return ownership to MoveIt:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"
```

Multiple publishers on `/om6dof/control_cmd` are last-writer-wins; they are not
summed. A future teleop/autonomy overlay must use a deliberate command mux.
