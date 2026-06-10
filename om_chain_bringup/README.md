# om_chain_bringup

URDF + ros2_control configuration for the 6-DOF **OpenManipulator Chain** arm
on this rig. Brings up the Dynamixel hardware (7 motors with custom IDs) and
loads the standard ROS 2 trajectory + gripper controllers.

This is the **bottom layer** of the MoveIt stack:

```
om_chain_pick_place        ← state machine (this lives elsewhere)
        ↓ MoveGroup action
om_chain_moveit_config     ← move_group, planners, RViz config
        ↓ /arm_controller/follow_joint_trajectory
om_chain_bringup           ← ros2_control + dxl_hw_interface     ← (you are here)
        ↓ /dev/ttyUSB0
U2D2 → 7 Dynamixel chain
```

If you only need joint-space teleop without MoveIt, use
[`go2w_remote_arm`](../go2w_remote_arm/) instead — it talks to the same motors
directly via dynamixel_sdk and is much lighter.

---

## Hardware mapping

| Joint  | Model       | Dynamixel ID |
|--------|-------------|--------------|
| joint1 | XM430-W350  | **31** |
| joint2 | XM430-W350  | **32** |
| joint3 | XM430-W350  | **33** |
| joint4 | XM430-W210  | **24** |
| joint5 | XM430-W350  | **35** |
| joint6 | XM430-W210  | **26** |
| gripper (revolute → prismatic) | XM430-W350 | **37** |

`gripper_sub` is a URDF `<mimic>` joint on `gripper` (paired finger) — it does
not get its own motor.

ID scheme on this rig: XM430-W350 → 30s, XM430-W210 → 20s, last digit mirrors
the joint number. Different from stock ROBOTIS Chain (1–7) and OM-X (11–15).

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select om_chain_bringup --symlink-install
source install/setup.bash
```

Build dependencies (declared in `package.xml`):

- `ament_cmake`
- runtime: `xacro`, `robot_state_publisher`, `controller_manager`,
  `joint_state_broadcaster`, `joint_trajectory_controller`,
  `position_controllers`, `dynamixel_hardware_interface`

---

## Layout

```
om_chain_bringup/
├── urdf/
│   ├── om_chain.urdf.xacro                  # top-level, includes the other two
│   ├── om_chain_description.urdf.xacro      # kinematic chain (ported from open_manipulator_friends, ROS-1 → ROS-2)
│   ├── om_chain_ros2_control.xacro          # dxl_hw_interface plugin + 7 motors at custom IDs
│   └── materials.xacro
├── meshes/                                  # 9 STL files from ROBOTIS
├── config/
│   └── controllers.yaml                     # arm_controller + gripper_controller + joint_state_broadcaster
└── launch/
    └── hardware.launch.py                   # everything you need
```

---

## Run

Make sure no other process owns `/dev/ttyUSB0` (stop `go2w-arm-launcher.service`
or kill any `teleop_node` first). Then:

```bash
ros2 launch om_chain_bringup hardware.launch.py
```

What happens:

1. `xacro` renders the URDF with our custom IDs
2. `ros2_control_node` loads the URDF as `robot_description`
3. `dxl_hw_interface` opens the port, pings all 7 IDs, configures Drive Mode /
   PID / profile, enables torque
4. `joint_state_broadcaster` activates → `/joint_states` starts publishing
5. `arm_controller` (joint_trajectory_controller) activates → can receive
   `/arm_controller/follow_joint_trajectory` action goals
6. `gripper_controller` (position_controllers/GripperActionController)
   activates → can receive `/gripper_controller/gripper_cmd` action goals
7. `robot_state_publisher` publishes the TF tree

---

## Verify

In a second terminal:

```bash
source /home/unitree/ros2_ws/install/setup.bash

# Controllers should all be `active`
ros2 control list_controllers
# joint_state_broadcaster  ...  active
# gripper_controller        ...  active
# arm_controller            ...  active

# Live joint state (7 joints — gripper_sub is mimic, hidden)
ros2 topic echo /joint_states --once

# Manual trajectory test — small joint1 wiggle:
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],
    points: [{positions: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
              time_from_start: {sec: 3, nanosec: 0}}]}}"
```

---

## Launch arguments

| Arg                  | Default              | Notes                                      |
|----------------------|----------------------|--------------------------------------------|
| `port_name`          | `/dev/ttyUSB0`       | U2D2 serial device                          |
| `baudrate`           | `1000000`            | DYNAMIXEL Protocol 2.0 baud                 |
| `use_fake_hardware`  | `false`              | If `true`, uses `fake_components/GenericSystem` — handy for testing the MoveIt stack without the arm |

```bash
ros2 launch om_chain_bringup hardware.launch.py use_fake_hardware:=true
```

---

## Gotchas

| Symptom | Cause / Fix |
|---|---|
| `Sync read fail rc=-3001` | FTDI latency_timer not at 1 ms. Apply udev rule `SUBSYSTEM=="usb-serial", DRIVERS=="ftdi_sio", ATTR{latency_timer}="1"`. |
| `ping ID xx failed` | Motor not powered, daisy-chain disconnected, or ID mismatch. Verify with `python3 -c "from dynamixel_sdk import *; p=PortHandler('/dev/ttyUSB0'); k=PacketHandler(2.0); p.openPort(); p.setBaudRate(1000000); print(k.broadcastPing(p))"`. |
| `HW error 0x...` | Hardware Error flag set (electrical-shock / overheat / etc.). Reboot the motor through DYNAMIXEL Wizard, or restart the launch — `disable_torque_at_init:=true` keeps things safe. |
| Gripper opens/closes the wrong direction or never reaches the goal | The revolute → prismatic mapping in `om_chain_ros2_control.xacro` (`revolute_max`/`revolute_min`/`prismatic_max`/`prismatic_min`) may not match your physical mechanism. Calibrate by hand and update those values. |

---

## Why not the stock ROBOTIS bringup?

`open_manipulator_x_bringup` (deleted from this workspace earlier) is hardwired
for the 4-DOF OM-X arm with IDs 11–15. The 6-DOF Chain has different links,
different joint count, different motor IDs, and a different gripper, so it
needs its own bringup. We kept the URDF geometry from
[open_manipulator_friends](https://github.com/ROBOTIS-GIT/open_manipulator_friends)
(stripping ROS 1 `<transmission>` tags) and replaced the hardware section with
our own ros2_control xacro.

---

## See also

- [`om_chain_moveit_config`](../om_chain_moveit_config/) — MoveIt 2 layer on top
- [`om_chain_pick_place`](../om_chain_pick_place/) — pick-and-place state machine
- [`go2w_remote_arm`](../go2w_remote_arm/) — alternative direct-DXL teleop (no ros2_control)
