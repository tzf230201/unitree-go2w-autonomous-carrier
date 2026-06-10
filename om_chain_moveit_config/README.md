# om_chain_moveit_config

MoveIt 2 configuration for the 6-DOF OpenManipulator Chain. Pairs with
[`om_chain_bringup`](../om_chain_bringup/) (which owns the ros2_control side)
to provide motion planning, IK, collision checking, and RViz visualization.

```
om_chain_pick_place        ← state machine
        ↓ MoveGroup action
om_chain_moveit_config     ← (you are here)
        ↓ /arm_controller/follow_joint_trajectory
om_chain_bringup
        ↓
arm hardware
```

The SRDF, planning groups, and disable-collisions pairs are adapted from
the ROS 1 [open_manipulator_friends](https://github.com/ROBOTIS-GIT/open_manipulator_friends)
package, then split into ROS 2 yaml + plain SRDF (no setup-assistant-generated
launch files).

---

## What's inside

```
om_chain_moveit_config/
├── srdf/
│   └── om_chain.srdf                     # planning groups, EE, group_states, virtual_joint, disable_collisions
├── config/
│   ├── kinematics.yaml                   # KDL plugin for the `arm` group
│   ├── joint_limits.yaml                 # MoveIt-side velocity/accel scaling
│   ├── moveit_controllers.yaml           # bind to arm_controller + gripper_controller
│   ├── ompl_planning.yaml                # RRTConnect / RRTstar / PRMstar
│   ├── pilz_cartesian_limits.yaml        # for Pilz industrial planner (optional)
│   └── sensors_3d.yaml                   # empty stub (no depth sensor yet)
└── launch/
    ├── move_group.launch.py              # just move_group
    ├── moveit_rviz.launch.py             # RViz with MotionPlanning panel preconfigured
    ├── demo.launch.py                    # all-in-one: hardware + move_group + RViz
    └── moveit.rviz                       # the saved panel/views/plugin config
```

---

## Planning groups

| Group     | Joints                          | Tip link            |
|-----------|---------------------------------|---------------------|
| `arm`     | joint1..joint6 (chain)          | `link7`             |
| `gripper` | `gripper`, `gripper_sub`        | end_effector group  |
| End-effector | `hand` (parent group `arm`, group `gripper`) | parent link `link7` |

Passive joint: `gripper_sub` (URDF mimic).

Named poses (group_states):

| Name | Group   | Joints |
|------|---------|--------|
| `home` | `arm`   | all zeros |
| `rest` | `arm`   | `[0, -0.6806, 1.3613, 0, 0.8901, 0]` |
| `open` | `gripper` | `gripper: 0.019` |
| `close` | `gripper` | `gripper: -0.010` |

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select om_chain_moveit_config --symlink-install
source install/setup.bash
```

---

## Run

### All-in-one (most common)

```bash
sudo systemctl stop go2w-arm-launcher.service   # release /dev/ttyUSB0 first
ros2 launch om_chain_moveit_config demo.launch.py
```

This brings up:

1. `om_chain_bringup hardware.launch.py` — ros2_control + Dynamixels
2. (4-second timer) `om_chain_moveit_config move_group.launch.py` — move_group with OMPL
3. (6-second timer) `om_chain_moveit_config moveit_rviz.launch.py` — RViz + MotionPlanning

When you see

```
You can start planning now!
```

…RViz should display the arm. The **MotionPlanning** panel is pre-loaded with
`Planning Group = arm`.

### Just MoveIt (hardware already running)

```bash
ros2 launch om_chain_moveit_config move_group.launch.py
ros2 launch om_chain_moveit_config moveit_rviz.launch.py   # in another shell
```

---

## Try it in RViz

In the MotionPlanning panel:

1. **Planning tab → Goal State** dropdown → pick `rest`, `home`, `open`, `close`
2. Click **Plan** → orange preview trajectory should appear
3. Click **Execute** → arm physically moves

To plan to a custom EE pose:

1. **Planning Request** → set **Interactive Marker Size** to e.g. `0.20`
2. A 6-DOF marker appears at the EE — drag it
3. **Plan** → **Execute**

---

## Tuning knobs

| File | What to change |
|---|---|
| `joint_limits.yaml` | `default_velocity_scaling_factor` / `default_acceleration_scaling_factor` (currently `0.3` = slow & safe) |
| `kinematics.yaml`   | Swap KDL for TracIK (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`) if IK fails near singularities. Requires `ros-humble-trac-ik-kinematics-plugin`. |
| `ompl_planning.yaml`| Change `default_planner_config` per group, increase planning time, add planner types |
| `srdf`              | Add more `<disable_collisions>` pairs if a real collision keeps blocking valid motions |

---

## Known gotchas

| Symptom | Likely cause |
|---|---|
| RViz starts but the arm is at the origin (un-posed) | `joint_states` not arriving — check that `hardware.launch.py` ran first and that `joint_state_broadcaster` is `active` |
| `MoveGroup failed, error code -16 (FRAME_TRANSFORM_FAILURE)` | Stale TF or named target not resolved client-side — see [`om_chain_pick_place`](../om_chain_pick_place/) for the SRDF-from-file fix |
| `MoveGroup failed, error code 99999` on Cartesian goals | IK ambiguity / singularity at wrist; switch to a joint-space goal (use the snapshot helper in [`om_chain_pick_place`](../om_chain_pick_place/)) |
| `RobotModel` display shows no robot | TF root mismatch — SRDF's `virtual_joint` ties the URDF root link `world` to the TF frame `world`. Set **Fixed Frame** to `world` in RViz Global Options |
| Trajectory accepted but motors don't move | controller manager is up but `arm_controller` is in error state — `ros2 control list_controllers` should show it `active` |

---

## See also

- [`om_chain_bringup`](../om_chain_bringup/) — ros2_control + Dynamixel hardware
- [`om_chain_pick_place`](../om_chain_pick_place/) — state machine that drives MoveGroup
