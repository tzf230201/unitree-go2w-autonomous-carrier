# om6dof_pick_and_place

Pick-and-place state machine for the 6-DOF OM6DOF. Sits on top
of `om6dof_moveit_config` (MoveIt 2) and
[`om6dof_bringup`](../om6dof_bringup/) (ros2_control + Dynamixels), and
drives a configurable waypoint sequence through the `MoveGroup` action.

The full stack:

```
om6dof_pick_and_place        ← this package (state machine, waypoints YAML)
        ↓ MoveGroup action  +  /gripper_controller/gripper_cmd action
om6dof_bringup ← move_group: OMPL planning, KDL IK, collision check
        ↓ /arm_controller/follow_joint_trajectory  +  /gripper_controller/gripper_cmd
om6dof_bringup           ← ros2_control + dxl_hw_interface
        ↓ /dev/ttyUSB0
U2D2 → 7 Dynamixel chain
```

No Python MoveIt bindings are used (`moveit_py` is not packaged for Humble,
`pymoveit2` isn't on PyPI for our arch). Instead the node talks to
`move_group` through the standard `moveit_msgs/action/MoveGroup` action.

---

## State machine

Default 11-step sequence:

```
 1. → start_pose       (named SRDF pose "rest")
 2. ↓ gripper OPEN
 3. → pick_above       (above the object)
 4. → pick             (lowered onto the object)
 5. ↓ gripper CLOSE    (grasp)
 6. → pick_above       (lift)
 7. → place_above      (above the drop spot)
 8. → place            (lowered)
 9. ↓ gripper OPEN     (release)
10. → place_above      (lift)
11. → start_pose       (return)
```

If any step fails, the sequence aborts and the worker thread exits cleanly.
The arm stops at whatever pose succeeded last.

---

## Waypoint format

Waypoints live in [`config/waypoints.yaml`](config/waypoints.yaml). Each entry
under `waypoints:` is one of three forms:

```yaml
waypoints:
  # 1. shorthand for a named SRDF group_state
  start_pose: rest

  # 2. explicit named form
  some_pose:
    named: home

  # 3. joint-space target (recommended — bypasses IK ambiguity)
  pick_above:
    joint: [-0.603, -0.413, 1.758, -2.014, -0.816, 2.083]

  # 4. Cartesian target (planner has to solve IK; can fail near singularities)
  drop_spot:
    xyz: [0.20, -0.10, 0.10]
    rpy: [0.0, 1.5708, 0.0]            # gripper pointing down
```

The order of `joint:` values is fixed: `[joint1, joint2, joint3, joint4, joint5, joint6]`.

### Why joint-space is more reliable than Cartesian

For 6-DOF arms with a spherical wrist, two distinct joint configurations can
both satisfy the same Cartesian goal. Picking the wrong one or having to pass
through a wrist singularity to reach the right one causes `MoveGroup` to fail
with the dreaded `error code 99999` (planning failure). Joint targets remove
the IK ambiguity entirely — the planner just connects two points in joint
space.

---

## Snapshot helper for tuning

Hand-writing reachable joint values is painful. So the node also exposes a
service that captures the current arm pose as a YAML snippet:

```bash
# Pose the arm to where you want it (RViz interactive marker, or by hand
# with torque off, or via teleop). Then:
ros2 service call /snapshot_waypoint std_srvs/srv/Trigger
```

Response message looks like:

```
  # paste under `waypoints:` and pick a name
  YOUR_NAME:
    joint: [-0.603, -0.413, 1.758, -2.014, -0.816, 2.083]
```

Paste the `YOUR_NAME` line into [`waypoints.yaml`](config/waypoints.yaml),
rename to `pick`, `place`, `pick_above`, etc.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select om6dof_pick_and_place --symlink-install
source install/setup.bash
```

`ament_python` package; depends on `om6dof_bringup`,
`rclpy`, `moveit_msgs`, `control_msgs`, `std_srvs`, `sensor_msgs`, `geometry_msgs`.

---

## Run

You need the full MoveIt stack up first. Two terminals:

```bash
# Terminal 1 — reuse permanent hardware; add move_group + RViz
sudo systemctl start om6dof-hardware.service
ros2 launch om6dof_bringup real.launch.py start_hardware:=false
# Wait for "You can start planning now!"
```

```bash
# Terminal 2 — the state machine
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_pick_and_place pick_place.launch.py auto_run:=false
```

Now trigger the sequence at any time:

```bash
ros2 service call /run_pick_place std_srvs/srv/Trigger
```

The service returns immediately (`success: true, message: "started"`); the
sequence runs on a worker thread and progress shows up in the pick-place
node's log.

If you set `auto_run:=true`, the sequence fires once, two seconds after the
node starts.

---

## Services

| Service                | Type                       | What it does |
|------------------------|----------------------------|--------------|
| `/run_pick_place`      | `std_srvs/srv/Trigger`     | Kick off the sequence on a worker thread. Returns immediately. Rejected (`success=false`) if a sequence is already running. |
| `/snapshot_waypoint`   | `std_srvs/srv/Trigger`     | Capture the current arm joint state and return a YAML snippet ready to paste into `waypoints.yaml`. |

---

## Parameters

| Param | Default | Notes |
|---|---|---|
| `waypoints_file`     | `<share>/config/waypoints.yaml`           | Path to the YAML |
| `srdf_file`          | auto (`om6dof_moveit_config`) | Used to resolve `named:` waypoints client-side |
| `auto_run`           | `true`                                    | Run once at startup (uses a 2-second timer) |
| `arm_group`          | `arm`                                     | SRDF planning group |
| `gripper_group`      | `gripper`                                 | SRDF planning group for the gripper |
| `ee_link`            | `end_effector_link`                       | TF link used for Cartesian pose constraints |
| `reference_frame`    | `world`                                   | Frame for `xyz`/`rpy` waypoints |
| `vel_scale`          | `0.3`                                     | MoveIt velocity scaling |
| `acc_scale`          | `0.3`                                     | MoveIt acceleration scaling |
| `planning_time`      | `10.0`                                    | Per-attempt planner timeout (s) |
| `planning_attempts`  | `20`                                      | Number of planner attempts |
| `gripper_open_pos`   | `0.019`                                   | Prismatic joint position (m) for OPEN |
| `gripper_close_pos`  | `-0.010`                                  | Prismatic joint position (m) for CLOSE |

Override at launch time, e.g.:

```bash
ros2 launch om6dof_pick_and_place pick_place.launch.py \
  auto_run:=false \
  waypoints_file:=/tmp/my_waypoints.yaml
```

---

## Architecture notes

* **Why a worker thread?** Every `MoveGroup` round-trip blocks waiting on an
  action result. If the sequence ran on the executor thread, the action's
  result callback could never fire — deadlock. We use a
  `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup` for
  services and timers, and spawn the sequence on a `threading.Thread`. The
  internal `_wait_future` helper polls with a small sleep so other executor
  threads keep dispatching incoming action responses.

* **Named-pose resolution.** The MoveGroup action server does **not** look up
  SRDF `group_state` names; the `Constraints.name` field is just a label.
  The node parses the SRDF XML at startup (from a file path resolved via
  `ament_index_python`) and turns `named: rest` into a list of
  `JointConstraint` messages before sending.

* **Loose Cartesian tolerances.** Position tolerance defaults to 3 cm,
  orientation tolerance to 0.2 rad. Tight tolerances often make Cartesian
  goals near the workspace boundary unreachable; if you need precise EE
  poses, override `position_tolerance` / `orientation_tolerance` on the
  `MoveItClient` constructor — but joint-space waypoints are almost always
  easier.

---

## Workflow: tuning waypoints for your real table

1. **Power-on, launch MoveIt:**
   ```bash
   sudo systemctl start om6dof-hardware.service
   ros2 launch om6dof_bringup real.launch.py start_hardware:=false
   ```

2. **Launch this node with `auto_run:=false`** so it just waits:
   ```bash
   ros2 launch om6dof_pick_and_place pick_place.launch.py auto_run:=false
   ```

3. **Place a fake object** at the pick spot.

4. **In RViz**, pose the arm so the gripper hovers ~10 cm above the object
   (interactive marker / joint sliders → Plan → Execute).

5. **Snapshot** the joint state:
   ```bash
   ros2 service call /snapshot_waypoint std_srvs/srv/Trigger
   ```
   Paste the printed `joint: [...]` into `waypoints.yaml` as `pick_above`.

6. **Lower the arm** to where the gripper grasps the object. Snapshot again
   → paste as `pick`.

7. **Repeat** for `place_above` and `place` at the drop location.

8. **Trigger the sequence** to verify:
   ```bash
   ros2 service call /run_pick_place std_srvs/srv/Trigger
   ```

9. If any waypoint fails (`MoveGroup failed, error code N`), check the log for
   which step, adjust that entry in `waypoints.yaml`, and re-trigger.

---

## Common error codes

| Code | Meaning | What usually causes it |
|------|---------|------------------------|
| `1`  | SUCCESS | … |
| `-1` | PLANNING_FAILED  | No collision-free path found in the time budget. Use joint goals or increase `planning_time`. |
| `-16`| FRAME_TRANSFORM_FAILURE | A required TF transform was missing — usually a malformed `Constraints` request (e.g. an empty constraint, or a named pose the server couldn't resolve). |
| `99999` | FAILURE (generic) | Catch-all for "planner gave up". Usually IK ambiguity / wrist singularity / unreachable Cartesian target. Switch to joint-space. |

---

## Perception-driven direct pickup

`direct_pick_node` is the runtime backend used by the **OM6DOF perception**
card in the Kublab web monitor. It consumes the selected YOLO target from
`/om6dof_perception/target_point`, the 3D bounding box from
`/om6dof_perception/status`, and the current arm joints. The wrist-camera
point is transformed into the arm `world` frame with FK and the calibrated
camera extrinsic in [`config/tag_pick.yaml`](config/tag_pick.yaml).

Before using **Pickup object**, start perception, set a target such as
`bottle`, turn F3 remote arm control OFF, and clear the complete arm workspace.
The direct picker runs this sequence:

1. Move to a ready pose while preserving a safe camera pan/tilt.
2. Open the gripper and centre the target in image X and Y.
3. Approach along the live three-dimensional EoE-to-object ray. The path is
   not forced to remain horizontal; low and high objects produce a sloped XYZ
   path. Default fingertip standoffs are 16, 12, and 8 cm.
4. Re-read depth and re-centre pan and tilt after every movement. A lost target
   or unstable depth cluster aborts the sequence.
5. At the final standoff, lock pan/tilt again, preserve the selected IK branch,
   and reject wrist solutions that would turn the gripper upside down.
6. Recompute the ray from the measured EoE pose, advance through short
   straight waypoints, close the gripper, retreat, move to `place_pose`, and
   release.

### Automatic upper-surface/top-pick fallback

The centre of a low object can be below the normal front-pick Z bound even
though the object is reachable from above. This is common when the arm base is
mounted above the floor. If **only** the lower Z bound is violated, the picker
does not blindly enlarge the complete workspace:

- X and Y must still be inside `perception_world_min/max`.
- The object centre must be above `top_pick_object_min_z`.
- A fresh YOLO 3D bounding box must be available.
- The upper world-Z surface is calculated from the full optical-frame box
  extent and the current wrist-camera rotation.
- The fingertip target is converted to an EoE target using `pick_offset`, and
  must remain above `top_pick_min_ee_z`.
- One continuous hover → pregrasp → grasp IK chain must be valid before any
  top-pick movement is sent.

For the current OM6DOF geometry, an exact 180-degree wrist orientation is not
IK-feasible at every bottle radius. `top_grasp_pitches` therefore starts at
2.4 rad (about 137.5 degrees) and tries steeper fallbacks afterward. The EoE
translation still descends vertically onto the upper surface. After grasping,
the arm retreats vertically before carrying the object to the drop pose.

Relevant configuration:

```yaml
auto_top_pick_enabled: true
top_pick_object_min_z: -0.25
top_pick_min_ee_z: -0.10
top_pick_hover_m: 0.13
top_pick_pregrasp_m: 0.05
top_pick_retreat_m: 0.07
top_approach_pitches: [2.0, 1.8, 2.2, 2.4]
top_grasp_pitches: [2.4, 2.7, 2.9, 3.1416]
```

Do not lower the Z limits merely to suppress an abort. First verify the camera
extrinsic, 3D-box overlay, physical support height, fingertip offset, and an IK
solution. The top-pick fallback intentionally does not bypass lateral safety.

### Object searching state

The web button **Start searching state** calls `/direct_search`. The arm first
moves to the safe ready pose, then scans:

```
low:    centre → left → right
medium: centre → left → right
high:   centre → left → right
```

Only joint1 (pan) and joint5 (tilt) are changed during the sweep. A target is
reported as `FOUND` only after it appears with a valid 3D point in 10
consecutive perception frames; one lost frame resets the streak to zero. Use
**Stop searching** or `/direct_search_stop` to cancel. Live state and the
`stable=N/10` counter are available through `/direct_search_status` and the
web monitor.

Direct-pick services:

| Service | Purpose |
|---|---|
| `/run_perception_pick` | Run the guarded perception pickup sequence. |
| `/direct_pick_status` | Current pickup step and transformed object point. |
| `/direct_track` | Continuous joint1/joint5 target tracking. |
| `/direct_stop` | Stop pickup or continuous tracking. |
| `/direct_search` | Start the low/medium/high search sweep. |
| `/direct_search_stop` | Cancel the search sweep. |
| `/direct_search_status` | Search angle/state and consecutive-frame count. |
| `/direct_reachable` | Report front-pick IK reachability for the live target. |

---

## AprilTag pick (level-1 vision PoC)

Two extra nodes make the *pick* side vision-driven while the place side
stays waypoint-based:

* `apriltag_detector` — RealSense D435i via pyrealsense2 (no ROS camera
  driver), AprilTag 36h11 via OpenCV aruco, depth-refined pose →
  `/apriltag/pose` (camera optical frame) + `/apriltag/debug_image`.
* `tag_pick_place_node` — transforms the tag into `world` through a static
  hand-measured camera extrinsic (`camera_xyz`/`camera_rpy` = camera *body*
  frame, x out of the lens, y left, z up), generates Cartesian pick poses
  (`approach_offset`/`grasp_offset` above the tag, `grasp_pitch` orientation,
  radial yaw), then runs the same 11-step sequence.

Config: [`config/tag_pick.yaml`](config/tag_pick.yaml). Target object is a
30 mm cube with AprilTag 36h11 ID 1 on one side face. A printable tag
(36h11, id 1, 30 mm) lives at
`~/ros2_ws/apriltag36h11_id1_30mm.png` — print at 100 % scale and check the
black square is exactly 30 mm. `object_center_from_tag` is `[0, 0, -0.015]`:
the cube centre is 15 mm behind the visible tag plane.

```bash
# camera is normally held by the web dashboard — release it first
sudo systemctl stop go2w-web-monitor.service

# MoveIt + detector reuse the permanent om6dof hardware service
ros2 launch om6dof_pick_and_place tag_pick_place.launch.py

# 1. verify the extrinsic: put the tag at a spot you can measure
ros2 service call /tag_world std_srvs/srv/Trigger
# 2. tune camera_xyz/camera_rpy in tag_pick.yaml until /tag_world matches
#    the ruler, then run:
ros2 service call /run_tag_pick std_srvs/srv/Trigger
```

For read-only manual coordinate debugging, leave the arm control sequence off
and only read the detector, `/joint_states`, and TF:

```bash
ros2 launch om6dof_pick_and_place tag_pick_place.launch.py start_picker:=false

# Full report: joints, EE world pose, tag/object world pose, and object-EE delta.
ros2 service call /coord_debug_snapshot std_srvs/srv/Trigger

# YAML-ready joint waypoint from the current manual arm pose.
ros2 service call /coord_debug_waypoint std_srvs/srv/Trigger
```

The launch shows the detector's OpenCV debug window by default; `rqt_image_view`
is optional with `start_image_viewer:=true`. RViz subscribes to `/tag_markers`
and `/coord_debug_markers`: the coordinate debugger shows the estimated tag,
cube centre, end effector, and the line from gripper to object. If the tag is
not visible when `/run_tag_pick` is called, the arm sweeps through the
configured `search_waypoints`; if it still cannot see the tag, it performs a
small nod gesture and aborts. After a successful pick-and-place it runs a small
wrist-circle gesture.

If a Cartesian step fails with error code `99999`, the tag is probably
outside the dexterous workspace for a straight-down grasp — try a tilted
approach (`grasp_pitch: 2.4` ≈ 45°) or move the tag closer to the arm.

---

## Possible follow-ups (not done yet)

- Add **collision objects** to the planning scene (table surface, walls)
  so the planner avoids them
- **Hand-eye calibration** instead of ruler-measured `camera_xyz`/`camera_rpy`
  (e.g. tag held in the gripper at known EE poses)
- Add richer planning-scene objects for ordinary MoveGroup planning
- Wrap the sequence in a **behavior tree** (`py_trees`, `behaviortree_cpp`)
  for retry / fallback / multi-object workflows

---

## See also

- [`om6dof_bringup`](../om6dof_bringup/) — hardware bringup
- `om6dof_moveit_config` — MoveIt 2 layer
- `om6dof_bringup` — ros2_control hardware layer
- [`om6dof_teleop`](../om6dof_teleop/) — JOINT/CARTESIAN/CYLINDRICAL remote
  control with an exclusive ros2_control switch between remote and autonomous
  arm ownership
