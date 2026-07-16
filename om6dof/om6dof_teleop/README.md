# om6dof_teleop

Go2W remote-input adapter for OM6DOF. This package translates joystick samples
into the generic command contract exposed by `om6dof_controller`; it does not
perform kinematics, switch ros2_control controllers, publish final joint
positions, open U2D2, or call Dynamixel SDK.

```text
/lowstate.wireless_remote ----+
                              |
/wirelesscontroller ----------+--> om6dof_teleop
                                      |  /om6dof/operation_mode
                                      |  /om6dof/control_cmd
                                      v
                               om6dof_controller
                                 | conversion / IK / limits
                                 | controller switching
                                 v
                      forward_position_controller
                                 v
                         om6dof_bringup
```

`om6dof_controller` is the only publisher to
`/forward_position_controller/commands`. The gripper is separate from the
six-axis arm command stream and uses the standard ros2_control
`GripperActionController`.

## Starting

Build the complete path:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  om6dof_bringup om6dof_controller om6dof_teleop --symlink-install
source install/setup.bash
```

Start hardware, the command converter, and the Go2W adapter together:

```bash
source ~/unitree_ros2/setup.sh
source ~/ros2_ws/install/setup.bash
ros2 launch om6dof_teleop full_stack.launch.py
```

`full_stack.launch.py` contains exactly one
`om6dof_bringup/hardware.launch.py`. Do not run a second hardware bringup on
the same U2D2.

If hardware is already running, start only the remaining layers:

```bash
ros2 launch om6dof_controller controller.launch.py
ros2 launch om6dof_teleop teleop.launch.py
```

The second launch is lightweight and can be restarted without reopening the
Dynamixel bus. Arm commands require `om6dof_controller` to be running.

## Canonical arm command API

The adapter publishes the same two topics available to any other command
source:

| Topic | Type | Meaning |
|---|---|---|
| `/om6dof/operation_mode` | `std_msgs/msg/String` | `JOINT`, `CARTESIAN`, `CYLINDRICAL`, `AUTONOMOUS`, `READY`, or `STARTUP` |
| `/om6dof/control_cmd` | `std_msgs/msg/Float64MultiArray` | Exactly six velocity values interpreted by the selected mode |

The six command values are:

| Mode | `control_cmd.data` | Units |
|---|---|---|
| `JOINT` | `[q1_dot, q2_dot, q3_dot, q4_dot, q5_dot, q6_dot]` | rad/s |
| `CARTESIAN` | `[vx, vy, vz, roll_dot, pitch_dot, yaw_dot]` | m/s, rad/s |
| `CYLINDRICAL` | `[radius_dot, theta_dot, z_dot, roll_dot, pitch_dot, yaw_dot]` | m/s, rad/s |

Confirmed controller state is read from reliable, transient-local topics:

| Topic | Type | Meaning |
|---|---|---|
| `/om6dof/operation_mode/state` | `std_msgs/msg/String` | Mode actually accepted by `om6dof_controller` |
| `/om6dof/remote_enabled/state` | `std_msgs/msg/Bool` | Whether `forward_position_controller` owns the arm |

The command topic is a stream, not a target pose. Teleop publishes at 50 Hz
while remote ownership is active. Stale or non-neutral input produces a zero
velocity command, and the controller has its own command watchdog.

Multiple publishers on `/om6dof/control_cmd` are last-writer-wins; their values
are not added. Use a deliberate command mux if remote and autonomous velocity
sources must contribute simultaneously.

## ROS topic control without the remote

Do not run these examples while the teleop adapter is active. Once remote
ownership is enabled, the adapter publishes `/om6dof/control_cmd` at 50 Hz,
including zero commands for a neutral joystick. A second publisher would race
with it (last-writer-wins). Run `om6dof_bringup` plus `om6dof_controller`
without `om6dof_teleop`, or add a command mux with one input per source.

Enable manual ownership and enter READY in JOINT mode:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: JOINT}"
```

Send a joint1 velocity at 20 Hz:

```bash
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.2, 0, 0, 0, 0, 0]}"
```

Select Cartesian mode and jog +X:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: CARTESIAN}"
ros2 topic pub --rate 20 /om6dof/control_cmd \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0, 0, 0, 0, 0]}"
```

Return the arm interfaces to the trajectory controller used by MoveIt:

```bash
ros2 topic pub --once /om6dof/operation_mode \
  std_msgs/msg/String "{data: AUTONOMOUS}"
```

Observe confirmed state:

```bash
ros2 topic echo --once --qos-reliability reliable \
  --qos-durability transient_local /om6dof/operation_mode/state
ros2 topic echo --once --qos-reliability reliable \
  --qos-durability transient_local /om6dof/remote_enabled/state
```

## Go2W input

The primary input is the continuous 40-byte `/lowstate.wireless_remote`
payload. `/wirelesscontroller` is retained as an event fallback and is also
used by the web dashboard. Cross-source edge deduplication prevents F1, F3,
Select, and gripper commands from firing twice.

`Select` cycles `JOINT -> CARTESIAN -> CYLINDRICAL -> JOINT`.

| Input | JOINT | CARTESIAN | CYLINDRICAL |
|---|---|---|---|
| Left / Right | joint1 -/+ | world Y +/- | theta CCW/CW |
| Down / Up | joint2 -/+ | world Z -/+ | Z -/+ |
| A / Y | joint3 -/+ | world X -/+ | radius inward/outward |
| X / B | joint4 -/+ | tool roll +/- | tool roll +/- |
| Right stick `ry` | joint5 analog | tool pitch | tool pitch |
| R1 / R2 | joint6 -/+ | tool yaw +/- | tool yaw +/- |
| L1 / L2 | gripper open/close | gripper open/close | gripper open/close |
| F3 | request `JOINT` / `AUTONOMOUS` | same | same |
| F1 | request `READY` / `STARTUP` | returns to JOINT first | returns to JOINT first |

F3 ON asks `om6dof_controller` to atomically deactivate `arm_controller`,
activate `forward_position_controller`, and ramp from measured feedback to
READY. F3 OFF restores `arm_controller`. The adapter waits for confirmed state;
it does not assume that a controller switch succeeded.

The first complete arm feedback captured by `om6dof_controller` is the STARTUP
pose. After entry to READY, F1 alternates STARTUP and READY. Joystick motion is
latched until a neutral sample arrives after enabling remote control or changing
mode, preventing a held button from unexpectedly moving a different axis.

## Gripper

L1/L2 and the optional string command topic use the ros2_control gripper action:

```bash
ros2 topic pub --once /om6dof_teleop/gripper_cmd \
  std_msgs/msg/String "{data: open}"
ros2 topic echo /om6dof_teleop/gripper_state
```

`gripper_state` combines measured position and the exported effort/current
state interface for `OPEN`, `CLOSED`, or grasp/hold detection. It is not part
of `/om6dof/control_cmd`.

## MoveIt and ownership

MoveIt continues to execute trajectories through `arm_controller`:

```bash
ros2 launch om6dof_moveit_config om6dof_moveit.launch.py
```

Before executing a plan, request `AUTONOMOUS` and verify that
`arm_controller` is active. Enabling remote ownership interrupts the trajectory
path; after returning to autonomous ownership, plan a fresh trajectory.

## Fake-hardware verification

```bash
ros2 launch om6dof_teleop sim.launch.py
ros2 control list_controllers
```

Expected autonomous state:

```text
joint_state_broadcaster     active
arm_controller              active
gripper_controller          active
forward_position_controller inactive
```

After F3 or a `JOINT` request, `arm_controller` and
`forward_position_controller` swap states. There must be exactly one publisher
on the final position topic:

```bash
ros2 topic info --verbose /forward_position_controller/commands
```

That publisher must be `om6dof_controller`, never `om6dof_teleop`.

## systemd

The installed `om6dof-hardware.service` runs the complete stack and sources the
Unitree CycloneDDS environment before the ROS workspace. After building, copy
the unit from the package share directory and enable it in the normal way. Do
not restart it while the physical arm is in an unsafe pose: startup can claim
the position interfaces and move to READY according to the launch setting.

The separate `om6dof-web-monitor.service` from the
`application_web_monitor` package stays alive while the arm stack is restarted.
Its dashboard at `http://<go2w-ip>:8080` provides **Restart OM6DOF stack**. The
request is asynchronous: the page waits up to 45 seconds for a new
systemd MainPID, the four OM6DOF runtime nodes, and healthy ros2_control
states. Healthy means `joint_state_broadcaster` and `gripper_controller` are
active and exactly one of `arm_controller` or
`forward_position_controller` owns the arm interfaces.

The monitor runs as the unprivileged `unitree` user. Install the packaged,
updated monitor unit and the single-command sudoers rule once after building:

```bash
sudo install -o root -g root -m 0644 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/systemd/om6dof-web-monitor.service \
  /etc/systemd/system/om6dof-web-monitor.service
sudo install -o root -g root -m 0440 \
  ~/ros2_ws/install/application_web_monitor/share/application_web_monitor/sudoers/om6dof-web-monitor \
  /etc/sudoers.d/om6dof-web-monitor
sudo visudo -cf /etc/sudoers.d/om6dof-web-monitor
sudo systemctl daemon-reload
sudo systemctl enable om6dof-web-monitor.service
sudo systemctl restart om6dof-web-monitor.service
```

The rule permits only this exact command:

```text
/usr/bin/systemctl --no-block restart om6dof-hardware.service
```

It does not grant a shell or general `systemctl` access. A controller process
exit also shuts down the containing full-stack launch; the hardware service's
`Restart=always` then rebuilds bringup, controller, and teleop together.

Restarting interrupts arm control and initialization can move the arm. Clear
the workspace first. The monitor binds to `0.0.0.0` and has no login page, so
expose port 8080 only on a trusted robot LAN or protect it with a firewall/VPN.
