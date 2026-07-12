# om6dof_teleop

A ROS 2 Python package that lets you teleoperate a 6-DOF OpenManipulator-style
arm using the **Go2W wireless remote controller**. The arm is driven directly
through the **DYNAMIXEL SDK** (Protocol 2.0) over a U2D2 on `/dev/ttyUSB0` —
no `ros2_control`, no MoveIt, no URDF.

The package is structured around two nodes:

| Node | Purpose |
|---|---|
| `arm_launcher` | Always-on daemon. Sets a partial torque state at boot, then watches `/wirelesscontroller` for the **F3** button to start/stop the teleop process. |
| `teleop_node` | Spawned by `arm_launcher`. Reads the present pose, centers the arm using a Time-based trapezoidal profile, then maps remote inputs to per-joint position commands. The **F1** button parks the arm back to the original pose and releases torque. |

A `systemd` unit auto-starts `arm_launcher` at Jetson boot so the user can
just power the robot on and tap **F3**.

---

## Hardware

7 Dynamixels on a single U2D2 chain, 1 Mbit/s, Protocol 2.0:

| Joint | Model | ID |
|---|---|---|
| joint1 | XM430-W350 | 31 |
| joint2 | XM430-W350 | 32 |
| joint3 | XM430-W350 | 33 |
| joint4 | XM430-W210 | 24 |
| joint5 | XM430-W350 | 35 |
| joint6 | XM430-W210 | 26 |
| gripper | XM430-W350 | 37 |

(ID convention on this rig: W350 motors → 30s, W210 motors → 20s, last digit
mirrors the joint number.)

A motor that does not respond at startup is auto-skipped — the rest of the
arm still works, so you can test with a partial chain.

---

## State machine

```
                    ┌────────────────────────────────────────────┐
                    │ Boot torque (run once when launcher starts)│
                    │   joint1 (31), joint4 (24)  →  torque ON   │
                    │   joint2/3/5/6/gripper       →  torque OFF │
                    └────────────────────────┬───────────────────┘
                                             │
                              tap F3 on the Go2W remote
                                             │
                                             ▼
                    ┌────────────────────────────────────────────┐
                    │ teleop_node spawned                        │
                    │   ping → filter live motors                │
                    │   reboot any HW-error flag                 │
                    │   gentle torque-on (all motors)            │
                    │   save present pose as initial_pose        │
                    │   center to 0 rad with                     │
                    │     Time-based trapezoid (4 s / 2 s accel) │
                    └────────────────────────┬───────────────────┘
                                             │
                              user jogs joints / gripper
                                             │
                              tap F1 on the Go2W remote
                                             │
                                             ▼
                    ┌────────────────────────────────────────────┐
                    │ park-and-release                           │
                    │   Time-based trapezoid → initial_pose      │
                    │   joint1 (31), joint4 (24) keep torque ON  │
                    │   joint2/3/5/6/gripper → torque OFF        │
                    │   teleop_node exits                        │
                    └────────────────────────┬───────────────────┘
                                             │
                              tap F3 → fresh cycle
```

At every transition only **one** teleop process runs. The launcher uses a
threading lock plus a 0.5 s F3 debounce, and the teleop owns a `_parking`
flag, so rapid F3/F1 mashing cannot spawn duplicate processes or
double-trigger the park sequence.

---

## Remote button mapping (Go2W wireless controller)

Bit layout (see `go2w_remote_monitor`):

```
0=R1  1=L1  2=Start  3=Select  4=R2  5=L2  6=F1  7=F3
8=A   9=B   10=X     11=Y      12=Up 13=Right 14=Down 15=Left
```

| Button | Owner | Effect |
|---|---|---|
| **F3** | launcher | Toggle: start the teleop launch if nothing is running, otherwise stop it. |
| **F1** | teleop | Park to `initial_pose`, then release torque on every motor outside `release_torque_except_ids`, then exit. |
| Left  / Right     | teleop | joint1 −/+ (held = move at `joint_velocity` rad/s) |
| Down  / Up        | teleop | joint2 −/+ |
| A     / Y         | teleop | joint3 −/+ |
| X     / B         | teleop | joint4 −/+ |
| Right stick ry    | teleop | joint5 analog (deadzone-corrected, value × `joint_velocity` per tick) |
| R1    / R2        | teleop | joint6 −/+ |
| L1                | teleop | gripper → `gripper_open_target` (edge-triggered) |
| L2                | teleop | gripper → `gripper_close_target` (edge-triggered) |

Mappings are fully configurable via ROS parameters — see
[`launch/teleop.launch.py`](launch/teleop.launch.py).

---

## Motion profile

Both the **startup centering** and the **F1 park-and-release** use the
XM430 Time-based Profile:

```
Drive Mode bit 2 = 1                 → Profile registers interpreted in ms
Profile Velocity     = total time    (default 4000 ms)
Profile Acceleration = accel time    (default 2000 ms)
Deceleration time = total − accel    (automatic, default 2000 ms)
```

When `accel == total / 2` this produces a perfect triangular profile (no
cruise phase). Adjustable per-launch via `center_total_time_s` and
`center_accel_time_s`.

Normal teleop jogging uses Velocity-based Profile (the default after
centering), with low Profile Velocity / Acceleration values so live position
updates from `_tick` look smooth.

---

## Install / build

```bash
cd ~/ros2_ws
colcon build --packages-select om6dof_teleop --symlink-install
source install/setup.bash
```

Dependencies (declared in `package.xml`):
- `rclpy`, `unitree_go` (for `/wirelesscontroller`)
- `dynamixel_sdk` (from `src/DynamixelSDK`)
- `sensor_msgs` (for the optional `/joint_states` publisher)

---

## Manual usage (no systemd)

Two terminals:

```bash
# Terminal 1 — the F3 watchdog. Also runs the boot torque step.
ros2 run om6dof_teleop arm_launcher
```

Then tap **F3** on the Go2W remote — the second terminal will see
`arm_launcher` spawn `ros2 launch om6dof_teleop teleop.launch.py`.

To launch the teleop **directly** (skipping F3 detection), e.g. for
development:

```bash
ros2 launch om6dof_teleop teleop.launch.py
# overrides:
ros2 launch om6dof_teleop teleop.launch.py joint_velocity:=0.3
```

---

## Autorun at boot (systemd)

A unit file is installed by colcon under
`install/om6dof_teleop/share/om6dof_teleop/systemd/`. To enable:

```bash
sudo cp /home/unitree/ros2_ws/install/om6dof_teleop/share/om6dof_teleop/systemd/go2w-arm-launcher.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable go2w-arm-launcher.service
sudo systemctl start  go2w-arm-launcher.service
```

Check it:

```bash
systemctl status go2w-arm-launcher.service
journalctl -u go2w-arm-launcher.service -f
```

The unit runs as user `unitree`, group `dialout` (for `/dev/ttyUSB0`), and
sources `/opt/ros/humble/setup.bash` + `~/ros2_ws/install/setup.bash` via a
login shell so the same CycloneDDS env the user has interactively is picked
up. On systemd restart the boot-torque step re-runs.

---

## Parameters

### `arm_launcher`

| Name | Default | Meaning |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Serial device for the U2D2 |
| `baudrate` | `1000000` | DYNAMIXEL baud |
| `boot_torque_on_ids` | `[31, 24]` | IDs to torque **ON** at launcher startup |
| `boot_torque_off_ids` | `[32, 33, 35, 26, 37]` | IDs to torque **OFF** at launcher startup |
| `debounce_seconds` | `0.5` | Minimum gap between F3 presses |
| `stop_timeout_seconds` | `5.0` | How long to wait for teleop to die after SIGINT |

### `teleop_node` (set via [`launch/teleop.launch.py`](launch/teleop.launch.py))

| Name | Default | Meaning |
|---|---|---|
| `device`, `baudrate` | `/dev/ttyUSB0`, `1000000` | Serial |
| `joint_names` | `[joint1..joint6, gripper]` | Logical names (last entry must be the gripper) |
| `motor_ids` | `[31, 32, 33, 24, 35, 26, 37]` | DXL IDs in matching order |
| `joint_velocity` | `0.5` (rad/s) | Per-tick increment magnitude (held button) |
| `joint_signs` | `[1.0, …]` | Per-joint sign flip |
| `joint_axes` | `[-1, -1, -1, -1, 3, -1]` | `-1` = button-pair mode, else stick axis idx (0=lx, 1=ly, 2=rx, 3=ry) |
| `button_pairs` | see launch | `(dec_bit, inc_bit)` per joint, flat-packed |
| `gripper_btn_open` / `gripper_btn_close` | `1` / `5` | Bit indices (L1 / L2) |
| `gripper_open_target` / `gripper_close_target` | `-1.0` / `0.0` | Absolute target (rad) for the gripper joint |
| `center_on_startup` | `true` | Run the 4-second centering on startup |
| `center_total_time_s` / `center_accel_time_s` | `4.0` / `2.0` | Trapezoid parameters |
| `center_position_rad` | `[0.0]` | Center target (broadcast or per-joint) |
| `release_torque_except_ids` | `[31, 24]` | IDs that **keep** torque ON after F1 park |
| `startup_slow_vel` / `startup_slow_acc` | `30` / `10` | Gentle torque-on profile |
| `run_vel` / `run_acc` | `100` / `30` | Normal teleop velocity/accel profile |

---

## Topics

| Topic | Direction | Type | Notes |
|---|---|---|---|
| `/wirelesscontroller` | Sub | `unitree_go/msg/WirelessController` | BEST_EFFORT QoS. Both nodes subscribe. |
| `/om6dof_teleop/joint_command` | Sub | `sensor_msgs/msg/JointState` | Optional absolute joint target command used by `demo_june_2026`. |
| `/joint_states` | Pub | `sensor_msgs/msg/JointState` | Published by `teleop_node` at the loop rate (default 50 Hz). |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ping ID xx failed: rc=-3001` on every motor | Bus is silent. Check 12 V supply, daisy-chain cable, U2D2 USB. Verify with `python3 -c "from dynamixel_sdk import PortHandler, PacketHandler; p=PortHandler('/dev/ttyUSB0'); k=PacketHandler(2.0); p.openPort(); p.setBaudRate(1000000); print(k.broadcastPing(p))"`. |
| Sporadic `-3001` only at high sync-read rate | FTDI latency timer is 16 ms by default. Set 1 ms via the udev rule (`SUBSYSTEM=="usb-serial", DRIVERS=="ftdi_sio", ATTR{latency_timer}="1"`). |
| Motor LED blinking red | Hardware Error flag set (electrical-shock / overheat). The launcher and teleop both auto-reboot flagged motors at startup, so just restart the service. |
| F3 has no effect right after F1 | Stale-window race fixed by an `OnProcessExit` handler in the launch file + a proactive `poll()` in the launcher. If it still happens, check `journalctl` for "previous teleop already exited (rc=…)" — the next F3 should spawn a fresh teleop. |
| Joint moves the wrong way | Flip the corresponding entry in `joint_signs`. |
| Gripper open/close goes to the wrong end | Swap `gripper_open_target` / `gripper_close_target`, or re-calibrate after physical mechanism change. |

---

## Why not `ros2_control` / MoveIt?

The arm IDs on this rig (`[31, 32, 33, 24, 35, 26, 37]`) and the joint count
(6 + gripper) do not match the stock ROBOTIS `open_manipulator_x_bringup`
(IDs 11–15, 4 DOF), so the official ros2_control + URDF + MoveIt stack would
need a full rewrite of the description / xacro / controller_manager.yaml.

Since the use case here is **direct teleop with a wireless remote**, not
motion planning, talking to the DYNAMIXEL SDK directly is dramatically
simpler and boots in ~2 seconds. If MoveIt / RViz visualization is ever
needed, a separate `open_manipulator_chain_bringup`-style package can be
added without touching this one.
