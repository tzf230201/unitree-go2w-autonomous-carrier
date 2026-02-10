## go2w_sim (ROS 2 Humble + Gazebo Harmonic)

This package launches **Gazebo Harmonic** (`gz sim`) and spawns the Go2W model from the `go2w_description` URDF (from the `unitree_ros` repository).

### Build (avoid workspace duplicate package errors)

Your workspace currently contains duplicate package names (e.g. `unitree_ros2_example`).
Build only the packages you need:

```bash
cd ~/ros2_ws
colcon build --base-paths src/unitree-go2w-autonomous-carrier/go2w_sim
source install/setup.bash
```

### Run simulation (Gazebo Harmonic)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2w_sim go2w_harmonic.launch.py
```

If you want GUI (WSLg):

```bash
ros2 launch go2w_sim go2w_harmonic.launch.py headless:=false
```

Start paused (recommended while debugging):

```bash
ros2 launch go2w_sim go2w_harmonic.launch.py run:=false
```

### Run simulation (ros2_control + standing posture)

This launch uses `gz_ros2_control` so wheels and leg joints are actuated via ROS 2 controllers.

**Important:** On Humble + Harmonic, you must source the `gz_ros2_control` overlay built against Harmonic.

Copy-paste:

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

# Optional: clean old gz sim servers if you launched many times
pkill -9 -f "gz sim" || true

ros2 launch go2w_sim go2w_harmonic_ros2_control.launch.py teleop:=true
```

Check controllers:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 control list_controllers
```

### Teleop

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2w_sim teleop.launch.py cmd_vel:=/cmd_vel
```

For `ros2_control` diff drive, the controller subscribes to:

```bash
/diff_drive_controller/cmd_vel_unstamped
```

If you run teleop separately (instead of `teleop:=true`):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2w_sim teleop.launch.py cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Notes

- The launch file converts the URDF to SDF at runtime using `gz sdf -p` and injects a Gazebo Sim diff-drive system plugin.
- `unitree_ros/robots/go2w_description` is ROS 1 (catkin), so this package reads the URDF/meshes directly from disk instead of building `go2w_description` with colcon.
- If your `go2w_description` is in a different location, override:

```bash
ros2 launch go2w_sim go2w_harmonic.launch.py go2w_description_dir:=/path/to/unitree_ros/robots/go2w_description
```
- Wheel radius and separation are approximate; tune with:

```bash
ros2 launch go2w_sim go2w_harmonic.launch.py wheel_radius:=0.09 wheel_separation:=0.50
```

Pause / unpause from CLI (Gazebo transport):

```bash
gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'
gz service -s /world/empty/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: false'
```

