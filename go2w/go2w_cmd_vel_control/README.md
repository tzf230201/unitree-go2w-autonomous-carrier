## go2w_cmd_vel_control (ROS 2 Foxy)

This package bridges `geometry_msgs/msg/Twist` from the `cmd_vel` topic into Unitree Sport API commands on `/api/sport/request`.

### Topic

- Input: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Output: `/api/sport/request` (`unitree_api/msg/Request`)

Published API commands:
- `1008` = Move
- `1003` = StopMove (on timeout)
- `1004` = StandUp (once at startup, enabled by default)

### Build

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w_ws/unitree_ros2/install/local_setup.bash
colcon build --packages-select go2w_cmd_vel_control
source /home/unitree/go2w_ws/install/local_setup.bash
```

### Keyboard Control (Teleop)

Terminal 1: run the `cmd_vel -> /api/sport/request` bridge

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w_ws/unitree_ros2/install/local_setup.bash
source /home/unitree/go2w_ws/install/local_setup.bash
ros2 run go2w_cmd_vel_control go2w_cmd_vel_control_node
```

Terminal 2: run keyboard teleop

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w_ws/install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Or run both together with launch:

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w_ws/unitree_ros2/install/local_setup.bash
source /home/unitree/go2w_ws/install/local_setup.bash
ros2 launch go2w_cmd_vel_control go2w_cmd_vel_control.launch.py
```

### How To Drive With Keyboard

In the teleop terminal, make sure the terminal is focused, then use:

- `i`: forward
- `,`: backward
- `j`: rotate left
- `l`: rotate right
- `u` / `o`: forward + turn
- `m` / `.`: backward + turn
- `k`: force stop
- `q/z`: increase/decrease all speeds
- `w/x`: increase/decrease linear speed only
- `e/c`: increase/decrease angular speed only

Important:

- Keep the key pressed or repeatedly press keys to keep sending `cmd_vel`.
- If no new command arrives within `cmd_timeout_sec` (default `0.5`), the node sends `StopMove`.

### Important Parameters

- `cmd_vel_topic` (default: `/cmd_vel`)
- `request_topic` (default: `/api/sport/request`)
- `cmd_timeout_sec` (default: `0.5`)
- `control_rate_hz` (default: `20.0`)
- `max_linear_x` (default: `0.6`)
- `max_linear_y` (default: `0.4`)
- `max_angular_z` (default: `1.0`)
- `auto_stand_up` (default: `true`)

Example: change timeout:

```bash
ros2 run go2w_cmd_vel_control go2w_cmd_vel_control_node --ros-args -p cmd_timeout_sec:=0.8
```

### Quick Check

Check teleop publishing:

```bash
ros2 topic echo /cmd_vel
```

Check requests sent to the robot:

```bash
ros2 topic echo /api/sport/request
```
