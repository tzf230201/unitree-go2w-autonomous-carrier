## go2w_nav2 (ROS 2 Foxy)

This package provides a ready-to-use Nav2 bringup for Go2-W.

![alt text](image.png)

### What it assumes

- TF is available: `map -> odom -> base_link`
- Odometry topic exists: `/odom`
- Laser scan topic exists: `/scan`
- Motion bridge exists: `go2w_cmd_vel_control` subscribing to `/cmd_vel`

### Files

- Launch: `launch/nav2_bringup.launch.py`
- Params: `config/nav2_params.yaml`
- Default map: `maps/blank_map.yaml`
- RViz (Nav2): `rviz/nav2.rviz`

### Build

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select go2w_nav2
source /home/unitree/go2w_ws/install/local_setup.bash
```

### Run (all-in-one launch)

This launch starts components in sequence:

1. `go2w_cmd_vel_control`
2. `lio_sam` (`run.launch.py`)
3. `odom_to_tf_ros2`
4. Nav2 bringup

```bash
cd /home/unitree/go2w_ws
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w_ws/unitree_ros2/install/local_setup.bash
source /home/unitree/go2w_ws/install/local_setup.bash
ros2 launch go2w_nav2 nav2_bringup.launch.py
```

If you do not have a map yet, run in SLAM mode:

```bash
ros2 launch go2w_nav2 nav2_bringup.launch.py slam:=true
```

You can disable RViz (for headless runs):

```bash
ros2 launch go2w_nav2 nav2_bringup.launch.py start_rviz:=false
```

### Use your own map

```bash
ros2 launch go2w_nav2 nav2_bringup.launch.py map:=/absolute/path/to/map.yaml
```

Example:

```bash
ros2 launch go2w_nav2 nav2_bringup.launch.py map:=/home/unitree/maps/lab_map.yaml
```

### Notes

- Default Nav2 output is `/cmd_vel`, already compatible with `go2w_cmd_vel_control`.
- Tune robot limits and costmap in `config/nav2_params.yaml` to match your real robot and environment.
- `odom_to_tf_ros2` params file can be overridden with:

```bash
ros2 launch go2w_nav2 nav2_bringup.launch.py odom_to_tf_params_file:=/path/to/odom_to_tf.yaml
```
