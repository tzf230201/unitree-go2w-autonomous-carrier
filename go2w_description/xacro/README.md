# Go2W Description with Xacro

This directory contains the xacro-based robot description for Unitree Go2W.

## Structure

- `robot.xacro` - Main robot model file
- `sensors.xacro` - Sensor definitions (XT16 Lidar)
- `const.xacro` - Constants and properties

## Usage

### With ROS2 Launch

```bash
# Launch with lidar
ros2 launch go2w_description display_xacro.launch.py

# Launch without lidar
ros2 launch go2w_description display_xacro.launch.py add_lidar:=false
```

### Convert to URDF

```bash
# Generate URDF with lidar
xacro $(ros2 pkg prefix go2w_description)/share/go2w_description/xacro/robot.xacro add_lidar:=true > go2w_with_lidar.urdf

# Generate URDF without lidar
xacro $(ros2 pkg prefix go2w_description)/share/go2w_description/xacro/robot.xacro add_lidar:=false > go2w_base.urdf
```

### In Python Launch Files

```python
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([FindPackageShare("go2w_description"), "xacro", "robot.xacro"]),
    " add_lidar:=true"
])
```

## Lidar Configuration

The XT16 Lidar (utlidar_lidar) is mounted at:
- Position: (0.28, 0.0, 0.08) relative to base_link
- Frame ID: `utlidar_lidar`
- Point cloud topic: `/utlidar/cloud`

Adjust the mounting position in `const.xacro` if needed:
```xml
<xacro:property name="lidar_offset_x" value="0.28" />
<xacro:property name="lidar_offset_y" value="0.0" />
<xacro:property name="lidar_offset_z" value="0.08" />
```

## SLAM Usage

With the lidar properly configured in the xacro:

```bash
# Terminal 1: Launch robot with lidar
ros2 launch go2w_description display_xacro.launch.py

# Terminal 2: Launch lidar driver
ros2 run hesai_ros_driver hesai_ros_driver_node

# Terminal 3: Run SLAM (e.g., SLAM Toolbox)
ros2 launch slam_toolbox online_async_launch.py
```
