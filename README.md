# Unitree-go2w

slide: https://tzf230201.github.io/unitree-go2w-autonomous-carrier/


<p align="center">
	<img src="images/image_1.png" alt="Unitree" width="200" height="300" />
</p>

## Packages

| Package | Description |
|---|---|
| `FAST_LIO_ROS2_HesaiLidar_XT16` | FAST-LIO2 with native Hesai PandarXT-16 support (no Livox dependency required) |
| `HesaiLidar_ROS_2.0` | Default Hesai LiDAR ROS 2 driver |
| `go2w_fast_lio2` | Launch & config for FAST-LIO2 + Hesai driver on Go2W |
| `go2w_description` | Go2W URDF/xacro robot description |
| `go2w_joints_state_and_imu_publisher` | Joint states & IMU publisher for Go2W |
| `go2w_cmd_vel_control` | Velocity control for Go2W |
| `pointcloud_to_laserscan` | Convert 3D point cloud to 2D laser scan |

## How to Use this Repo

### 1. Go to workspace
```bash
cd ~/ros2_ws/src/
```

### 2. Clone this repo
```bash
git clone --recurse-submodules https://github.com/tzf230201/unitree-go2w-autonomous-carrier.git
```
<p align="center">
	<img src="images/image_2.png" alt="Unitree" width="400" />
</p>

### 3. Check out ROS 2 branch for LIO-SAM
```bash
cd LIO-SAM
git checkout ros2
cd ..
```

### 4. Install dependencies
```bash
sudo apt install ros-humble-perception-pcl \
       ros-humble-pcl-msgs \
       ros-humble-vision-opencv \
       ros-humble-xacro \
       ros-humble-pcl-conversions
sudo apt-get install libboost-all-dev libyaml-cpp-dev
```

### 5. Build packages
```bash
cd ~/ros2_ws

# Build hesai_ros_driver first
colcon build --packages-select hesai_ros_driver

# Source, then build fast_lio (native Hesai support, no Livox SDK needed)
source install/setup.bash
colcon build --packages-select fast_lio

# Source, then build remaining packages
source install/setup.bash
colcon build
```

### 6. Run FAST-LIO2 with Hesai PandarXT-16
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2w_fast_lio2 fast_lio2.launch.py
```

This launches:
- Hesai LiDAR driver (default, publishes `/lidar_points`)
- FAST-LIO2 with native Hesai handler (`lidar_type: 5`)
- Go2W robot description & joint state publisher
- RViz2 (optional, disable with `rviz:=false`)

### (Optional) Livox LiDAR support

If you want to use a Livox LiDAR instead, install Livox SDK2 and `livox_ros_driver2`:

```bash
# Install Livox SDK2
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install

# Build livox_ros_driver2
cd ~/ros2_ws
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

# Rebuild fast_lio (will auto-detect livox_ros_driver2 and enable Livox support)
source install/setup.bash
colcon build --packages-select fast_lio
```

## Acknowledgement

- `go2w_joints_state_and_imu_publisher` is modified from:
  https://github.com/felixokolo/go2_slam_2d_3d/tree/main/src/go2_joints_state_publisher

- `go2w_cmd_vel_control` is modified from:
  https://github.com/TechShare-inc/go2_unitree_ros2.git

- `pointcloud_to_laserscan` from:
  https://github.com/felixokolo/pointcloud_to_laserscan/tree/97c195bbc84f410263178a02ee1117b661a45015

- `FAST_LIO_ROS2_HesaiLidar_XT16` is based on [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) with added native Hesai point cloud support and optional Livox dependency
