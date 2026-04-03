# Unitree-go2w

slide: https://tzf230201.github.io/unitree-go2w-autonomous-carrier/


<p align="center">
	<img src="image_1.png" alt="Unitree" width="200" height="300" />
</p>

# How to Use this Repo
1. go to workspace
```
cd ~/ros2_ws/src/
```

2. clone this repo

```
git clone --recurse-submodules https://github.com/tzf230201/unitree-go2w-autonomous-carrier.git
```
<p align="center">
	<img src="image_2.png" alt="Unitree" width="400" />
</p>

3. check out ros2 version of lio sam

```
cd LIO-SAM
git checkout ros2
cd ..
cd FAST_LIO
git checkout ROS2
```

4. install dependencies

```
sudo apt install ros-humble-perception-pcl \
       ros-humble-pcl-msgs \
       ros-humble-vision-opencv \
       ros-humble-xacro \
       ros-humble-pcl-conversions
sudo apt-get install libboost-all-dev libyaml-cpp-dev
```

5. install Livox SDK2 (required to build `livox_ros_driver2` and `fast_lio`)

```bash
cd /tmp
git clone https://github.com/Livox-SDK/Livox-SDK2
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

6. build packages (must be built in order)

```bash
cd ~/ros2_ws

# Build livox_ros_driver2 first (requires special cmake args for Humble)
colcon build --packages-select livox_ros_driver2 \
  --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

# Build hesai_ros_driver
colcon build --packages-select hesai_ros_driver

# Source, then build fast_lio
source install/setup.bash
colcon build --packages-select fast_lio

# Source, then build remaining packages
source install/setup.bash
colcon build
```


# Acknowledgement:

the go2w_joints_state_and_imu_publisher is modified from this:
https://github.com/felixokolo/go2_slam_2d_3d.git
especiallly this part:
https://github.com/felixokolo/go2_slam_2d_3d/tree/main/src/go2_joints_state_publisher

the go2w_cmd_vel_control is modified from this:
https://github.com/TechShare-inc/go2_unitree_ros2.git

the pointcloud_to_laserscan from : 
https://github.com/felixokolo/pointcloud_to_laserscan/tree/97c195bbc84f410263178a02ee1117b661a45015
