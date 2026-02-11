# Unitree-go2w

slide: https://tzf230201.github.io/unitree-go2w-autonomous-carrier/

## remote via noVNC:

### Install NoVNC:
https://github.com/anh0001/ubuntu-novnc-quickstart


### Open browser:
http://192.168.0.90:6080/vnc.html



## Connect to PC

### in case using LAN cable
https://techshare.co.jp/faq/unitree/unitree-go2_pc_lan.html#1-1_Docking_Station
in this case i'm using 192.168.123.163

### in case using Wifi Adapter
https://github.com/tzf230201/unitree-go2w-autonomous-carrier/tree/main/wifi_adapter

### VSCode setup
1. install remote development extention
2. ctrl+shift+p
3. Remote-SSH: Connect to Host…
4. ssh unitree@192.168.123.18 
5. export DISPLAY=192.168.123.164:0.0

## Development

datasheet:
https://support.unitree.com/home/en/Go2-W_developer/about_Go2-W

### teleop
https://techshare.co.jp/faq/unitree/unitree-go_cmd_vel_control.html

or basically:

If you see `No executable found` for `cmd_vel_control`, your ROS 2 environment is being *shadowed* by another overlay (commonly `~/go2w_ws/unitree_ros2/install`) which contains an older `unitree_ros2_example` package without that executable.

Use this safe sourcing order (dependencies first, then this workspace) and prefer `local_setup.bash` (not `setup.bash`) to avoid the generated prefix-chain ordering:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/foxy/setup.bash
source ~/go2w_ws/unitree_ros2/install/local_setup.bash
source ~/go2w_ws/install/local_setup.bash

ros2 run unitree_ros2_example cmd_vel_control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

colcon build --packages-select unitree_ros2_example

### XT16 Lidar

## how to install
https://techshare.co.jp/faq/unitree/go2_xt16_foxy.html

### how to run

```
cd go2w_ws
source install/setup.bash
ros2 run hesai_ros_driver hesai_ros_driver_node
```

# d-lio
https://techshare.co.jp/faq/unitree/xt16-on-go2_d-lio.html


# Acknowledgement:

the go2w_joints_state_publisher is modified by this:
https://github.com/felixokolo/go2_slam_2d_3d/tree/main/src/go2_joints_state_publisher





