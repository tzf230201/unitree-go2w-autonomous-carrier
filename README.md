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

## Development

### teleop
https://techshare.co.jp/faq/unitree/unitree-go_cmd_vel_control.html

or basically:
ros2 run unitree_ros2_example cmd_vel_control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

colcon build --packages-select unitree_ros2_example

### XT16 Lidar
https://techshare.co.jp/faq/unitree/xt16-on-go2_d-lio.html





