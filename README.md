# Unitree-go2w

slide: https://tzf230201.github.io/unitree-go2w-autonomous-carrier/


![alt text](image.png)


## remote via NoMachine (recommended):

NoMachine server is already installed on this robot (`nxserver`) and listens on port `4000`.

### Connect from your laptop:
1. Install NoMachine client.
2. Create a new connection to one of robot IPs:
   - `192.168.123.18` (LAN dock)
   - `192.168.0.90` (Wi-Fi adapter)
3. Port: `4000`
4. Login with robot username/password (`unitree` user).

### Check server on robot:
```bash
sudo systemctl status nxserver
ss -tulpn | grep :4000
```

## legacy remote via noVNC (optional):
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
5. (optional) for `ssh -X` / `ssh -Y`, do not hardcode DISPLAY in `~/.bashrc`

## Development

datasheet:
https://support.unitree.com/home/en/Go2-W_developer/about_Go2-W

### teleop
https://techshare.co.jp/faq/unitree/unitree-go_cmd_vel_control.html

or basically:

If you see `No executable found` for `cmd_vel_control`, your ROS 2 environment is being *shadowed* by another overlay (commonly `~/go2w_ws/unitree_ros2/install`) which contains an older `unitree_ros2_example` package without that executable.

Use this safe sourcing order (dependencies first, then this workspace) and prefer `local_setup.bash` (not `setup.bash`) to avoid the generated prefix-chain ordering:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
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

### troubleshooting: `LidarState` message missing

Most important symptom:

```
AttributeError: module 'unitree_go.msg' has no attribute 'LidarState'
```

Artinya: saat `ros2 topic echo /utlidar/lidar_state` jalan, ROS2 mencoba import tipe pesan bernama `unitree_go/msg/LidarState`, tapi di instalasi kamu message itu tidak ada / tidak ter-generate / tidak tersource.

In this repo, the most common root cause is a broken/older overlay (commonly `~/go2w_ws/unitree_ros2/install`) shadowing the correct `unitree_go` Python message package. You can confirm which one is being imported:

```bash
python3 -c "import unitree_go,unitree_go.msg; print(unitree_go.__file__); print('has LidarState:', hasattr(unitree_go.msg,'LidarState'))"
```

Safe fix for this workspace (clean + source only what you need):

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/foxy/setup.bash
source ~/go2w_ws/install/local_setup.bash

# Foxy doesn't have --once; use timeout if you only want 1-2 seconds of output
timeout 2 ros2 topic echo /utlidar/lidar_state
```

### built-in Unitree SLAM (without building your own stack)

Unitree docs state that `unitree_slam` and test routines can conflict with ROS/ROS2 initialized shells. Use separate terminals:

#### Terminal A: run Unitree built-in SLAM only (no ROS sourcing)

```bash
env -i HOME=$HOME USER=$USER SHELL=/bin/bash TERM=$TERM PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/go2w_ws/cyclonedds_ws/cyclonedds.xml

cd /unitree/module/unitree_slam/bin
./unitree_slam
```

Open another non-ROS terminal for lidar driver (pick one):

```bash
env -i HOME=$HOME USER=$USER SHELL=/bin/bash TERM=$TERM PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash --noprofile --norc
cd /unitree/module/unitree_slam/bin
./mid360_driver
# or
./xt16_driver
```

Optional test client (also non-ROS shell):

```bash
cd /unitree/module/unitree_slam/bin
./keyDemo eth0
```

#### Terminal B: ROS2 workspace (monitoring/tools only)

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/foxy/setup.bash
source ~/go2w_ws/install/local_setup.bash
```

Recommended: do not start heavy ROS2 nodes while built-in SLAM is running on dock PC.

#### quick rule

- Built-in SLAM binaries (`unitree_slam`, `mid360_driver`/`xt16_driver`, `keyDemo`): run in clean non-ROS shell.
- ROS2 tools/nodes: run in separate ROS2 shell.
- Do not source ROS and run built-in SLAM binaries in the same terminal session.

#### helper scripts (recommended)

From `~/go2w_ws/src/unitree-go2w-autonomous-carrier`:

```bash
chmod +x scripts/*.sh
```

Terminal A:

```bash
./scripts/run_unitree_slam.sh
```

Terminal B:

```bash
./scripts/run_xt16_driver.sh
```

Optional command terminal:

```bash
./scripts/run_keydemo.sh eth0
```

Optional ROS2 monitor shell:

```bash
./scripts/run_ros2_monitor_shell.sh
```

One command to run built-in SLAM stack (tmux):

```bash
./scripts/run_builtin_slam_stack.sh
# then:
tmux attach -t go2w_slam
```

# d-lio
https://techshare.co.jp/faq/unitree/xt16-on-go2_d-lio.html


# Acknowledgement:

the go2w_joints_state_publisher is modified from this:
https://github.com/felixokolo/go2_slam_2d_3d.git
especiallly this part:
https://github.com/felixokolo/go2_slam_2d_3d/tree/main/src/go2_joints_state_publisher

the go2w_cmd_vel_control is modified from this:
https://github.com/TechShare-inc/go2_unitree_ros2.git

the pointcloud_to_laserscan from : 
https://github.com/felixokolo/pointcloud_to_laserscan/tree/97c195bbc84f410263178a02ee1117b661a45015
