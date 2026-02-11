
# WSL (Ubuntu 22.04) Setup: NVIDIA + ROS 2 Humble + Gazebo Harmonic

This document describes what was installed and how to verify it on **WSL2 (Ubuntu 22.04)**:

- NVIDIA GPU support in WSL2
- ROS 2 Humble (binary install via `apt`)
- Gazebo **Harmonic** (Gazebo Sim, CLI `gz`) + ROS 2 bridge (`ros_gz`)

## 1) NVIDIA GPU in WSL2 (Compute + OpenGL)

### 1.1 Verify NVIDIA compute works

In WSL, the Linux kernel driver is provided by Windows, so it is normal to NOT see Linux `/dev/nvidia*` nodes or kernel modules.

Run:

```bash
nvidia-smi
```

Expected:

- GPU is listed
- Driver version and CUDA version are shown

### 1.2 Verify OpenGL uses the NVIDIA GPU (WSLg)

WSLg uses Mesa on Linux and typically renders via a **D3D12** backend. On hybrid laptops, it may default to the Intel iGPU unless forced.

Install OpenGL info tools:

```bash
sudo apt-get update
sudo apt-get install -y mesa-utils
```

Check the renderer:

```bash
glxinfo -B | grep -E 'OpenGL renderer|Device:'
```

If it shows Intel, force Mesa D3D12 to prefer NVIDIA:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
glxinfo -B | grep -E 'OpenGL renderer|Device:'
```

To make this persistent for new terminals, add this to `~/.bashrc`:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
```

Then open a new terminal or run:

```bash
source ~/.bashrc
```

## 2) Install ROS 2 Humble (Ubuntu 22.04)

### 2.1 System prerequisites

```bash
sudo apt-get update
sudo apt-get install -y locales software-properties-common curl gnupg lsb-release ca-certificates
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```

### 2.2 Add ROS 2 apt repository

```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
	| sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
	| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update
```

### 2.3 Install ROS 2 Humble Desktop

```bash
sudo apt-get install -y \
	ros-humble-desktop \
	ros-dev-tools \
	python3-rosdep \
	python3-colcon-common-extensions \
	python3-vcstool
```

### 2.4 Initialize rosdep

```bash
sudo rosdep init || true
rosdep update
```

### 2.5 Source ROS on every terminal

Add this to `~/.bashrc`:

```bash
if [ -f /opt/ros/humble/setup.bash ]; then
	source /opt/ros/humble/setup.bash
fi
```

Verify:

```bash
source ~/.bashrc
echo $ROS_DISTRO
ros2 --help | head
```

Expected: `ROS_DISTRO=humble`.

## 3) Gazebo: migrate from Classic to Harmonic (Gazebo Sim)

### 3.1 Why `gazebo` vs `gz`

- `gazebo`, `gzserver`, `gzclient` = **Gazebo Classic** (Gazebo 11)
- `gz sim` = **Gazebo Sim** (new generation; Harmonic uses Gazebo Sim 8)

ROS 2 integration differs:

- Classic used `gazebo_ros` packages (`ros-humble-gazebo-ros-pkgs`)
- Harmonic uses `ros_gz` packages (in this setup: `ros-humble-ros-gzharmonic-*`)

### 3.2 Remove Gazebo Classic (if installed)

```bash
sudo apt-get remove --purge -y 'gazebo*' 'libgazebo*' 'ros-humble-gazebo-*'
sudo apt-get autoremove --purge -y
```

### 3.3 Add OSRF repository for Gazebo Harmonic

```bash
sudo apt-get install -y wget gnupg
sudo mkdir -p /etc/apt/keyrings
wget -qO- https://packages.osrfoundation.org/gazebo.gpg \
	| sudo gpg --dearmor -o /etc/apt/keyrings/gazebo-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
	| sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
```

### 3.4 Install Gazebo Harmonic

```bash
sudo apt-get install -y gz-harmonic
```

Verify:

```bash
gz sim --version
```

### 3.5 Install ROS 2 bridge for Gazebo Harmonic

```bash
sudo apt-get install -y \
	ros-humble-ros-gzharmonic \
	ros-humble-ros-gzharmonic-bridge \
	ros-humble-ros-gzharmonic-sim
```

Verify:

```bash
source ~/.bashrc
ros2 pkg list | grep -E '^ros_gz'
```

### 3.6 Smoke test Gazebo Harmonic

Headless:

```bash
gz sim -s -r /usr/share/gz/gz-sim8/worlds/empty.sdf
```

With GUI (WSLg):

```bash
gz sim -v 4 /usr/share/gz/gz-sim8/worlds/empty.sdf

## 4) Connect WSL2 ROS 2 to Unitree robot (CycloneDDS)

WSL2 can ping the robot but DDS discovery often fails unless network and firewall
rules allow inbound UDP and the robot can reach the WSL IP. The steps below are
the minimal setup that worked reliably.

### 4.1 Ensure WSL2 is on the robot subnet

Use Windows 11 mirrored networking so WSL gets an IP on the same subnet as the robot.

Create or edit `%UserProfile%\.wslconfig` in Windows:

```ini
[wsl2]
networkingMode=mirrored
```

Then run (PowerShell):

```powershell
wsl --shutdown
```

After reopening WSL, confirm you get an IP like `192.168.123.x`:

```bash
ifconfig
```

### 4.2 Allow inbound ICMP + DDS UDP on Windows

Even with mirrored networking, Windows Firewall can block inbound traffic to WSL.
Run these in **PowerShell as Administrator**:

```powershell
netsh advfirewall firewall add rule name="WSL ICMPv4 In" dir=in action=allow protocol=icmpv4
netsh advfirewall firewall add rule name="WSL ROS2 UDP In" dir=in action=allow protocol=udp localport=7400-7600
```

Verify the robot can ping the WSL IP.

### 4.3 Configure CycloneDDS peers (two-way)

Set explicit peers on both sides to avoid multicast discovery issues in WSL2.

WSL config (this repo): [cyclonedds_ws/cyclonedds.xml](cyclonedds_ws/cyclonedds.xml)

```xml
<Discovery>
	<Peers>
		<Peer address="192.168.123.18"/>
	</Peers>
</Discovery>
```

Robot config: add the WSL IP as a peer (example `192.168.123.164`).

```xml
<Discovery>
	<Peers>
		<Peer address="192.168.123.164"/>
	</Peers>
</Discovery>
```

If the robot only reads its DDS config at startup, **reboot the robot** after
changing the file.

### 4.4 Run ROS 2 from WSL

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/teuku/go2w_ws/cyclonedds_ws/cyclonedds.xml

source /home/teuku/go2w_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 topic list
```
```



