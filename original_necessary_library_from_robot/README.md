# Original Necessary Library from Robot

This folder contains essential libraries and configurations from the Unitree Go2W robot.

## Files Included

- `unitree_ros2.zip` - Unitree ROS2 SDK and packages
- `cyclonedds_ws.zip` - CycloneDDS workspace with configuration

## Installation Instructions

### 1. Copy Files to Workspace Root

```bash
cd ~/go2w_ws
cp src/unitree-go2w-autonomous-carrier/original_necessary_library_from_robot/*.zip .
```

### 2. Extract Archives

```bash
unzip unitree_ros2.zip
unzip cyclonedds_ws.zip
```

### 3. Set Proper Permissions

```bash
find unitree_ros2 -type d -exec chmod 775 {} \;
find unitree_ros2 -type f -exec chmod 664 {} \;
find cyclonedds_ws -type d -exec chmod 775 {} \;
find cyclonedds_ws -type f -exec chmod 664 {} \;
```

### 4. Configure Environment in ~/.bashrc

Add the following to your `~/.bashrc`:

```bash
# ROS2 Foxy Environment
source /opt/ros/foxy/setup.bash
source ~/go2w_ws/install/setup.bash

# CycloneDDS Configuration
export CYCLONEDDS_URI=file://${HOME}/go2w_ws/cyclonedds_ws/cyclonedds.xml

# X11 Forwarding for GUI (when using SSH from Windows)
if [ -n "$SSH_CLIENT" ]; then
    export DISPLAY=$(echo $SSH_CLIENT | awk '{print $1}'):0.0
    echo "Display forwarded to: $DISPLAY"
fi
```

### 5. Apply Changes

```bash
source ~/.bashrc
```

## Troubleshooting

### GUI Applications Not Showing (SSH from Windows)

If you're connecting via SSH from Windows and GUI applications (RViz, joint_state_publisher_gui) don't appear:

1. **Install X Server on Windows:**
   - Download and install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
   - Or use Xming or X410

2. **Configure VcXsrv:**
   - Select "Multiple windows"
   - Display number: 0
   - Start no client
   - **Important:** Check "Disable access control"

3. **Allow Windows Firewall:**
   - Make sure VcXsrv is allowed through Windows Firewall

4. **Reconnect SSH:**
   - Close and reopen your SSH connection
   - The DISPLAY variable will be set automatically

5. **Manual Override (if needed):**
   ```bash
   export DISPLAY=<YOUR_WINDOWS_IP>:0.0
   ```

### CycloneDDS Configuration Error

If you see:
```
can't open configuration file /home/unitree/go2w_ws/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
```

**Solution:**
- Check that `cyclonedds.xml` exists at: `~/go2w_ws/cyclonedds_ws/cyclonedds.xml`
- Verify `CYCLONEDDS_URI` environment variable is set correctly:
  ```bash
  echo $CYCLONEDDS_URI
  ```

### Test GUI Connection

```bash
# Simple test with xeyes
xeyes

# Or xclock
xclock

# If these work, ROS2 GUI applications should work too
ros2 launch go2w_description check_joint.launch.py
```

## Notes

- Make sure to run VcXsrv **before** connecting SSH if you need GUI
- The DISPLAY variable is automatically configured for SSH sessions
- For local graphical sessions, use `export DISPLAY=:0` or `export DISPLAY=:1`