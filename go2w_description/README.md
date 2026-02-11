# Go2W Description

ROS 2 description package for Unitree Go2W.

## RViz (display_xacro)

If RViz is not clickable or freezes, run RViz separately:

```bash
ros2 launch go2w_description display_xacro.launch.py rviz:=false
rviz2 -d $(ros2 pkg prefix go2w_description)/share/go2w_description/launch/check_joint.rviz
```

If you still want to launch RViz from `ros2 launch`, try forcing X11 and software OpenGL:

```bash
ros2 launch go2w_description display_xacro.launch.py rviz_use_x11:=true rviz_software_gl:=true
```
