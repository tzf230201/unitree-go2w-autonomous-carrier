# om6dof_bringup

Launch the controller stack without MoveIt:

```bash
ros2 launch om6dof_bringup hardware.launch.py
```

Launch hardware, MoveIt, and RViz:

```bash
ros2 launch om6dof_bringup real.launch.py
```

For a hardware-free integration check:

```bash
ros2 launch om6dof_bringup real.launch.py use_fake_hardware:=true
```

The real hardware mapping is `joint1..joint6, gripper_left_joint` to Dynamixel IDs
`31, 32, 33, 24, 35, 26, 37` at 1 Mbps.
