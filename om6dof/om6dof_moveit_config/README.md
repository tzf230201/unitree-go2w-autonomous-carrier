# om6dof_moveit_config

Start MoveIt and RViz after the OM6DOF hardware/controller stack is running:

```bash
ros2 launch om6dof_moveit_config om6dof_moveit.launch.py
```

Disable RViz for headless operation:

```bash
ros2 launch om6dof_moveit_config om6dof_moveit.launch.py start_rviz:=false
```
