import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _ensure_child(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def _apply_joint_stance_spring(
    sdf_root: ET.Element,
    joint_name: str,
    reference: float,
    stiffness: float,
    damping: float,
    friction: float,
) -> None:
    joint_elem = sdf_root.find(f".//joint[@name='{joint_name}']")
    if joint_elem is None:
        return
    axis_elem = joint_elem.find("axis")
    if axis_elem is None:
        return

    dynamics_elem = _ensure_child(axis_elem, "dynamics")
    _ensure_child(dynamics_elem, "spring_reference").text = str(reference)
    _ensure_child(dynamics_elem, "spring_stiffness").text = str(stiffness)
    _ensure_child(dynamics_elem, "damping").text = str(damping)
    _ensure_child(dynamics_elem, "friction").text = str(friction)

    init_pos_elem = joint_elem.find("initial_position")
    if init_pos_elem is None:
        init_pos_elem = ET.SubElement(joint_elem, "initial_position")
    init_pos_elem.text = str(reference)


def _apply_leg_stance_springs(
    sdf_root: ET.Element,
    stance_hip: float,
    stance_thigh: float,
    stance_calf: float,
    stiffness: float,
    damping: float,
    friction: float,
) -> None:
    leg_joints = [
        ("FL_hip_joint", stance_hip),
        ("FL_thigh_joint", stance_thigh),
        ("FL_calf_joint", stance_calf),
        ("FR_hip_joint", stance_hip),
        ("FR_thigh_joint", stance_thigh),
        ("FR_calf_joint", stance_calf),
        ("RL_hip_joint", stance_hip),
        ("RL_thigh_joint", stance_thigh),
        ("RL_calf_joint", stance_calf),
        ("RR_hip_joint", stance_hip),
        ("RR_thigh_joint", stance_thigh),
        ("RR_calf_joint", stance_calf),
    ]

    for joint_name, ref in leg_joints:
        _apply_joint_stance_spring(
            sdf_root=sdf_root,
            joint_name=joint_name,
            reference=ref,
            stiffness=stiffness,
            damping=damping,
            friction=friction,
        )


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    run = _as_bool(LaunchConfiguration("run").perform(context))

    model_name = LaunchConfiguration("model_name").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)

    wheel_radius = LaunchConfiguration("wheel_radius").perform(context)
    wheel_separation = LaunchConfiguration("wheel_separation").perform(context)

    use_bridge = _as_bool(LaunchConfiguration("bridge").perform(context))
    use_teleop = _as_bool(LaunchConfiguration("teleop").perform(context))

    ros_cmd_vel_topic = LaunchConfiguration("ros_cmd_vel_topic").perform(context)
    ros_odom_topic = LaunchConfiguration("ros_odom_topic").perform(context)
    gz_cmd_vel_topic = LaunchConfiguration("gz_cmd_vel_topic").perform(context)

    # Generate URDF from xacro with standing configuration
    xacro_cmd = [
        "xacro",
        os.path.join(get_package_share_directory("go2w_description"), "xacro", "robot_standing.xacro"),
        "add_lidar:=true"
    ]
    
    # Get the URDF text
    result = subprocess.run(xacro_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    urdf_text = result.stdout
    
    # Ensure Gazebo can resolve: model://go2w_description/...
    go2w_description_dir = get_package_share_directory("go2w_description")
    models_parent_dir = os.path.dirname(go2w_description_dir)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined_resource_path = ":".join([p for p in [existing_resource_path, models_parent_dir] if p])

    # Convert URDF -> SDF
    tmp_dir = os.path.join(tempfile.gettempdir(), "go2w_sim")
    os.makedirs(tmp_dir, exist_ok=True)
    urdf_path = os.path.join(tmp_dir, f"{model_name}_standing.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf_text)
    
    conversion = subprocess.run(
        ["gz", "sdf", "-p", urdf_path],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    root = ET.fromstring(conversion.stdout)
    model_elem = root.find("model")
    if model_elem is None:
        raise RuntimeError("Failed to find <model> element in generated SDF")

    # Override model name so topics become /model/<name>/...
    model_elem.set("name", model_name)

    # Make legs hold a standing pose using passive joint springs.
    stance_hip = float(LaunchConfiguration("stance_hip").perform(context))
    stance_thigh = float(LaunchConfiguration("stance_thigh").perform(context))
    stance_calf = float(LaunchConfiguration("stance_calf").perform(context))
    leg_spring_stiffness = float(LaunchConfiguration("leg_spring_stiffness").perform(context))
    leg_spring_damping = float(LaunchConfiguration("leg_spring_damping").perform(context))
    leg_joint_friction = float(LaunchConfiguration("leg_joint_friction").perform(context))
    _apply_leg_stance_springs(
        sdf_root=root,
        stance_hip=stance_hip,
        stance_thigh=stance_thigh,
        stance_calf=stance_calf,
        stiffness=leg_spring_stiffness,
        damping=leg_spring_damping,
        friction=leg_joint_friction,
    )

    # Inject a diff drive plugin which listens to cmd_vel and drives the 4 wheel joints.
    plugin = ET.Element(
        "plugin",
        {
            "filename": "gz-sim-diff-drive-system",
            "name": "gz::sim::systems::DiffDrive",
        },
    )

    for left_joint in ("FL_foot_joint", "RL_foot_joint"):
        ET.SubElement(plugin, "left_joint").text = left_joint

    for right_joint in ("FR_foot_joint", "RR_foot_joint"):
        ET.SubElement(plugin, "right_joint").text = right_joint

    ET.SubElement(plugin, "wheel_radius").text = wheel_radius
    ET.SubElement(plugin, "wheel_separation").text = wheel_separation
    ET.SubElement(plugin, "topic").text = gz_cmd_vel_topic

    odom_topic = f"/model/{model_name}/odometry"
    ET.SubElement(plugin, "odom_topic").text = odom_topic
    ET.SubElement(plugin, "odom_publish_frequency").text = "50"

    model_elem.append(plugin)

    sdf_path = os.path.join(tmp_dir, f"{model_name}_standing.sdf")
    ET.ElementTree(root).write(sdf_path, encoding="unicode", xml_declaration=False)

    # Figure out which Gazebo transport topic should be bridged.
    if gz_cmd_vel_topic.startswith("/"):
        gz_cmd_vel_topic_full = gz_cmd_vel_topic
    else:
        gz_cmd_vel_topic_full = f"/model/{model_name}/{gz_cmd_vel_topic}"

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined_resource_path),
    ]

    # Start Gazebo
    if headless:
        gz_cmd = ["gz", "sim", "-s", world]
        if run:
            gz_cmd.insert(3, "-r")
    else:
        gz_cmd = ["gz", "sim", world]
        if run:
            gz_cmd.insert(2, "-r")

    actions.append(ExecuteProcess(cmd=gz_cmd, output="screen"))

    # Spawn model after server startup
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            model_name,
            "-file",
            sdf_path,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
        ],
    )
    actions.append(TimerAction(period=3.0, actions=[spawn]))

    # Publish TF from URDF
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": urdf_text}],
        )
    )

    if use_bridge:
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    f"{gz_cmd_vel_topic_full}@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    f"{odom_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                ],
                remappings=[
                    (gz_cmd_vel_topic_full, ros_cmd_vel_topic),
                    (odom_topic, ros_odom_topic),
                ],
            )
        )

    if use_teleop:
        teleop_launch = os.path.join(get_package_share_directory("go2w_sim"), "launch", "teleop.launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch),
                launch_arguments={
                    "cmd_vel": ros_cmd_vel_topic,
                    "use_xterm": "true",
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="/usr/share/gz/gz-sim8/worlds/empty.sdf",
                description="Path to a Gazebo Harmonic world SDF.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without GUI.",
            ),
            DeclareLaunchArgument(
                "run",
                default_value="true",
                description="Start simulation running immediately (adds gz sim -r). Set false to start paused.",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="go2w",
                description="Entity name in Gazebo.",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.28"),
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.09",
                description="Diff-drive wheel radius (m). Tune if needed.",
            ),
            DeclareLaunchArgument(
                "wheel_separation",
                default_value="0.50",
                description="Diff-drive wheel separation (m). Tune if needed.",
            ),
            DeclareLaunchArgument(
                "stance_hip",
                default_value="0.0",
                description="Standing hip joint reference (rad).",
            ),
            DeclareLaunchArgument(
                "stance_thigh",
                default_value="0.8",
                description="Standing thigh joint reference (rad).",
            ),
            DeclareLaunchArgument(
                "stance_calf",
                default_value="-1.5",
                description="Standing calf joint reference (rad).",
            ),
            DeclareLaunchArgument(
                "leg_spring_stiffness",
                default_value="300.0",
                description="Passive leg joint spring stiffness (N*m/rad).",
            ),
            DeclareLaunchArgument(
                "leg_spring_damping",
                default_value="8.0",
                description="Passive leg joint spring damping (N*m*s/rad).",
            ),
            DeclareLaunchArgument(
                "leg_joint_friction",
                default_value="0.2",
                description="Passive leg joint friction (N*m).",
            ),
            DeclareLaunchArgument(
                "bridge",
                default_value="true",
                description="Start ros_gz_bridge for /clock, cmd_vel and odom.",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="false",
                description="Start teleop_twist_keyboard (interactive).",
            ),
            DeclareLaunchArgument(
                "ros_cmd_vel_topic",
                default_value="/cmd_vel",
                description="ROS 2 cmd_vel topic.",
            ),
            DeclareLaunchArgument(
                "ros_odom_topic",
                default_value="/odom",
                description="ROS 2 odom topic.",
            ),
            DeclareLaunchArgument(
                "gz_cmd_vel_topic",
                default_value="cmd_vel",
                description="DiffDrive plugin cmd_vel topic (relative or absolute).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
