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


def _inject_ros2_control_tag(urdf_xml: str) -> str:
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise RuntimeError("Invalid URDF: root is not <robot>")

    # Remove existing ros2_control tags if any (avoid duplicates)
    for existing in list(root.findall("ros2_control")):
        root.remove(existing)

    ros2_control = ET.Element("ros2_control", {"name": "GzSimSystem", "type": "system"})
    hardware = ET.SubElement(ros2_control, "hardware")
    ET.SubElement(hardware, "plugin").text = "gz_ros2_control/GazeboSimSystem"

    # Wheels: velocity command interface
    wheel_joints = ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]
    for joint_name in wheel_joints:
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "velocity"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    # Legs: position command interface (for standing / posture control)
    leg_joints = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]
    for joint_name in leg_joints:
        joint = ET.SubElement(ros2_control, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    root.append(ros2_control)
    return ET.tostring(root, encoding="unicode")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    run = _as_bool(LaunchConfiguration("run").perform(context))

    model_name = LaunchConfiguration("model_name").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)

    go2w_description_dir = LaunchConfiguration("go2w_description_dir").perform(context)
    urdf_path = os.path.join(go2w_description_dir, "urdf", "go2w_description.urdf")

    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")

    # Ensure Gazebo can resolve: model://go2w_description/...
    models_parent_dir = os.path.dirname(go2w_description_dir)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined_resource_path = ":".join([p for p in [existing_resource_path, models_parent_dir] if p])

    # Make sure Gazebo can locate a Harmonic-compatible gz_ros2_control system plugin.
    # Prefer a locally-built overlay (built with GZ_VERSION=harmonic) if present, otherwise fall back to /opt/ros.
    overlay_system_plugin_dir = os.path.join(
        os.path.expanduser("~"), "gz_ros2_control_ws", "install", "gz_ros2_control", "lib"
    )
    if not os.path.isdir(overlay_system_plugin_dir):
        overlay_system_plugin_dir = ""

    existing_system_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    combined_system_plugin_path = ":".join(
        [p for p in [existing_system_plugin_path, overlay_system_plugin_dir, "/opt/ros/humble/lib"] if p]
    )
    existing_plugin_path = os.environ.get("GZ_SIM_PLUGIN_PATH", "")
    combined_plugin_path = ":".join(
        [p for p in [existing_plugin_path, overlay_system_plugin_dir, "/opt/ros/humble/lib"] if p]
    )

    # Read URDF and inject ros2_control tag
    with open(urdf_path, "r", encoding="utf-8") as f:
        base_urdf = f.read()
    urdf_text = _inject_ros2_control_tag(base_urdf)

    # Convert URDF -> SDF for Gazebo
    tmp_dir = os.path.join(tempfile.gettempdir(), "go2w_sim")
    os.makedirs(tmp_dir, exist_ok=True)
    patched_urdf_path = os.path.join(tmp_dir, f"{model_name}.ros2_control.urdf")
    with open(patched_urdf_path, "w", encoding="utf-8") as f:
        f.write(urdf_text)

    conversion = subprocess.run(
        ["gz", "sdf", "-p", patched_urdf_path],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    sdf_root = ET.fromstring(conversion.stdout)
    model_elem = sdf_root.find("model")
    if model_elem is None:
        raise RuntimeError("Failed to find <model> element in generated SDF")

    model_elem.set("name", model_name)

    # Add gz_ros2_control system plugin
    plugin = ET.Element(
        "plugin",
        {
            "filename": "libgz_ros2_control-system.so",
            "name": "gz_ros2_control::GazeboSimROS2ControlPlugin",
        },
    )
    ET.SubElement(plugin, "robot_param_node").text = "robot_state_publisher"
    ET.SubElement(plugin, "robot_param").text = "robot_description"

    controllers_yaml = os.path.join(get_package_share_directory("go2w_sim"), "config", "go2w_ros2_control.yaml")
    ET.SubElement(plugin, "parameters").text = controllers_yaml

    model_elem.append(plugin)

    sdf_path = os.path.join(tmp_dir, f"{model_name}.ros2_control.sdf")
    ET.ElementTree(sdf_root).write(sdf_path, encoding="unicode", xml_declaration=False)

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", combined_resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", combined_system_plugin_path),
        SetEnvironmentVariable("GZ_SIM_PLUGIN_PATH", combined_plugin_path),
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

    # Spawn model
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

    # Robot state publisher provides robot_description for gz_ros2_control
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": urdf_text}],
        )
    )

    # Bridge /clock only
    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
        )
    )

    # Spawn controllers after plugin has created controller_manager
    spawner_args = ["--controller-manager", "/controller_manager"]
    actions.append(
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"] + spawner_args,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner", "diff_drive_controller"] + spawner_args,
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner", "leg_position_controller"] + spawner_args,
                    output="screen",
                ),
            ],
        )
    )

    # Publish a default stance to help the robot stand
    stance_hip = LaunchConfiguration("stance_hip").perform(context)
    stance_thigh = LaunchConfiguration("stance_thigh").perform(context)
    stance_calf = LaunchConfiguration("stance_calf").perform(context)

    stance = [
        stance_hip,
        stance_thigh,
        stance_calf,
        stance_hip,
        stance_thigh,
        stance_calf,
        stance_hip,
        stance_thigh,
        stance_calf,
        stance_hip,
        stance_thigh,
        stance_calf,
    ]

    stance_cmd = "{data: [" + ", ".join(stance) + "]}"
    actions.append(
        TimerAction(
            period=9.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "--once",
                        "/leg_position_controller/commands",
                        "std_msgs/msg/Float64MultiArray",
                        stance_cmd,
                    ],
                    output="screen",
                )
            ],
        )
    )

    # Optional teleop
    if _as_bool(LaunchConfiguration("teleop").perform(context)):
        teleop_launch = os.path.join(get_package_share_directory("go2w_sim"), "launch", "teleop.launch.py")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch),
                launch_arguments={
                    "cmd_vel": "/diff_drive_controller/cmd_vel_unstamped",
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
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "run",
                default_value="true",
                description="Start simulation running immediately (adds gz sim -r). Set false to start paused.",
            ),
            DeclareLaunchArgument("teleop", default_value="false"),
            DeclareLaunchArgument("model_name", default_value="go2w"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.50"),
            DeclareLaunchArgument(
                "go2w_description_dir",
                default_value="/home/teuku/ros2_ws/src/unitree_ros/robots/go2w_description",
                description="Path to unitree_ros go2w_description directory containing urdf/ and dae/.",
            ),
            DeclareLaunchArgument(
                "stance_hip",
                default_value="0.0",
                description="Default leg hip joint position (rad).",
            ),
            DeclareLaunchArgument(
                "stance_thigh",
                default_value="0.8",
                description="Default leg thigh joint position (rad).",
            ),
            DeclareLaunchArgument(
                "stance_calf",
                default_value="-1.5",
                description="Default leg calf joint position (rad).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
