import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


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

    go2w_description_dir = LaunchConfiguration("go2w_description_dir").perform(context)
    urdf_path = os.path.join(go2w_description_dir, "urdf", "go2w_description.urdf")

    if not os.path.exists(urdf_path):
        raise RuntimeError(f"URDF not found: {urdf_path}")

    # Ensure Gazebo can resolve: model://go2w_description/...
    # For that, add the parent directory which contains the go2w_description folder.
    models_parent_dir = os.path.dirname(go2w_description_dir)
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    combined_resource_path = ":".join([p for p in [existing_resource_path, models_parent_dir] if p])

    # Convert URDF -> SDF
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

    tmp_dir = os.path.join(tempfile.gettempdir(), "go2w_sim")
    os.makedirs(tmp_dir, exist_ok=True)
    sdf_path = os.path.join(tmp_dir, f"{model_name}.sdf")
    ET.ElementTree(root).write(sdf_path, encoding="unicode", xml_declaration=False)

    # Figure out which Gazebo transport topic should be bridged.
    # If you keep gz_cmd_vel_topic as 'cmd_vel', the plugin will typically subscribe on:
    #   /model/<model_name>/cmd_vel
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

    # Publish TF from URDF (optional but helpful)
    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read()

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
        actions.append(
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                emulate_tty=True,
                remappings=[("/cmd_vel", ros_cmd_vel_topic), ("cmd_vel", ros_cmd_vel_topic)],
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
            DeclareLaunchArgument("z", default_value="0.50"),
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
            DeclareLaunchArgument(
                "go2w_description_dir",
                default_value="/home/teuku/ros2_ws/src/unitree_ros/robots/go2w_description",
                description="Path to unitree_ros go2w_description directory containing urdf/ and dae/.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
