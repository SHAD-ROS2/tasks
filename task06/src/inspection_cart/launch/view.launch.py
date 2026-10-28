"""Exactly one state source; playback reconstructs TF from recorded JointState."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_nodes(context):
    share = Path(get_package_share_directory("inspection_cart"))
    variant = LaunchConfiguration("model").perform(context)
    state_source = LaunchConfiguration("state_source").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower()
    sim_time = LaunchConfiguration("use_sim_time").perform(context).lower()
    if variant not in {"clean", "broken"}:
        raise ValueError("model must be clean or broken")
    if state_source not in {"gui", "synthetic", "none"}:
        raise ValueError("state_source must be gui, synthetic or none")
    if rviz not in {"true", "false"} or sim_time not in {"true", "false"}:
        raise ValueError("rviz and use_sim_time must be true or false")
    use_sim_time = sim_time == "true"
    if use_sim_time and state_source != "none":
        raise ValueError("Replay uses state_source:=none; the bag owns /joint_states and /clock")
    filename = "cart.urdf" if variant == "clean" else "cart-broken.urdf"
    description = (share / "urdf" / filename).read_text()
    nodes = [Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": description, "use_sim_time": use_sim_time}],
    )]
    if state_source == "gui":
        nodes.append(Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui", output="screen",
            parameters=[{"robot_description": description, "use_sim_time": False}],
        ))
    elif state_source == "synthetic":
        nodes.append(Node(
            package="inspection_cart", executable="synthetic_states",
            name="synthetic_joint_states", output="screen",
            parameters=[{"use_sim_time": False}],
        ))
    if rviz == "true":
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", str(share / "rviz" / "cart.rviz")],
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="clean"),
        DeclareLaunchArgument("state_source", default_value="synthetic"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=start_nodes),
    ])
