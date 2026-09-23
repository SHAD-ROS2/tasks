from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("warehouse_bringup")
    default_params = os.path.join(package_share, "config", "warehouse.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    panel_delay_s = LaunchConfiguration("panel_delay_s")

    common = {
        "package": "warehouse_nodes",
        "output": "screen",
        "parameters": [params_file],
    }

    stack = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(executable="range_sensor", name="range_sensor", **common),
            Node(executable="obstacle_filter", name="obstacle_filter", **common),
            Node(executable="mission_planner", name="mission_planner", **common),
            Node(executable="drive_controller", name="drive_controller", **common),
            Node(executable="safety_monitor", name="safety_monitor", **common),
            TimerAction(
                period=panel_delay_s,
                actions=[
                    Node(executable="operator_panel", name="operator_panel", **common),
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="robot_1",
                description="Relative namespace for this robot instance.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Absolute path to the stack parameter file.",
            ),
            DeclareLaunchArgument(
                "panel_delay_s",
                default_value="1.5",
                description="Delay used to exercise late-joiner behavior.",
            ),
            stack,
        ]
    )
