from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("warehouse_bringup")
    stack_launch = os.path.join(package_share, "launch", "warehouse.launch.py")
    default_params = os.path.join(package_share, "config", "warehouse.yaml")

    robot_1_namespace = LaunchConfiguration("robot_1_namespace")
    robot_2_namespace = LaunchConfiguration("robot_2_namespace")
    params_file = LaunchConfiguration("params_file")
    panel_delay_s = LaunchConfiguration("panel_delay_s")

    def include_stack(namespace: LaunchConfiguration) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack_launch),
            launch_arguments={
                "namespace": namespace,
                "params_file": params_file,
                "panel_delay_s": panel_delay_s,
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_1_namespace", default_value="robot_1"),
            DeclareLaunchArgument("robot_2_namespace", default_value="robot_2"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("panel_delay_s", default_value="1.5"),
            include_stack(robot_1_namespace),
            include_stack(robot_2_namespace),
        ]
    )
