"""A correct cart sandbox, separate from the rover incident."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    namespace, prefix = value('namespace').strip('/'), value('frame_prefix')
    sim = value('use_sim_time').lower() == 'true'
    description = get_package_share_directory('rover_description')
    xml = xacro.process_file(os.path.join(description, 'urdf', 'cart.urdf.xacro'),
                             mappings={key: value(key) for key in ('frame_prefix', 'mount_x', 'visual_x', 'arm_axis')}).toxml()
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': xml, 'use_sim_time': sim, 'publish_frequency': 30.0}]),
        Node(package='rover_fixture', executable='fixture', namespace=namespace, output='screen',
             parameters=[value('config_file'), {'use_sim_time': sim, 'frame_prefix': prefix,
                          'mount_x': float(value('mount_x')), 'arm_axis': value('arm_axis'),
                          'joint_angle': float(value('joint_angle')), 'delay': float(value('delay')),
                          'moving': value('moving').lower() == 'true'}]),
    ]
    if value('rviz').lower() == 'true':
        with open(os.path.join(description, 'rviz', 'practice.rviz')) as source:
            rviz_text = source.read().replace('@PREFIX@', prefix).replace('@NS@', '/' + namespace if namespace else '')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as generated:
            generated.write(rviz_text)
        actions.append(Node(package='rviz2', executable='rviz2', namespace=namespace,
                            arguments=['-d', generated.name], parameters=[{'use_sim_time': sim}]))
    return actions


def generate_launch_description():
    bringup = get_package_share_directory('rover_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rover'),
        DeclareLaunchArgument('frame_prefix', default_value='rover/'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mount_x', default_value='0.05'),
        DeclareLaunchArgument('visual_x', default_value='0.10'),
        DeclareLaunchArgument('arm_axis', default_value='0 0 1'),
        DeclareLaunchArgument('joint_angle', default_value='0.40'),
        DeclareLaunchArgument('delay', default_value='0.20'),
        DeclareLaunchArgument('moving', default_value='true'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(bringup, 'config', 'practice.yaml')),
        OpaqueFunction(function=start),
    ])
