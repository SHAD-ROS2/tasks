"""Launch the homework rover. Configurations are resolved from installed shares."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def start(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    description = get_package_share_directory('rover_description')
    config = LaunchConfiguration('config_file').perform(context)
    xml = xacro.process_file(os.path.join(description, 'urdf', 'rover.urdf.xacro'),
                             mappings={'frame_prefix': prefix,
                                       'mounts_file': LaunchConfiguration('mounts_file').perform(context)}).toxml()
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': xml, 'use_sim_time': sim,
                          'publish_frequency': 30.0}]),
        Node(package='rover_fixture', executable='fixture', namespace=namespace,
             output='screen', parameters=[config, {'use_sim_time': sim, 'frame_prefix': prefix,
                 'moving': LaunchConfiguration('moving').perform(context).lower() == 'true',
                 'delay': float(LaunchConfiguration('delay').perform(context))}]),
    ]
    if LaunchConfiguration('validator').perform(context).lower() == 'true':
        actions.append(Node(package='rover_tf_validator', executable='validator_node',
                            namespace=namespace, output='screen',
                            parameters=[{'use_sim_time': sim, 'frame_prefix': prefix}]))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        with open(os.path.join(description, 'rviz', 'homework.rviz')) as source:
            rviz_text = source.read().replace('@PREFIX@', prefix).replace('@NS@', '/' + namespace if namespace else '')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as generated:
            generated.write(rviz_text)
        actions.append(Node(package='rviz2', executable='rviz2', namespace=namespace,
                            arguments=['-d', generated.name], parameters=[{'use_sim_time': sim}]))
    return actions


def generate_launch_description():
    description = get_package_share_directory('rover_description')
    bringup = get_package_share_directory('rover_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rover'),
        DeclareLaunchArgument('frame_prefix', default_value='rover/'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('validator', default_value='true'),
        DeclareLaunchArgument('moving', default_value='true'),
        DeclareLaunchArgument('delay', default_value='0.20'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(bringup, 'config', 'homework.yaml')),
        DeclareLaunchArgument('mounts_file', default_value=os.path.join(description, 'config', 'homework_mounts.yaml')),
        OpaqueFunction(function=start),
    ])
