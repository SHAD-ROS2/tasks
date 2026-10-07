"""Both activities use installed resources; arguments never depend on cwd."""
from pathlib import Path
import shutil
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rover_sim_lab.configuration import load_settings, validate_settings, names, bridge_config, sdf, urdf


def launch(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    share = Path(get_package_share_directory('rover_sim_lab'))
    config = load_settings(share, get('mode'))
    validate_settings(config)
    ns, prefix, world = names(get('namespace'), get('frame_prefix'), get('world_name'))
    folder = Path(tempfile.mkdtemp(prefix='l04-'))
    (folder/'world.sdf').write_text(sdf(config, prefix, world, get('profile'), float(get('target_rtf'))))
    (folder/'rover.urdf').write_text(urdf())
    (folder/'bridge.yaml').write_text(yaml.safe_dump(bridge_config(config, ns, prefix, world)))
    (folder/'view.rviz').write_text((share/'rviz/lab.rviz').read_text().replace('_NS_', ns)
        .replace('_TF_PREFIX_', prefix.rstrip('/')).replace('_PREFIX_', prefix))
    print(f'[L04] Generated installed resources: {folder}', flush=True)
    server_args = ['gz', 'sim', '-s', '--headless-rendering', '-v', '3', '--seed', get('seed')]
    if get('paused') != 'true':
        server_args += ['-r']
    server_args += [str(folder/'world.sdf')]
    server = ExecuteProcess(cmd=server_args, output='screen',
                            additional_env={'DISPLAY': '', 'EGL_PLATFORM': 'surfaceless'})
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
                  parameters=[{'config_file': str(folder/'bridge.yaml'), 'use_sim_time': True}], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=ns,
               parameters=[{'robot_description': urdf(), 'frame_prefix': prefix, 'use_sim_time': True}],
               remappings=[('joint_states', f'/{ns}/joint_states')], output='screen')
    actions = [server, bridge, rsp,
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: shutil.rmtree(folder, ignore_errors=True) or [])]))]
    if get('gui') == 'true':
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'], output='screen'))
    if get('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
            arguments=['-d', str(folder/'view.rviz')], parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    defaults = {'mode': 'practice', 'gui': 'true', 'rviz': 'true', 'namespace': 'rover',
                'frame_prefix': 'rover/', 'world_name': 'lab', 'seed': '42',
                'target_rtf': '1.0', 'profile': 'ideal', 'paused': 'false'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=launch)])
