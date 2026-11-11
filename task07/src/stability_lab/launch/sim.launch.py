"""Launch the public G1 experiment. Each launch owns a fresh Gazebo world."""
import json
from pathlib import Path
import re
import shutil
import tempfile
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from stability_lab.model import build, CONTROLLER, FAULTS


def write_gui_config(folder):
    stock_config = (Path(get_package_prefix('gz_sim_vendor')) /
                    'opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config')
    # Use the demo's three-quarter view while keeping Gazebo's standard plugins.
    camera_pose = '0.785470 -1.151985 0.658437 0 0.192542 1.881711'
    config, count = re.subn(r'<camera_pose>[^<]*</camera_pose>',
                            f'<camera_pose>{camera_pose}</camera_pose>',
                            stock_config.read_text())
    if count != 1:
        raise ValueError(f'Expected one camera pose in {stock_config}, found {count}')
    path = folder / 'gui.config'
    path.write_text(config)
    return path


def launch(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    base, fault = get('base'), get('fault')
    start_paused = get('start_paused') == 'true'
    share = Path(get_package_share_directory('stability_lab'))
    folder = Path(tempfile.mkdtemp(prefix='l07-'))
    gui = get('gui') == 'true'
    try:
        cfg = build(share/'vendor/unitree_g1', folder, base, fault, float(get('physics_step')),
                    lateral_push=get('lateral_push') == 'true')
        if gui:
            gui_config = write_gui_config(folder)
    except Exception:
        shutil.rmtree(folder, ignore_errors=True)
        raise
    print(f'[L07] resources={folder} base={base} fault={fault} spawn_z={cfg["height"]:.6f}', flush=True)
    server_command = ['gz', 'sim', '-s']
    if not start_paused:
        server_command.append('-r')
    server_command.extend(['-v', '3', cfg['world']])
    server = ExecuteProcess(cmd=server_command, output='screen',
                            additional_env={'DISPLAY':'', 'EGL_PLATFORM':'surfaceless'})
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='g1',
               parameters=[{'robot_description':Path(cfg['robot']).read_text(),
                            'frame_prefix':'g1/', 'use_sim_time':True}], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='lab_bridge',
                  parameters=[{'config_file':cfg['bridge']}], output='screen')
    telemetry = Node(package='stability_lab', executable='state_bridge', namespace='g1',
                     parameters=[{'use_sim_time':True, 'config_json':json.dumps(cfg)}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', name='spawn_g1',
                 arguments=['-world','stability_lab','-name','g1','-file',cfg['robot'],
                            '-z',str(cfg['height'] if base == 'free' else 0.0)], output='screen')
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster','-c','/g1/controller_manager',
                                  '--controller-manager-timeout','60','--switch-timeout','30'], output='screen')
    controller = Node(package='controller_manager', executable='spawner',
                      arguments=[CONTROLLER,'-c','/g1/controller_manager',
                                 '--controller-manager-timeout','60','--switch-timeout','30'], output='screen')
    def after(next_action):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason=f'L07 process failed: {event.returncode}'))]
            return [next_action] if next_action is not None else []
        return callback
    first_action = broadcaster
    pause_events = []
    if start_paused:
        # The bridge maps this world's clock to /clock. A fresh paused world
        # stays at time zero, so controller switching can wait for physics.
        first_action = ExecuteProcess(
            name='wait_for_sim_time', output='screen',
            cmd=['ros2', 'topic', 'echo', '/clock', 'rosgraph_msgs/msg/Clock', '--once',
                 '--filter', 'm.clock.sec > 0 or m.clock.nanosec > 0',
                 '--qos-reliability', 'best_effort', '--qos-durability', 'volatile'],
        )
        pause_events.append(RegisterEventHandler(OnProcessExit(
            target_action=first_action, on_exit=after(broadcaster))))
        print('[L07] Waiting for Play in Gazebo before starting controllers.', flush=True)
    actions = [server, rsp, bridge, telemetry, spawn,
        *pause_events,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after(first_action))),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster, on_exit=after(controller))),
        RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=after(None))),
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo server exited'))])),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda context: shutil.rmtree(folder,ignore_errors=True) or [])]))]
    if gui:
        actions.append(ExecuteProcess(cmd=['gz','sim','-g','-v','2','--gui-config',str(gui_config)],
                                      output='screen'))
    if get('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            parameters=[{'use_sim_time':True}], output='screen'))
    return actions


def generate_launch_description():
    defaults = {'base':'free','physics_step':'0.001','gui':'true','rviz':'false'}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        DeclareLaunchArgument('fault', default_value='none', choices=FAULTS),
        DeclareLaunchArgument('lateral_push', default_value='false', choices=('false', 'true'),
                              description='Apply a steady lateral force for the friction comparison'),
        DeclareLaunchArgument('start_paused', default_value='false', choices=('false', 'true'),
                              description='Wait for Play or WorldControl before stepping physics'),
        OpaqueFunction(function=launch),
    ])
