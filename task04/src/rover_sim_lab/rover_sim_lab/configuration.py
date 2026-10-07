"""Build installed, local-only resources for the two laboratory scenes."""
from pathlib import Path
from html import escape
import copy
import re
import yaml

HOMEWORK_ENDPOINTS = {'command': 'actuator/velocity', 'scan': 'front/ranges',
                      'imu': 'body/inertial', 'image': 'front/rgb', 'info': 'front/calibration'}
SYSTEMS = {'Physics': 'physics', 'UserCommands': 'user-commands',
           'SceneBroadcaster': 'scene-broadcaster', 'Sensors': 'sensors', 'Imu': 'imu'}


def load_settings(share, mode):
    root = Path(share)
    if mode == 'practice':
        return yaml.safe_load((root / 'practice/settings.yaml').read_text())
    if mode != 'homework':
        raise ValueError('mode must be practice or homework')
    settings = yaml.safe_load((root / 'homework/systems.yaml').read_text())
    settings = copy.deepcopy(settings)
    settings.update(mode=mode, model_name='inspection_cart', gz_root='/lab/homework',
                    systems=['Physics', 'UserCommands', 'SceneBroadcaster'] + settings.pop('sensor_systems'),
                    bridge=yaml.safe_load((root / 'homework/wiring.yaml').read_text()))
    return settings


def validate_settings(settings):
    errors = []
    for name in settings['systems']:
        if name not in SYSTEMS:
            errors.append(f'Unknown system {name}; allowed: {list(SYSTEMS)}')
    for channel, row in settings['bridge'].items():
        if any(str(v) == 'TODO' for v in row.values()):
            errors.append(f'wiring.{channel}: complete TODO fields')
        if row['direction'] not in ('ROS_TO_GZ', 'GZ_TO_ROS'):
            errors.append(f'wiring.{channel}: direction must be ROS_TO_GZ or GZ_TO_ROS')
    for name, rate in settings['rates'].items():
        if not isinstance(rate, (float, int)) or not 0 < rate <= 200:
            errors.append(f'rates.{name}: use a positive rate <= 200 Hz')
    if errors:
        raise ValueError('\n'.join(errors))


def names(namespace='rover', frame_prefix='rover/', world_name='lab'):
    namespace = namespace.strip('/')
    frame_prefix = frame_prefix.strip('/') + '/'
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', namespace):
        raise ValueError('namespace must be a nonempty relative ROS namespace')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', frame_prefix):
        raise ValueError('invalid frame_prefix')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', world_name):
        raise ValueError('world_name must contain letters, numbers or underscores')
    return namespace, frame_prefix, world_name


def bridge_config(settings, namespace, prefix, world):
    root = settings['gz_root']
    result = []
    for row in settings['bridge'].values():
        result.append(dict(ros_topic_name=f"/{namespace}/{row['ros']}",
                           gz_topic_name=f"{root}/{row['gz']}",
                           ros_type_name=row['ros_type'], gz_type_name=row['gz_type'],
                           direction=row['direction'], lazy=False))
    supplied = [
        ('odom', f'{root}/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        ('joint_states', f'{root}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
        ('ground_truth', f"/model/{settings['model_name']}/pose", 'geometry_msgs/msg/PoseArray', 'gz.msgs.Pose_V'),
        ('/tf', f'{root}/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V'),
        ('/clock', f'/world/{world}/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
    ]
    for ros, gz, rt, gt in supplied:
        result.append(dict(ros_topic_name=ros if ros.startswith('/') else f'/{namespace}/{ros}',
                           gz_topic_name=gz, ros_type_name=rt, gz_type_name=gt,
                           direction='GZ_TO_ROS', lazy=False))
    return result


def plugin(name):
    extra = '<render_engine>ogre2</render_engine>' if name == 'Sensors' else ''
    return f'<plugin filename="gz-sim-{SYSTEMS[name]}-system" name="gz::sim::systems::{name}">{extra}</plugin>'


def noise(profile, gyro=False):
    sigma = (0.02 if gyro else 0.05) if profile != 'ideal' else 0.0
    mean = 0.08 if profile == 'biased' and gyro else 0.0
    return f'<noise type="gaussian"><mean>{mean}</mean><stddev>{sigma}</stddev></noise>'


def sdf(settings, prefix, world, profile='ideal', target_rtf=1.0):
    if profile not in ('ideal', 'white', 'biased'):
        raise ValueError('profile: ideal | white | biased')
    if not 0 < target_rtf <= 2:
        raise ValueError('target_rtf must be > 0 and <= 2')
    s = settings
    root = s['gz_root']
    rates, cam = s['rates'], s['camera']
    endpoints = HOMEWORK_ENDPOINTS if s['mode'] == 'homework' else {k:v['gz'] for k,v in s['bridge'].items()}
    topic = lambda channel: escape(f"{root}/{endpoints[channel]}")
    inertial = '<inertial><mass>3</mass><inertia><ixx>0.04</ixx><iyy>0.06</iyy><izz>0.07</izz></inertia></inertial>'
    wheels = ''
    for side, y in [('left', 0.18), ('right', -0.18)]:
        wheels += f'''
        <link name="{side}_wheel"><pose>0.04 {y} -0.06 1.57079632679 0 0</pose>
          <inertial><mass>0.2</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0008</izz></inertia></inertial>
          <collision name="wheel"><geometry><cylinder><radius>0.09</radius><length>0.04</length></cylinder></geometry></collision>
          <visual name="wheel"><geometry><cylinder><radius>0.09</radius><length>0.04</length></cylinder></geometry><material><diffuse>0.12 0.13 0.16 1</diffuse></material></visual>
        </link>
        <joint name="{side}_wheel_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child>
          <axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
        </joint>'''
    imu_axes = ''.join(f'<{axis}>{noise(profile, True)}</{axis}>' for axis in 'xyz')
    acc_axes = ''.join(f'<{axis}>{noise(profile)}</{axis}>' for axis in 'xyz')
    # Practice and homework use different locally stored obstacle layouts.
    wall_x = 3.0 if s['mode'] == 'practice' else 2.4
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="{world}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>{target_rtf}</real_time_factor></physics>
  <gravity>0 0 -9.81</gravity>
  {''.join(plugin(n) for n in s['systems'])}
  <light name="sun" type="directional"><pose>0 0 8 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -1</direction><cast_shadows>false</cast_shadows></light>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.82 0.9 1</background><shadows>false</shadows></scene>
  <model name="ground"><static>true</static><link name="ground"><collision name="ground"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision><visual name="ground"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><diffuse>0.7 0.73 0.76 1</diffuse></material></visual></link></model>
  <model name="front_wall"><static>true</static><pose>{wall_x} 0 0.5 0 0 0</pose><link name="wall"><collision name="wall"><geometry><box><size>0.1 4 1</size></box></geometry></collision><visual name="wall"><geometry><box><size>0.1 4 1</size></box></geometry><material><diffuse>0.85 0.28 0.16 1</diffuse></material></visual></link></model>
  <model name="side_marker"><static>true</static><pose>1.3 1 0.25 0 0 0</pose><link name="marker"><collision name="marker"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></collision><visual name="marker"><geometry><box><size>0.3 0.3 0.5</size></box></geometry><material><diffuse>0.1 0.3 0.85 1</diffuse></material></visual></link></model>
  <model name="{s['model_name']}"><pose>0 0 0.15 0 0 0</pose>
    <link name="base_link">{inertial}
      <collision name="body"><geometry><box><size>0.44 0.28 0.08</size></box></geometry></collision>
      <visual name="body"><geometry><box><size>0.44 0.28 0.08</size></box></geometry><material><diffuse>0.08 0.55 0.52 1</diffuse></material></visual>
      <collision name="caster"><pose>-0.16 0 -0.115 0 0 0</pose><geometry><sphere><radius>0.035</radius></sphere></geometry><surface><friction><ode><mu>0.001</mu><mu2>0.001</mu2></ode></friction></surface></collision>
      <visual name="caster"><pose>-0.16 0 -0.115 0 0 0</pose><geometry><sphere><radius>0.035</radius></sphere></geometry></visual>
      <sensor name="front_lidar" type="gpu_lidar"><pose>0.13 0 0.10 0 0 0</pose><topic>{topic('scan')}</topic><gz_frame_id>{prefix}lidar_frame</gz_frame_id><update_rate>{rates['scan']}</update_rate>
        <lidar><scan><horizontal><samples>180</samples><resolution>1</resolution><min_angle>-1.57079632679</min_angle><max_angle>1.57079632679</max_angle></horizontal></scan><range><min>0.08</min><max>12</max><resolution>0.01</resolution></range></lidar>
      </sensor>
      <sensor name="body_imu" type="imu"><pose>0.02 0 0.02 0 0 0</pose><topic>{topic('imu')}</topic><gz_frame_id>{prefix}imu_frame</gz_frame_id><update_rate>{rates['imu']}</update_rate><imu><angular_velocity>{imu_axes}</angular_velocity><linear_acceleration>{acc_axes}</linear_acceleration></imu></sensor>
      <sensor name="front_camera" type="camera"><pose>0.21 0 0.08 0 0 0</pose><topic>{topic('image')}</topic><gz_frame_id>{prefix}camera_optical_frame</gz_frame_id><update_rate>{rates['camera']}</update_rate>
        <camera><horizontal_fov>1.2</horizontal_fov><image><width>{cam['width']}</width><height>{cam['height']}</height><format>R8G8B8</format></image><clip><near>0.05</near><far>15</far></clip><camera_info_topic>{topic('info')}</camera_info_topic></camera>
      </sensor>
    </link>{wheels}
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive"><left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint><wheel_separation>0.36</wheel_separation><wheel_radius>0.09</wheel_radius><topic>{topic('command')}</topic><odom_topic>{root}/odometry</odom_topic><tf_topic>{root}/tf</tf_topic><frame_id>{prefix}odom</frame_id><child_frame_id>{prefix}base_link</child_frame_id><odom_publish_frequency>{rates['odom']}</odom_publish_frequency></plugin>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>{root}/joint_states</topic><joint_name>left_wheel_joint</joint_name><joint_name>right_wheel_joint</joint_name></plugin>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><publish_nested_model_pose>false</publish_nested_model_pose><use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>50</update_frequency></plugin>
  </model>
</world></sdf>'''


def urdf():
    wheel_links = ''
    for side, y in [('left', 0.18), ('right', -0.18)]:
        wheel_links += f'''<link name="{side}_wheel"><visual><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.09" length="0.04"/></geometry><material name="dark"><color rgba="0.12 0.13 0.16 1"/></material></visual></link>
        <joint name="{side}_wheel_joint" type="continuous"><parent link="base_link"/><child link="{side}_wheel"/><origin xyz="0.04 {y} -0.06"/><axis xyz="0 1 0"/></joint>'''
    fixed = ''
    for parent, name, xyz, rpy in [('base_link','lidar_frame','0.13 0 0.10','0 0 0'),
        ('base_link','imu_frame','0.02 0 0.02','0 0 0'), ('base_link','camera_link','0.21 0 0.08','0 0 0'),
        ('camera_link','camera_optical_frame','0 0 0','-1.57079632679 0 -1.57079632679')]:
        fixed += f'<link name="{name}"/><joint name="{name}_fixed" type="fixed"><parent link="{parent}"/><child link="{name}"/><origin xyz="{xyz}" rpy="{rpy}"/></joint>'
    return f'''<?xml version="1.0"?><robot name="l04_cart"><link name="base_link"><visual><geometry><box size="0.44 0.28 0.08"/></geometry><material name="teal"><color rgba="0.08 0.55 0.52 1"/></material></visual></link>{wheel_links}{fixed}</robot>'''
