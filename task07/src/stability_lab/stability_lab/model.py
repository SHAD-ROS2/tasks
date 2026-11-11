"""Create a course wrapper without changing the pinned upstream G1 model.

Position actuation uses the supplied gz_ros2_control servo. It is not a
torque-limited motor model, and the free-base example is not a balance controller.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import yaml

MODEL_FILE = 'g1_23dof_mode_10.urdf'
FEET = ('left_ankle_roll_link', 'right_ankle_roll_link')
CONTROLLER = 'whole_body_controller'
DROP_FAULTS = ('spawn_drop', 'spawn_bounce')
FAULTS = ('none', 'slippery_feet', 'torso_com', 'inertia', 'joint_axis', 'link_mass',
          *DROP_FAULTS, 'slow_servo')
LATERAL_PUSH_FORCE_N = 15.0


def transform(xyz='0 0 0', rpy='0 0 0'):
    x, y, z = map(float, xyz.split())
    r, p, a = map(float, rpy.split())
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    out = np.eye(4)
    out[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                   [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    out[:3, 3] = x, y, z
    return out


def rotation(axis, angle):
    v = np.array(list(map(float, axis.split())))
    v /= np.linalg.norm(v)
    x, y, z = v
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    out = np.eye(4)
    out[:3, :3] += math.sin(angle)*k + (1-math.cos(angle))*(k@k)
    return out


def frames(root, positions):
    result = {'pelvis': np.eye(4)}
    pending = [j for j in root.findall('joint') if j.find('parent').get('link') != 'world']
    while pending:
        progress = False
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in result:
                continue
            origin = j.find('origin')
            mat = transform(origin.get('xyz', '0 0 0'), origin.get('rpy', '0 0 0')) if origin is not None else np.eye(4)
            if j.get('type') in ('revolute', 'continuous'):
                mat = mat @ rotation(j.find('axis').get('xyz'), positions.get(j.get('name'), 0.0))
            result[child] = result[parent] @ mat
            pending.remove(j)
            progress = True
        if not progress:
            raise ValueError('Disconnected model or unknown root')
    return result


def joint_names(root):
    return [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute']


def pose(root, depth=0.0):
    """Small symmetric leg flexion, depth in rad (hip angle), not metres."""
    out = {name: 0.0 for name in joint_names(root)}
    for side in ('left', 'right'):
        out[f'{side}_hip_pitch_joint'] = -depth
        out[f'{side}_knee_joint'] = 2*depth
        out[f'{side}_ankle_pitch_joint'] = -depth
        out[f'{side}_shoulder_roll_joint'] = 0.12 if side == 'left' else -0.12
        out[f'{side}_elbow_joint'] = 0.15
    return out


def source_root(vendor):
    return ET.parse(Path(vendor)/MODEL_FILE).getroot()


def apply_model_fault(root, fault):
    if fault == 'torso_com':
        inertial = root.find("link[@name='torso_link']/inertial/origin")
        xyz = list(map(float, inertial.get('xyz').split()))
        # Deliberately severe unit/frame error: shifts whole-robot COM beyond
        # the front support edge. This is a teaching fault, not calibration.
        xyz[0] += .60
        inertial.set('xyz', ' '.join(map(str, xyz)))
    elif fault == 'inertia':
        tensor = root.find("link[@name='left_knee_link']/inertial/inertia")
        # Treating kg·mm² values as kg·m² multiplies the entire tensor by 10⁶.
        # Keep the shank's mass, COM and tensor orientation unchanged.
        for component in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
            tensor.set(component, f'{float(tensor.get(component)) * 1e6:.12g}')
    elif fault == 'joint_axis':
        # Both knees are straight at initialization. The error appears when
        # the left knee bends about an axis tilted 20° from local +Y toward +X.
        # This gives visible lateral motion without intersecting the floor in
        # the fixed-base diagnostic squat (depth 0..0.18 rad).
        root.find("joint[@name='left_knee_joint']/axis").set('xyz', '0.342020143 0.939692621 0')
    elif fault == 'link_mass':
        # Read the left forearm's mass in grams as kilograms. Preserve the
        # link COM and inertia fields so this remains a one-parameter fault.
        mass = root.find("link[@name='left_elbow_link']/inertial/mass")
        mass.set('value', f'{float(mass.get("value")) * 1e3:.12g}')


def build(vendor, folder, base='free', fault='none', step=0.001, *, lateral_push=False):
    if base not in ('free', 'fixed') or fault not in FAULTS:
        raise ValueError('Unknown base/fault option')
    if base == 'fixed' and (fault in DROP_FAULTS or lateral_push):
        raise ValueError('The drop and lateral push demonstrations require base:=free')
    if not 0.0002 <= step <= 0.004:
        raise ValueError('physics_step must lie between .0002 and .004 seconds')
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    root = source_root(vendor)
    root.set('name', 'g1')
    mujoco = root.find('mujoco')
    if mujoco is not None:
        root.remove(mujoco)
    apply_model_fault(root, fault)
    initial = pose(root)
    names = joint_names(root)
    local = frames(root, initial)
    low = min(float((local[foot]@np.array([0,0,-.035,1]))[2]) for foot in FEET)
    spawn_clearance = .20 if fault in DROP_FAULTS else .003
    height = -low + spawn_clearance
    for mesh in root.findall('.//mesh'):
        mesh.set('filename', (Path(vendor)/mesh.get('filename')).resolve().as_uri())
    for j in root.findall('joint'):
        if j.get('type') == 'revolute':
            dynamics = j.find('dynamics')
            if dynamics is None:
                dynamics = ET.SubElement(j, 'dynamics')
            dynamics.set('damping', '0.15')
            dynamics.set('friction', '0.0')
    # DART multiplies the restitution coefficients of the contact pair. Both
    # drop scenes use the same floor. Only spawn_bounce makes the soles elastic.
    elastic_surface = '<surface><bounce><restitution_coefficient>0.9</restitution_coefficient><threshold>0</threshold></bounce></surface>'
    for foot in FEET:
        link = root.find(f"link[@name='{foot}']")
        contacts = []
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', f'{foot}_contact_{index}')
            # libsdformat's URDF converter appends these suffixes to named
            # collisions. Sensor references must match the converted SDF names.
            contacts.append(collision.get('name') + '_collision' + (f'_{index}' if index else ''))
        gz = ET.SubElement(root, 'gazebo', reference=foot)
        mu = '0.02' if fault == 'slippery_feet' else '0.9'
        ET.SubElement(gz, 'mu1').text = mu
        ET.SubElement(gz, 'mu2').text = mu
        if fault == 'spawn_bounce':
            ET.SubElement(gz, 'collision').append(ET.fromstring(elastic_surface))
        sensor = ET.SubElement(gz, 'sensor', name=f'{foot}_sensor', type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '50'
        topic = '/g1/contacts/' + ('left' if foot.startswith('left') else 'right')
        ET.SubElement(sensor, 'topic').text = topic
        contact = ET.SubElement(sensor, 'contact')
        # Gazebo Sim Contact reads the contact element's topic, not only the
        # generic sensor topic. Set both to keep the ROS bridge explicit.
        ET.SubElement(contact, 'topic').text = topic
        for collision in contacts:
            ET.SubElement(contact, 'collision').text = collision
    if base == 'fixed':
        ET.SubElement(root, 'link', name='world')
        anchor = ET.SubElement(root, 'joint', name='course_anchor', type='fixed')
        ET.SubElement(anchor, 'parent', link='world')
        ET.SubElement(anchor, 'child', link='pelvis')
        ET.SubElement(anchor, 'origin', xyz=f'0 0 {height}')
    controller_file = folder/'controllers.yaml'
    yaml.safe_dump({'/g1/controller_manager': {'ros__parameters': {
        'update_rate': 200,
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
        CONTROLLER: {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        f'/g1/{CONTROLLER}': {'ros__parameters': {
            'joints': names, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'allow_partial_joints_goal': False, 'action_monitor_rate': 20.0,
            'constraints': {'goal_time': 2.0, 'stopped_velocity_tolerance': .10,
                            **{name: {'trajectory': .20, 'goal': .06} for name in names}}}}},
        controller_file.open('w'))
    control = ET.SubElement(root, 'ros2_control', name='GazeboSimSystem', type='system')
    ET.SubElement(ET.SubElement(control,'hardware'),'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    for name in names:
        node = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(node,'command_interface',name='position')
        state = ET.SubElement(node,'state_interface',name='position')
        ET.SubElement(state,'param',name='initial_value').text = str(initial[name])
        ET.SubElement(node,'state_interface',name='velocity')
        ET.SubElement(node,'state_interface',name='effort')
    gz = ET.SubElement(root,'gazebo')
    plugin = ET.SubElement(gz,'plugin',filename='libgz_ros2_control-system.so',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text = str(controller_file)
    ET.SubElement(plugin,'hold_joints').text = 'true'
    ET.SubElement(plugin,'position_proportional_gain').text = '0.015' if fault == 'slow_servo' else '0.15'
    ET.SubElement(ET.SubElement(plugin,'ros'),'namespace').text = '/g1'
    if base == 'free':
        odom = ET.SubElement(gz,'plugin',filename='gz-sim-odometry-publisher-system',
                             name='gz::sim::systems::OdometryPublisher')
        for key, value in {'odom_frame':'world','robot_base_frame':'g1/pelvis',
                           'odom_publish_frequency':'50','odom_topic':'/g1/odometry',
                           'dimensions':'3'}.items():
            ET.SubElement(odom,key).text = value
    description = folder/'robot.urdf'
    ET.indent(root)
    ET.ElementTree(root).write(description,encoding='unicode',xml_declaration=True)
    world = folder/'world.sdf'
    wrench = ''
    if lateral_push:
        # A reproducible horizontal load on the model's canonical link, the
        # pelvis. Use the same load in both friction runs; no GUI timing needed.
        wrench = f'''
      <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench">
        <persistent><entity_name>g1</entity_name><entity_type>model</entity_type><force>0 {LATERAL_PUSH_FORCE_N:g} 0</force></persistent>
      </plugin>'''
    floor_surface = elastic_surface if fault in DROP_FAULTS else ''
    world.write_text(f'''<sdf version="1.9"><world name="stability_lab">
      <physics name="physics" type="ignored"><max_step_size>{step}</max_step_size><real_time_factor>1</real_time_factor></physics>
      <gravity>0 0 -9.81</gravity>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
      <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>{wrench}
      <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.94 0.96 0.98 1</background><shadows>true</shadows></scene>
      <light name="key" type="directional"><pose>0 0 8 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.5 -1</direction><cast_shadows>true</cast_shadows></light>
      <model name="floor"><static>true</static><link name="floor"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>{floor_surface}</collision><visual name="floor"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>0.65 0.7 0.74 1</ambient><diffuse>0.65 0.7 0.74 1</diffuse></material></visual></link></model>
    </world></sdf>''')
    # World-scoped clock remains available even when Gazebo discovers another
    # global /clock publisher (including a previous world during restart).
    bridge = [{'ros_topic_name':'/clock','gz_topic_name':'/world/stability_lab/clock',
               'ros_type_name':'rosgraph_msgs/msg/Clock','gz_type_name':'gz.msgs.Clock','direction':'GZ_TO_ROS'},
              {'ros_topic_name':'/g1/odometry','gz_topic_name':'/g1/odometry',
               'ros_type_name':'nav_msgs/msg/Odometry','gz_type_name':'gz.msgs.Odometry','direction':'GZ_TO_ROS'}]
    for side in ('left','right'):
        bridge.append({'ros_topic_name':f'/g1/contacts/{side}','gz_topic_name':f'/g1/contacts/{side}',
                       'ros_type_name':'ros_gz_interfaces/msg/Contacts','gz_type_name':'gz.msgs.Contacts','direction':'GZ_TO_ROS'})
    bridge_file = folder/'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridge))
    return {'robot':str(description),'world':str(world),'bridge':str(bridge_file),
            'base':base,'fault':fault,'height':height,'step':step,'names':names,
            'spawn_sole_clearance_m':spawn_clearance,
            'lateral_push_force_y_n':LATERAL_PUSH_FORCE_N if lateral_push else 0.0,
            'vendor':str(vendor),'initial':initial}
