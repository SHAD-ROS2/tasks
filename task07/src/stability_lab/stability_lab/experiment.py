"""Bounded public motion/observation client; CSV and plots describe measured runs.

Simulator odometry is ground truth, not an estimate from encoders. Foot motion
uses the latest base pose and measured joints at nearby timestamps. It is an
approximate diagnostic, not a force-based no-slip proof.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from stability_lab.model import frames, pose, FEET


WAIST_YAW = 'waist_yaw_joint'
LEFT_ELBOW = 'left_elbow_joint'
RIGHT_ELBOW = 'right_elbow_joint'
CHATTER_AMPLITUDE_RAD = .16
CHATTER_SEGMENT_S = .40
SWEEP_PROBES = {'yaw_sweep': (WAIST_YAW, .60),
                'elbow_sweep': (LEFT_ELBOW, .75),
                'swapped_joints': (LEFT_ELBOW, .75)}
FEEDBACK_JOINTS = {'waist_yaw': WAIST_YAW, 'left_elbow': LEFT_ELBOW, 'right_elbow': RIGHT_ELBOW}
MOTION_PROFILES = ('steady', 'reference_chatter', *SWEEP_PROBES)
STARTUP_HEIGHT_LIMIT = 500


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def max_abs_vertical_speed(height_samples):
    """Estimate peak speed from successive distinct odometry stamps."""
    return max((abs((z2-z1)/(t2-t1)) for (t1,z1),(t2,z2) in zip(height_samples,height_samples[1:])
                if t2 > t1), default=None)


def motion_waypoints(duration, depth, profile):
    """Return (sim time, hip depth, probe joint offset) for one finite action goal."""
    if profile == 'reference_chatter':
        segments = round(duration / CHATTER_SEGMENT_S)
        return [(0.5, 0.0, 0.0)] + [
            (.5 + i * duration / segments, 0.0, CHATTER_AMPLITUDE_RAD * (-1)**(i-1))
            for i in range(1, segments)
        ] + [(.5 + duration, 0.0, 0.0)]
    offset = SWEEP_PROBES[profile][1] if profile in SWEEP_PROBES else 0.0
    return [(0.5, 0.0, 0.0), (.5 + .4 * duration, depth, offset),
            (.5 + .6 * duration, depth, offset), (.5 + duration, 0.0, 0.0)]


def joint_feedback(feedback, name):
    """Read desired and actual angles from the same named action feedback sample."""
    if feedback is not None:
        try:
            index = feedback.joint_names.index(name)
            return feedback.desired.positions[index], feedback.actual.positions[index]
        except (ValueError, IndexError):
            pass
    return math.nan, math.nan


def motion_joint_names(names, profile):
    result = list(names)
    if profile == 'swapped_joints':
        # Deliberately swap the labels, leaving positions in the original order.
        left, right = result.index(LEFT_ELBOW), result.index(RIGHT_ELBOW)
        result[left], result[right] = result[right], result[left]
    return result


def quaternion_matrix(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    out = np.eye(4)
    out[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    return out


class Experiment(Node):
    def __init__(self):
        super().__init__('experiment', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.cfg = self.root = self.state = self.odom = None
        self.received = {}
        self.contacts = {'left':False,'right':False}
        self.contact_samples = {'left':0,'right':0}
        self.feedback = None
        self.rows = []
        self.startup_height = []
        self.start = None
        self.foot_initial = None
        self.create_subscription(String,'/g1/lab_config',self.on_config,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState,'/g1/joint_states',self.on_state,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/g1/odometry',self.on_odom,qos_profile_sensor_data)
        for side in ('left','right'):
            self.create_subscription(Contacts,f'/g1/contacts/{side}',
                lambda msg,s=side: self.on_contacts(msg,s),
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.client = ActionClient(self,FollowJointTrajectory,'/g1/whole_body_controller/follow_joint_trajectory')
        self.manager = self.create_client(ListControllers,'/g1/controller_manager/list_controllers')
        self.manager_future = None
        self.controller_active = False
        self.goal = None
        self.reason = None
        self.run_parameters = {}

    def on_config(self,msg):
        cfg = json.loads(msg.data)
        if self.root is None:
            self.root = ET.parse(cfg['robot']).getroot()
        self.cfg = cfg

    def on_state(self,msg):
        self.state = msg
        self.received['joints'] = time.monotonic()

    def on_odom(self,msg):
        self.odom = msg
        self.received['base'] = time.monotonic()
        # An observer pre-armed before spawn can see the initial bounce before
        # joint states and contacts make the ordinary monitor ready.
        if (self.run_parameters.get('with_motion') is False and self.start is None
                and len(self.startup_height) < STARTUP_HEIGHT_LIMIT):
            stamp = seconds(msg.header.stamp)
            z = msg.pose.pose.position.z
            if math.isfinite(stamp) and math.isfinite(z) and (
                    not self.startup_height or stamp > self.startup_height[-1][0]):
                self.startup_height.append((stamp,z))

    def on_contacts(self,msg,side):
        self.contacts[side] = bool(msg.contacts)
        self.contact_samples[side] += 1
        self.received['contact_'+side] = time.monotonic()

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def ready(self,fresh=False):
        present = (self.cfg and self.state and (self.cfg['base']=='fixed' or self.odom)
                   and (self.cfg['base']=='fixed' or all(self.contact_samples.values())))
        if not present or not fresh:
            return bool(present)
        values = dict(zip(self.state.name,self.state.position))
        if any(name not in values or not math.isfinite(values[name]) for name in self.cfg['names']):
            return False
        stamps = [seconds(self.state.header.stamp)]
        if self.cfg['base']=='free':
            q,p = self.odom.pose.pose.orientation,self.odom.pose.pose.position
            if (not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0)>.01):
                return False
            stamps.append(seconds(self.odom.header.stamp))
        return all(-.05<=self.now()-stamp<=.20 for stamp in stamps)

    def sample(self):
        if not self.ready():
            return
        now = self.now()
        if self.start is None:
            self.start = now
        joint_age = now-seconds(self.state.header.stamp)
        values = dict(zip(self.state.name,self.state.position))
        if any(name not in values or not math.isfinite(values[name]) for name in self.cfg['names']):
            self.reason = 'joint_measurement_invalid'
            return
        local = frames(self.root,values)
        base = np.eye(4)
        base[2,3] = self.cfg['height']
        base_age = 0.0
        if self.cfg['base']=='free':
            q = self.odom.pose.pose.orientation
            p = self.odom.pose.pose.position
            if (not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0)>.01):
                self.reason = 'base_measurement_invalid'
                return
            base = quaternion_matrix(self.odom.pose.pose.orientation)
            p = self.odom.pose.pose.position
            base[:3,3] = p.x,p.y,p.z
            base_age = now-seconds(self.odom.header.stamp)
        pitch = math.asin(float(np.clip(-base[2,0],-1,1)))
        roll = math.atan2(base[2,1],base[2,2])
        feet = np.array([(base@local[f])[:3,3] for f in FEET])
        if self.foot_initial is None:
            self.foot_initial = feet.copy()
        displacement = np.linalg.norm(feet[:,:2]-self.foot_initial[:,:2],axis=1)
        tracking = math.nan
        if self.feedback and self.feedback.error.positions:
            tracking = max(abs(x) for x in self.feedback.error.positions)
        named_feedback = {}
        for label, name in FEEDBACK_JOINTS.items():
            desired, actual = joint_feedback(self.feedback, name)
            named_feedback[f'{label}_desired_rad'] = desired
            named_feedback[f'{label}_actual_rad'] = actual
            named_feedback[f'{label}_error_rad'] = desired-actual
        row = {'t':now-self.start,'base_z':base[2,3],'roll':roll,'pitch':pitch,
               'left_foot_xy_drift':displacement[0],'right_foot_xy_drift':displacement[1],
               'left_contact':int(self.contacts['left']),'right_contact':int(self.contacts['right']),
               'max_tracking_error':tracking,'joint_age':joint_age,'base_age':base_age,
               'joint_base_stamp_skew':abs(joint_age-base_age) if self.cfg['base']=='free' else 0.0,
               **named_feedback,
               **{name:values.get(name,math.nan) for name in self.cfg['names']}}
        self.rows.append(row)
        # Containment policy for this simulator experiment, not a physical robot E-stop.
        if max(abs(roll),abs(pitch))>.45:
            self.reason = 'tilt_limit'
        elif self.cfg['base']=='free' and base[2,3]<.52:
            self.reason = 'base_height_limit'
        elif max(joint_age,base_age)>.5 or min(joint_age,base_age)<-.05:
            self.reason = 'measurement_time_invalid'
        elif any(time.monotonic()-t>3.0 for t in self.received.values()):
            self.reason = 'wall_watchdog'

    def on_feedback(self,msg):
        self.feedback = msg.feedback

    def check_controller(self):
        if self.manager_future is not None and self.manager_future.done():
            result = self.manager_future.result()
            self.controller_active = any(c.name=='whole_body_controller' and c.state=='active'
                                         for c in result.controller) if result else False
            self.manager_future = None
        if self.manager_future is None and self.manager.service_is_ready():
            self.manager_future = self.manager.call_async(ListControllers.Request())

    def send(self,duration,depth,profile='steady'):
        goal = FollowJointTrajectory.Goal()
        joint_names = self.cfg['names']
        goal.trajectory.joint_names = motion_joint_names(joint_names, profile)
        probe_joint = SWEEP_PROBES[profile][0] if profile in SWEEP_PROBES else WAIST_YAW
        for t,d,offset in motion_waypoints(duration,depth,profile):
            values = pose(self.root,d)
            values[probe_joint] += offset
            point = JointTrajectoryPoint()
            point.positions = [values[n] for n in joint_names]
            point.velocities = [0.0]*len(joint_names)
            sec, nanosec = divmod(round(t*1e9), 10**9)
            point.time_from_start = Duration(sec=sec,nanosec=nanosec)
            goal.trajectory.points.append(point)
        return self.client.send_goal_async(goal,feedback_callback=self.on_feedback)

    def contain(self):
        if self.goal:
            self.goal.cancel_goal_async()
            rclpy.spin_once(self,timeout_sec=.1)
        try:
            result = subprocess.run(['gz','service','-s','/world/stability_lab/control',
                '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                '--timeout','2000','--req','pause: true'],capture_output=True,text=True,timeout=4)
            return result.returncode==0 and 'true' in result.stdout
        except (OSError,subprocess.TimeoutExpired):
            return False

    def save(self,prefix,status,paused):
        prefix = Path(prefix).expanduser()
        prefix.parent.mkdir(parents=True,exist_ok=True)
        if self.rows:
            with Path(str(prefix)+'.csv').open('w') as f:
                writer = csv.DictWriter(f,fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
        startup = []
        if self.startup_height:
            origin = self.start if self.start is not None else self.startup_height[0][0]
            startup = [(stamp-origin, stamp, z) for stamp,z in self.startup_height]
            with Path(str(prefix)+'.startup.csv').open('w') as f:
                writer = csv.writer(f)
                writer.writerow(('t', 'odom_stamp_s', 'base_z'))
                writer.writerows(startup)
        def maximum(key):
            values = [abs(r[key]) for r in self.rows if math.isfinite(r[key])]
            return max(values) if values else None
        report = {'status':status,'abort_reason':self.reason,'simulator_pause_confirmed':paused,
                  'run_parameters':self.run_parameters,
                  'physics_step_s':self.cfg['step'] if self.cfg else None,
                  'spawn_sole_clearance_m':self.cfg.get('spawn_sole_clearance_m') if self.cfg else None,
                  'lateral_push_force_y_n':self.cfg.get('lateral_push_force_y_n') if self.cfg else None,
                  'controller_update_rate_hz':200,
                  'upstream_revision':'725b59b6de39a26c425632120c89f23c49a36f1e',
                  'samples':len(self.rows),'base':self.cfg['base'] if self.cfg else None,
                  'fault':self.cfg['fault'] if self.cfg else None,
                  'max_abs_roll_rad':maximum('roll'),'max_abs_pitch_rad':maximum('pitch'),
                  'min_base_z_m':min((r['base_z'] for r in self.rows),default=None),
                  'max_base_z_m':max([r['base_z'] for r in self.rows] + [z for _,z in self.startup_height],
                                     default=None),
                  'startup_samples':len(self.startup_height),
                  'max_startup_base_z_m':max((z for _,z in self.startup_height),default=None),
                  'max_abs_startup_vertical_speed_m_s':max_abs_vertical_speed(self.startup_height),
                  'max_tracking_error_rad':maximum('max_tracking_error'),
                  'max_joint_base_stamp_skew_s':maximum('joint_base_stamp_skew'),
                  'max_left_foot_xy_drift_m':maximum('left_foot_xy_drift'),
                  'max_right_foot_xy_drift_m':maximum('right_foot_xy_drift'),
                  'contact_samples':self.contact_samples,
                  'both_feet_contact_fraction':sum(r['left_contact'] and r['right_contact'] for r in self.rows)/len(self.rows) if self.rows and all(self.contact_samples.values()) else None,
                  'contact_note':'Gazebo contact messages are event-driven. A fixed pelvis may suspend feet with no messages; missing events alone do not prove no contact.',
                  'measurement_note':'Free base: ground-truth odometry; fixed base: configured anchor pose. Measured joints + approximate foot FK from the generated URDF. Named desired/actual joint angles are paired controller feedback after goal mapping. Position servo is not a torque-limited actuator model.'}
        for label in FEEDBACK_JOINTS:
            report[f'max_abs_{label}_desired_rad'] = maximum(f'{label}_desired_rad')
            report[f'max_abs_{label}_actual_rad'] = maximum(f'{label}_actual_rad')
            report[f'max_{label}_tracking_error_rad'] = maximum(f'{label}_error_rad')
        if startup:
            report['startup_note'] = ('Odometry-only samples before monitor readiness, saved separately; '
                                      'vertical speed is a position finite difference at odometry rate.')
        Path(str(prefix)+'.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if self.rows or startup:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            profile = self.run_parameters.get('profile')
            if self.rows:
                t = [r['t'] for r in self.rows]
                elbow_profile = profile in ('elbow_sweep', 'swapped_joints')
                shown_joints = ('left_elbow', 'right_elbow') if elbow_profile else ('waist_yaw',)
                plot_feedback = any(report[f'max_{label}_tracking_error_rad'] is not None for label in shown_joints)
                fig, axes = plt.subplots(4 if plot_feedback else 3,1,
                                         figsize=(11,9.5 if plot_feedback else 8),sharex=True,layout='constrained')
                axes[0].plot(t,[math.degrees(r['roll']) for r in self.rows],label='roll')
                axes[0].plot(t,[math.degrees(r['pitch']) for r in self.rows],label='pitch')
                axes[0].set_ylabel('Base angle [deg]')
                height_axis = axes[1]
                height_axis.plot(t,[r['base_z'] for r in self.rows],label='pelvis height')
                axes[2].plot(t,[r['left_foot_xy_drift'] for r in self.rows],label='left foot')
                axes[2].plot(t,[r['right_foot_xy_drift'] for r in self.rows],label='right foot')
                axes[2].set_ylabel('Foot XY drift [m]')
                if plot_feedback:
                    for label in shown_joints:
                        name = label.replace('_', ' ')
                        actual, = axes[3].plot(t,[r[f'{label}_actual_rad'] for r in self.rows],label=f'{name} actual')
                        axes[3].plot(t,[r[f'{label}_desired_rad'] for r in self.rows],label=f'{name} desired',
                                     color=actual.get_color(),linestyle='--')
                    axes[3].set_ylabel('Elbow angle [rad]' if elbow_profile else 'Waist yaw [rad]')
            else:
                fig, height_axis = plt.subplots(figsize=(11,4),layout='constrained')
                axes = [height_axis]
            height_axis.set_ylabel('Height [m]')
            if startup:
                height_axis.plot([t for t,_,_ in startup], [z for _,_,z in startup],
                                 label='startup odometry', color='#c37a12', linestyle='--',
                                 marker='.' if len(startup)==1 else None)
                if self.rows:
                    height_axis.axvline(0.0,color='0.55',linestyle=':',linewidth=1)
            axes[-1].set_xlabel('Simulation time [s]')
            for ax in axes:
                ax.grid(alpha=.2)
                ax.legend(loc='best')
            title = f'Measured G1 experiment: {report["base"]}, {report["fault"]}, {status}'
            fig.suptitle(f'{title}, {profile}' if profile else title)
            fig.savefig(str(prefix)+'.png',dpi=150)
            plt.close(fig)
        print(json.dumps(report,indent=2),flush=True)


def run(with_motion):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration',type=float,default=12.0,help='Motion/observation duration in simulation seconds')
    parser.add_argument('--depth',type=float,help='Hip flexion amplitude in radians (knee amplitude is twice this)')
    if with_motion:
        parser.add_argument('--profile',choices=MOTION_PROFILES,default='steady',
                            help='Normal motion, a single-joint probe, or a bounded reference fault')
    parser.add_argument('--output',default='evidence/run',help='New output prefix: .json .csv .png and optional .startup.csv')
    args,ros_args = parser.parse_known_args()
    profile = args.profile if with_motion else None
    standing_profile = profile not in (None, 'steady')
    if args.depth is None:
        args.depth = 0.0 if standing_profile else .18
    if not 4 <= args.duration <= 60 or not 0 <= args.depth <= .25:
        parser.error('duration must be 4..60 s; depth must be 0..0.25 rad')
    if standing_profile and args.depth != 0.0:
        parser.error(f'{profile} uses a standing leg pose; set --depth 0 or omit --depth')
    if any(Path(args.output+suffix).expanduser().exists() for suffix in ('.json','.csv','.png','.startup.csv')):
        parser.error('Use a new output prefix; existing evidence is preserved')
    # Leave Python's SIGINT handler active so containment can use the ROS
    # context before shutting it down in finally.
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node = Experiment()
    node.run_parameters = {'duration_s':args.duration,'hip_depth_rad':args.depth,
                           'with_motion':with_motion,'profile':profile}
    if profile in SWEEP_PROBES:
        intended, offset = SWEEP_PROBES[profile]
        node.run_parameters.update(probe_intended_joint=intended,
                                   probe_commanded_joint=RIGHT_ELBOW if profile == 'swapped_joints' else intended,
                                   probe_offset_rad=offset)
    status,paused = 'not_started',False
    try:
        deadline = time.monotonic()+45
        while (rclpy.ok() and time.monotonic()<deadline and
               not (node.ready(fresh=True) and (not with_motion or
                    (node.client.server_is_ready() and node.controller_active)))):
            if with_motion:
                node.check_controller()
            rclpy.spin_once(node,timeout_sec=.1)
        if not node.ready(fresh=True) or (with_motion and not node.controller_active):
            node.reason = 'readiness_timeout'
            raise RuntimeError('No fresh model/joints/base telemetry within 45 wall seconds')
        node.sample()
        if node.reason:
            raise RuntimeError('Preflight measurement/safety check failed: '+node.reason)
        result_future = None
        if with_motion:
            future = node.send(args.duration,args.depth,profile)
            rclpy.spin_until_future_complete(node,future,timeout_sec=10)
            if not future.done() or not future.result().accepted:
                raise RuntimeError('Goal rejected or goal response timed out')
            node.goal = future.result()
            result_future = node.goal.get_result_async()
        node.sample()
        wall_deadline = time.monotonic()+max(90,args.duration*15)
        stop_time = node.now()+args.duration+(1.5 if with_motion else 0)
        last_sample = -1.0
        status = 'running'
        while rclpy.ok() and node.now()<stop_time and time.monotonic()<wall_deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.now()-last_sample>=.019:
                node.sample()
                last_sample=node.now()
            if any(time.monotonic()-t>3.0 for t in node.received.values()):
                node.reason = 'wall_watchdog'
            if node.reason:
                status='aborted'
                paused=node.contain()
                break
        if status=='running':
            if time.monotonic()>=wall_deadline:
                node.reason='wall_deadline'
                status='aborted'
                paused=node.contain()
            elif with_motion:
                if not result_future.done():
                    status='action_result_missing'
                    paused=node.contain()
                else:
                    result=result_future.result()
                    status='succeeded' if result.status==4 and result.result.error_code==0 else f'action_failed_{result.result.error_code}'
                    if status!='succeeded':
                        node.reason='trajectory_action_failed'
                        paused=node.contain()
            else:
                status='observed'
    except KeyboardInterrupt:
        node.reason='interrupted'
        status='aborted'
        paused=node.contain()
    except RuntimeError as exc:
        print(str(exc),flush=True)
        node.reason=node.reason or str(exc)
        status='failed'
        paused=node.contain()
    finally:
        node.save(args.output,status,paused)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if status in ('succeeded','observed') else 2


def motion_main():
    raise SystemExit(run(True))


def observe_main():
    raise SystemExit(run(False))
