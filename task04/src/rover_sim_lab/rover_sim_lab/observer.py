"""Supplied transport, collection and world-control infrastructure.

All deadlines use monotonic time. Measurements retain sensor stamps.
The homework implements its own epoch-aware metrics in homework_metrics.py.
"""
from collections import defaultdict
import math
import statistics
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, Imu, Image, CameraInfo, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, TransformException
from gz.transport13 import Node as GzNode
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.twist_pb2 import Twist as GzTwist
from google.protobuf import text_format


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def gzseconds(stamp):
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class Observer(Node):
    def __init__(self, args):
        super().__init__('l04_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.streams = defaultdict(list)
        self.clock = None
        self.stats = None
        self.stats_serial = 0
        self.gz_command = None
        self.clock_jumps = []
        self.subscriptions_ = []
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.gz = GzNode()
        self.gz.subscribe(WorldStatistics, f'/world/{args.world_name}/stats', self.on_stats)
        root = '/lab/practice' if args.mode == 'practice' else '/lab/homework'
        suffix = 'drive' if args.mode == 'practice' else 'actuator/velocity'
        self.gz.subscribe(GzTwist, root+'/'+suffix, self.on_gz_command)
        self.subscriptions_.append(self.create_subscription(Clock, '/clock', self.on_clock, 20))
        topics = [('scan', LaserScan, 'scan'), ('imu', Imu, 'imu'),
                  ('image', Image, 'camera/image_raw'), ('info', CameraInfo, 'camera/camera_info'),
                  ('odom', Odometry, 'odom'), ('ground_truth', PoseArray, 'ground_truth'),
                  ('joint_states', JointState, 'joint_states')]
        for name, typ, suffix in topics:
            self.subscriptions_.append(self.create_subscription(
                typ, f'/{args.namespace}/{suffix}',
                lambda msg, key=name: self.on_message(key, msg), qos_profile_sensor_data))
        self.command_pub = self.create_publisher(Twist, f'/{args.namespace}/cmd_vel', 10)

    def on_clock(self, msg):
        value = seconds(msg.clock)
        if self.clock is not None and value < self.clock-1e-9:
            self.clock_jumps.append({'from': self.clock, 'to': value, 'arrival': time.monotonic()})
        self.clock = value

    def on_stats(self, msg):
        self.stats = dict(sim_time=gzseconds(msg.sim_time), iterations=int(msg.iterations),
                          paused=bool(msg.paused), real_time_factor=msg.real_time_factor,
                          arrival=time.monotonic())
        self.stats_serial += 1

    def on_gz_command(self, msg):
        self.gz_command = (msg.linear.x, msg.angular.z, time.monotonic())

    def on_message(self, key, msg):
        item = {'stamp': seconds(msg.header.stamp), 'arrival': time.monotonic(),
                'frame': msg.header.frame_id, 'value': None}
        if key == 'scan':
            finite = [float(v) for v in msg.ranges if math.isfinite(v)]
            center = msg.ranges[len(msg.ranges)//2] if msg.ranges else math.nan
            item.update(count=len(msg.ranges), finite_count=len(finite),
                center=float(center) if math.isfinite(center) else None,
                min_range=min(finite) if finite else None, range_min=msg.range_min,
                range_max=msg.range_max, angle_min=msg.angle_min, angle_max=msg.angle_max,
                scan_time=msg.scan_time, time_increment=msg.time_increment)
        elif key == 'imu':
            item.update(value=msg.angular_velocity.z,
                acceleration=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
                angular_velocity=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                angular_covariance=list(msg.angular_velocity_covariance),
                acceleration_covariance=list(msg.linear_acceleration_covariance))
        elif key == 'image':
            sample = bytes(msg.data)
            item.update(width=msg.width, height=msg.height, encoding=msg.encoding, step=msg.step,
                        bytes=len(sample), pixel_min=min(sample) if sample else None,
                        pixel_max=max(sample) if sample else None)
        elif key == 'info':
            item.update(width=msg.width, height=msg.height, k=list(msg.k), p=list(msg.p))
        elif key == 'odom':
            p = msg.pose.pose
            item.update(x=p.position.x, y=p.position.y, z=p.position.z, yaw=yaw(p.orientation),
                        child_frame=msg.child_frame_id, linear_x=msg.twist.twist.linear.x,
                        angular_z=msg.twist.twist.angular.z)
        elif key == 'ground_truth':
            if msg.poses:
                p = msg.poses[0]
                item.update(x=p.position.x, y=p.position.y, z=p.position.z,
                            yaw=yaw(p.orientation), count=len(msg.poses))
        elif key == 'joint_states':
            item.update(names=list(msg.name), positions=list(msg.position))
        self.streams[key].append(item)
        if len(self.streams[key]) > 120000:
            del self.streams[key][:40000]

    def pump(self, wall_seconds=0.05):
        end = time.monotonic()+wall_seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0, end-time.monotonic())))

    def until(self, condition, label, timeout=None, tick=None):
        end = time.monotonic()+(timeout or self.args.timeout)
        while not condition():
            if time.monotonic() >= end:
                missing = [key for key in ('scan','imu','image','info','odom','ground_truth','joint_states')
                           if not self.streams[key]]
                raise TimeoutError(f'{label}: wall deadline; missing streams={missing}; stats={self.stats}')
            if tick:
                tick()
            self.pump(0.02)

    def ready(self):
        self.until(lambda: self.clock is not None and self.stats is not None and
                   all(self.streams[k] for k in ('scan','imu','image','info','odom','ground_truth','joint_states')),
                   'ready')

    def control(self, operation, steps=1000):
        request = WorldControl()
        request.pause = operation != 'play'
        if operation == 'step':
            request.multi_step = steps
        elif operation == 'reset':
            request.reset.all = True
        # gz's Python synchronous request can contend with Python subscription
        # callbacks for the GIL. A bounded CLI subprocess releases that GIL;
        # completion is still established by fresh stats below, never by ack.
        response = subprocess.run(['gz','service','-s',f'/world/{self.args.world_name}/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000',
            '--req',text_format.MessageToString(request)],capture_output=True,text=True,timeout=5)
        if response.returncode or 'data: true' not in response.stdout:
            raise RuntimeError(f'Gazebo rejected or timed out: {operation}: {response.stdout} {response.stderr}')

    def set_velocity(self, linear=0.0, angular=0.0, barrier=True):
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        issued = time.monotonic()
        self.command_pub.publish(message)
        if barrier:
            self.until(lambda: self.gz_command is not None and self.gz_command[2] >= issued and
                       abs(self.gz_command[0]-linear)<1e-9 and abs(self.gz_command[1]-angular)<1e-9,
                       'command bridge receipt', timeout=min(self.args.timeout,10),
                       tick=lambda: self.command_pub.publish(message))

    def advance(self, duration):
        start = self.clock
        self.until(lambda: self.clock is not None and self.clock >= start+duration, 'sim-time interval')

    def collect(self, duration):
        self.ready()
        self.set_velocity()
        self.control('play')
        self.advance(1.0)
        starts = {k: len(v) for k,v in self.streams.items()}
        begin = {'sim': self.clock, 'wall': time.monotonic()}
        self.advance(duration)
        end = {'sim': self.clock, 'wall': time.monotonic()}
        samples = {k:v[starts.get(k,0):] for k,v in self.streams.items()}
        return samples, {'begin': begin, 'end': end,
            'measured_rtf': (end['sim']-begin['sim'])/(end['wall']-begin['wall'])}

    def pose(self, key='ground_truth'):
        return {k:self.streams[key][-1][k] for k in ('x','y','yaw')}

    def scenario(self):
        self.ready()
        self.set_velocity()
        self.control('play')
        self.advance(0.5)
        before = self.pose()
        odom_before = self.pose('odom')
        self.set_velocity(0.2, 0)
        self.advance(2.0)
        self.set_velocity()
        self.advance(0.2)
        stopped_before = self.pose()
        self.advance(0.5)
        straight = self.pose()
        odom_straight = self.pose('odom')
        self.set_velocity(0, 0.4)
        self.advance(1.5)
        self.set_velocity()
        self.advance(0.5)
        turn = self.pose()
        self.control('pause')
        self.until(lambda: self.stats['paused'], 'pause acknowledgement')
        paused = self.stats.copy()
        self.pump(0.5)
        after_pause = self.stats.copy()
        target = after_pause['iterations']+self.args.steps
        self.control('step', self.args.steps)
        self.until(lambda: self.stats['iterations'] >= target and self.stats['paused'], 'step completion')
        stepped = self.stats.copy()
        self.control('reset')
        self.until(lambda: self.stats['sim_time'] < stepped['sim_time'] and self.stats['iterations'] == 0,
                   'reset completion')
        reset = self.stats.copy()
        self.pump(0.3)
        return dict(before=before, odom_before=odom_before, straight=straight, stopped_before=stopped_before,
                    odom_straight=odom_straight, turn=turn, paused=paused, after_pause=after_pause,
                    stepped=stepped, reset=reset, clock_jumps=list(self.clock_jumps),
                    requested_steps=self.args.steps)


def practice_metrics(records):
    """Display a deliberately single-epoch, already stationary practice window."""
    if len(records)<2:
        return {'samples':len(records),'sim_hz':None,'wall_hz':None,'gyro_mean':None,'gyro_std':None}
    gyro = [r['value'] for r in records if r['value'] is not None and math.isfinite(r['value'])]
    ds, dw = records[-1]['stamp']-records[0]['stamp'], records[-1]['arrival']-records[0]['arrival']
    return {'samples':len(records), 'sim_hz':(len(records)-1)/ds if ds>0 else None,
            'wall_hz':(len(records)-1)/dw if dw>0 else None,
            'gyro_mean':statistics.mean(gyro) if gyro else None,
            'gyro_std':statistics.pstdev(gyro) if gyro else None}


def evaluate(observer, samples, phases=None):
    checks = []
    def add(name, category, ok, detail):
        checks.append(dict(name=name, category=category, ok=bool(ok), detail=detail))
    p = observer.args.frame_prefix
    clock_publishers = observer.get_publishers_info_by_topic('/clock')
    add('clock_authority','frames_time',len(clock_publishers)==1,
        {'publisher_count':len(clock_publishers)})
    expected = {'scan': p+'lidar_frame', 'imu':p+'imu_frame',
                'image':p+'camera_optical_frame','info':p+'camera_optical_frame','odom':p+'odom'}
    for key, frame in expected.items():
        rows = samples.get(key,[])
        add(key+'_frames', 'frames_time', bool(rows) and all(r['frame']==frame for r in rows),
            {'expected':frame, 'observed':sorted(set(r['frame'] for r in rows))})
    for key in ('scan','imu','image','info','odom','joint_states','ground_truth'):
        rows = samples.get(key,[])
        add(key+'_samples', 'sensors', len(rows)>=2, {'count':len(rows)})
        add(key+'_stamps','frames_time', len(rows)>=2 and all(b['stamp']>=a['stamp']>0 for a,b in zip(rows,rows[1:])),
            'nondecreasing nonzero stamps within the observation epoch')
    for key, target in [('scan',10),('imu',100),('image',10),('odom',50)]:
        rows = samples.get(key,[])
        metric = practice_metrics(rows)
        f = metric['sim_hz']
        add(key+'_rate','sensors', f is not None and 0.85*target <= f <= 1.15*target,
            {'sim_hz':f,'target':target,'relative_tolerance':0.15})
    if samples.get('scan'):
        last=samples['scan'][-1]
        expected_center = 2.82 if observer.args.mode == 'practice' else 2.22
        add('lidar_content','sensors', last['count']==180 and last['finite_count']>30 and
            last['center'] is not None and abs(last['center']-expected_center)<0.15,
            {'last':last, 'stationary_center_expected':expected_center,'tolerance_m':0.15})
    if samples.get('image') and samples.get('info'):
        im,info=samples['image'][-1],samples['info'][-1]
        add('camera_content','sensors',im['width']==info['width']==160 and im['height']==info['height']==120
            and im['encoding']=='rgb8' and im['bytes']==160*120*3 and im['pixel_max']>im['pixel_min']
            and info['k'][0]>0 and info['k'][4]>0, {'image':im,'info':info})
    if samples.get('imu'):
        az=statistics.mean(r['acceleration'][2] for r in samples['imu'])
        add('imu_gravity','sensors', abs(az-9.81)<0.25, {'mean_z':az,'expected':9.81,'tolerance':0.25})
        gyro=practice_metrics(samples['imu'])
        target_mean=0.08 if observer.args.profile=='biased' else 0.0
        target_std=0.0 if observer.args.profile=='ideal' else 0.02
        tolerance=0.001 if observer.args.profile=='ideal' else 0.008
        add('gyro_statistics','sensors',abs(gyro['gyro_mean']-target_mean)<tolerance and
            abs(gyro['gyro_std']-target_std)<tolerance,
            {'mean':gyro['gyro_mean'],'std':gyro['gyro_std'],'target_mean':target_mean,
             'target_std':target_std,'tolerance':tolerance})
    if samples.get('joint_states'):
        add('wheel_joints','frames_time',{'left_wheel_joint','right_wheel_joint'} <= set(samples['joint_states'][-1]['names']),samples['joint_states'][-1]['names'])
    # Verify transforms at measurement times, before resetting the simulation.
    for key in ('scan','imu','image'):
        rows=samples.get(key,[])
        ok,detail=False,'no samples'
        if rows:
            # Leave delivery time for the dynamic odom transform to reach tf2.
            row=rows[-2] if len(rows)>1 else rows[-1]
            try:
                observer.buffer.lookup_transform(p+'odom',row['frame'],Time(seconds=row['stamp']))
                ok,detail=True,'lookup at measurement stamp succeeded'
            except TransformException as error:
                detail=str(error)
        add(key+'_tf','frames_time',ok,detail)
    if samples.get('odom'):
        od=samples['odom'][max(0,len(samples['odom'])-3)]
        ok,detail=False,'no transform'
        try:
            tr=observer.buffer.lookup_transform(p+'odom',p+'base_link',Time(seconds=od['stamp']))
            error=math.hypot(tr.transform.translation.x-od['x'],tr.transform.translation.y-od['y'])
            ok=error<0.02 and od['child_frame']==p+'base_link'
            detail={'xy_error':error,'child_frame':od['child_frame'],'tolerance_m':0.02}
        except TransformException as error:
            detail=str(error)
        add('odom_tf_consistency','frames_time',ok,detail)
    if phases:
        b,s,t=phases['before'],phases['straight'],phases['turn']
        dx,dy=s['x']-b['x'],s['y']-b['y']
        forward=math.cos(b['yaw'])*dx+math.sin(b['yaw'])*dy
        turn_delta=angle_delta(t['yaw'],s['yaw'])
        ob,os=phases['odom_before'],phases['odom_straight']
        add('straight_motion','motion',0.28<forward<0.55,{'world_forward_m':forward,'expected_interval':[0.28,0.55]})
        add('odometry_motion','motion',0.28<math.hypot(os['x']-ob['x'],os['y']-ob['y'])<0.55,{'before':ob,'after':os})
        add('turn_motion','motion',0.4<turn_delta<0.8,{'world_yaw_delta':turn_delta,'expected_interval':[0.4,0.8]})
        stop=phases['stopped_before']
        drift=math.hypot(s['x']-stop['x'],s['y']-stop['y'])
        add('stop_motion','motion',drift<0.02,{'world_drift_m':drift,'max_m':0.02})
        a,z,n=phases['paused'],phases['after_pause'],phases['stepped']
        add('pause','semantics',a['sim_time']==z['sim_time'] and a['iterations']==z['iterations'],{'before':a,'after':z})
        add('step','semantics',n['iterations']-z['iterations']==phases['requested_steps'] and
            abs(n['sim_time']-z['sim_time']-0.001*phases['requested_steps'])<1e-8,
            {'before':z,'after':n,'expected_dt':0.001*phases['requested_steps']})
        add('reset','semantics',phases['reset']['iterations']==0 and phases['reset']['sim_time']<1e-9,
            {'reset':phases['reset'],'clock_jumps':phases['clock_jumps']})
    return checks
