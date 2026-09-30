"""Public headless observer. The geometric oracle is independent of the URDF.

Start the homework launch first, then run this process in the same namespace.
A successful TF lookup is reported separately from geometric agreement.
"""
import argparse
from collections import Counter, defaultdict, deque
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException

LANDMARKS = {'lidar': (3., 0., .36), 'camera': (4., .4, .4), 'imu': (1., 2., .6)}
SOURCES = {'lidar': 'lidar_link', 'camera': 'camera_optical_frame', 'imu': 'imu_link'}
EDGES = {'odom': 'map', 'base_link': 'odom', 'lidar_link': 'base_link',
         'camera_link': 'base_link', 'camera_optical_frame': 'camera_link',
         'imu_link': 'base_link'}


class Acceptance(Node):
    def __init__(self, prefix, warmup):
        super().__init__('rover_acceptance')
        self.prefix = prefix
        self.ready_at = time.monotonic() + warmup
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.statuses = Counter()
        self.samples = Counter()
        self.residuals = defaultdict(list)
        self.failures = set()
        self.odom_queue = deque(maxlen=100)
        self.odom_matches = 0
        self.edges = defaultdict(set)
        self.publisher_graph = {}
        self.observations = {}
        self.unmatched_reports = []
        self.report_sub = self.create_subscription(String, 'validation/report', self.receive_report, 100)
        self.observation_subs = [self.create_subscription(PointStamped, f'observation/{sensor}',
            lambda msg, sensor=sensor: self.receive_observation(sensor, msg), 100)
            for sensor in LANDMARKS]
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.receive_odom, 50)
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.receive_tf, 100)
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_sub = self.create_subscription(TFMessage, '/tf_static', self.receive_tf, qos)

    def receive_observation(self, sensor, msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        self.observations[(sensor, stamp)] = msg.header.frame_id
        # Bounded recent-history evidence, well above fixture rate and wait deadline.
        while len(self.observations) > 1500:
            del self.observations[next(iter(self.observations))]

    def receive_tf(self, msg, info):
        for tf in msg.transforms:
            if tf.child_frame_id.startswith(self.prefix):
                self.edges[tf.child_frame_id].add(tf.header.frame_id)

    def receive_report(self, msg):
        if time.monotonic() < self.ready_at:
            return
        try:
            result = json.loads(msg.data)
            if result.get('kind') != 'observation':
                if result.get('status') in ('INVALID_INPUT', 'NOT_IMPLEMENTED'):
                    self.failures.add('raw TF audit: ' + result.get('status', 'missing status'))
                return
            sensor = result.get('sensor')
            if sensor not in LANDMARKS:
                self.failures.add('report sensor is outside the contract')
                return
            status = result.get('status', 'MISSING_STATUS')
            self.statuses[status] += 1
            if status != 'OK':
                self.failures.add('validator status: ' + status)
                return
            self.samples[sensor] += 1
            if result.get('target_frame') != self.prefix+'odom':
                self.failures.add('report target_frame differs from prefixed odom')
            if result.get('source_frame') != self.prefix+SOURCES[sensor]:
                self.failures.add(sensor + ': source frame differs from sensor contract')
            stamp = result.get('stamp_ns')
            if not isinstance(stamp, int) or stamp <= 0:
                self.failures.add(sensor + ': missing original positive stamp')
            elif (sensor, stamp) not in self.observations:
                self.unmatched_reports.append((sensor, stamp))
            point = result.get('point')
            if not isinstance(point, list) or len(point) != 3 or not all(math.isfinite(v) for v in point):
                self.failures.add(sensor + ': non-finite or missing transformed point')
                return
            residual = math.dist(point, LANDMARKS[sensor])
            self.residuals[sensor].append(residual)
            if residual > .005:
                self.failures.add(sensor + ': landmark residual exceeds 0.005 m')
        except (ValueError, TypeError, KeyError):
            self.failures.add('report JSON or schema is invalid')

    def receive_odom(self, msg):
        if time.monotonic() < self.ready_at:
            return
        if msg.header.frame_id != self.prefix+'odom' or msg.child_frame_id != self.prefix+'base_link':
            self.failures.add('odometry parent/child frame contract')
        self.odom_queue.append(msg)

    def check_pending_odom(self):
        for _ in range(len(self.odom_queue)):
            msg = self.odom_queue.popleft()
            try:
                tf = self.buffer.lookup_transform(self.prefix+'odom', self.prefix+'base_link',
                                                  Time.from_msg(msg.header.stamp))
            except TransformException:
                self.odom_queue.append(msg)
                continue
            t, p = tf.transform.translation, msg.pose.pose.position
            q, o = tf.transform.rotation, msg.pose.pose.orientation
            xyz_error = math.dist((t.x,t.y,t.z), (p.x,p.y,p.z))
            # q and -q describe the same rotation.
            a, b = (q.x,q.y,q.z,q.w), (o.x,o.y,o.z,o.w)
            q_error = min(math.dist(a,b), math.dist(a,[-v for v in b]))
            if xyz_error > 1e-5 or q_error > 1e-5:
                self.failures.add('odometry pose differs from TF at the same stamp')
            self.odom_matches += 1

    def finish(self):
        for sensor, stamp in self.unmatched_reports:
            if (sensor, stamp) not in self.observations:
                self.failures.add(sensor + ': report stamp does not match received measurement')
        for sensor in LANDMARKS:
            if self.samples[sensor] < 10:
                self.failures.add(sensor + ': fewer than 10 successful observations')
        if self.odom_matches < 10:
            self.failures.add('fewer than 10 odometry/TF comparisons')
        for child, parent in EDGES.items():
            if self.edges[self.prefix+child] != {self.prefix+parent}:
                self.failures.add(child + ': missing or conflicting parent')
        expected = Counter({(self.get_namespace(), 'fixture'): 1,
                            (self.get_namespace(), 'robot_state_publisher'): 1})
        for topic in ('/tf', '/tf_static'):
            publishers = self.get_publishers_info_by_topic(topic)
            observed = Counter((p.node_namespace, p.node_name) for p in publishers)
            self.publisher_graph[topic] = [p.node_namespace+'/'+p.node_name for p in publishers]
            if observed != expected:
                self.failures.add(topic + ': expected one fixture and one robot_state_publisher broadcaster')
        return {'passed': not self.failures, 'failures': sorted(self.failures),
                'transport_statuses': dict(self.statuses),
                'successful_samples': dict(self.samples),
                'geometry_max_error_m': {k: max(v) for k,v in self.residuals.items() if v},
                'odom_comparisons': self.odom_matches, 'frame_prefix': self.prefix,
                'tolerance_m': .005, 'tf_publishers': self.publisher_graph,
                'ownership_scope': 'Expected publisher graph and unique edge parents; per-message publisher attribution is unavailable in Jazzy Python metadata.'}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=6.)
    parser.add_argument('--warmup', type=float, default=1.)
    parser.add_argument('--frame-prefix', default='rover/')
    parser.add_argument('--output', type=Path)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.duration <= parsed.warmup + .5:
        parser.error('duration must exceed warmup by at least 0.5 seconds')
    rclpy.init(args=ros_args)
    node = Acceptance(parsed.frame_prefix, parsed.warmup)
    deadline = time.monotonic() + parsed.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            node.check_pending_odom()
        result = node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    payload = json.dumps(result, indent=2, sort_keys=True)
    print(payload)
    if parsed.output:
        parsed.output.parent.mkdir(parents=True, exist_ok=True)
        parsed.output.write_text(payload+'\n')
    raise SystemExit(0 if result['passed'] else 1)
