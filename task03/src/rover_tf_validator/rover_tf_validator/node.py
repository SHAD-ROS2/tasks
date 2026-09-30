"""Supplied ROS transport. Student edits belong in validator.py."""
import json
import math

import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException

from .contracts import Observation, RigidTransform, LookupFailure
from .queue import RetryQueue
from . import validator


class AuditedBuffer(Buffer):
    """Observe malformed TF before tf2 rejects it and hides the original cause."""
    def __init__(self, node, emit):
        super().__init__(cache_time=Duration(seconds=10), node=node)
        self.emit = emit
        self.audit_todo_reported = False

    def audit(self, transform, is_static):
        q = transform.transform.rotation
        t = transform.transform.translation
        valid = validator.quaternion_is_valid((q.x, q.y, q.z, q.w))
        finite_translation = all(math.isfinite(v) for v in (t.x, t.y, t.z))
        if valid is None:
            if not self.audit_todo_reported:
                self.emit({'kind': 'tf_audit', 'status': 'NOT_IMPLEMENTED',
                           'detail': 'Complete quaternion_is_valid in validator.py'})
                self.audit_todo_reported = True
            return True  # Keep starter demonstrable; this never constitutes a passing audit.
        if not valid or not finite_translation:
            stamp = transform.header.stamp
            self.emit({'kind': 'tf_audit', 'status': 'INVALID_INPUT',
                       'source_frame': transform.child_frame_id,
                       'target_frame': transform.header.frame_id,
                       'stamp_ns': stamp.sec*1_000_000_000+stamp.nanosec,
                       'static': is_static, 'rotation': [q.x, q.y, q.z, q.w],
                       'detail': 'raw TF contains a non-unit quaternion or non-finite translation'})
            return False
        return True

    def set_transform(self, transform, authority):
        if self.audit(transform, False):
            super().set_transform(transform, authority)

    def set_transform_static(self, transform, authority):
        if self.audit(transform, True):
            super().set_transform_static(transform, authority)


class ValidatorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('rover_tf_validator', **kwargs)
        self.declare_parameter('frame_prefix', 'rover/')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('wait_timeout_sec', .50)
        self.declare_parameter('queue_limit', 128)
        prefix = self.get_parameter('frame_prefix').value
        self.target = prefix + self.get_parameter('target_frame').value
        self.publisher = self.create_publisher(String, 'validation/report', 100)
        self.buffer = AuditedBuffer(self, self.emit)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.queue = RetryQueue(validator.validate_observation, self.lookup, self.emit,
                                timeout=self.get_parameter('wait_timeout_sec').value,
                                capacity=self.get_parameter('queue_limit').value)
        self.subscriptions_ = [self.create_subscription(
            PointStamped, f'observation/{sensor}',
            lambda msg, sensor=sensor: self.receive(sensor, msg), 20)
            for sensor in ('lidar', 'camera', 'imu')]
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.retry_timer = self.create_timer(.02, self.queue.tick, clock=self.steady_clock)
        self.jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
            post_callback=lambda jump: self.queue.clear())

    def emit(self, result):
        msg = String()
        # JSON has no NaN/Infinity; malformed raw values retain their diagnostics.
        def safe(value):
            if isinstance(value, float) and not math.isfinite(value):
                return str(value)
            if isinstance(value, dict):
                return {key: safe(item) for key, item in value.items()}
            if isinstance(value, (tuple, list)):
                return [safe(item) for item in value]
            return value
        msg.data = json.dumps(safe(result), sort_keys=True, allow_nan=False)
        self.publisher.publish(msg)

    def receive(self, sensor, msg):
        stamp = msg.header.stamp
        observation = Observation(sensor, msg.header.frame_id, self.target,
                                  stamp.sec*1_000_000_000+stamp.nanosec,
                                  (msg.point.x, msg.point.y, msg.point.z))
        self.queue.submit(observation)

    def lookup(self, target, source, stamp_ns):
        try:
            # No positive timeout here: callbacks must leave the executor free.
            transform = self.buffer.lookup_transform(
                target, source, Time(nanoseconds=stamp_ns, clock_type=ClockType.ROS_TIME))
        except ConnectivityException as error:
            raise LookupFailure('DISCONNECTED', str(error)) from error
        except ExtrapolationException as error:
            raise LookupFailure('TIME_UNAVAILABLE', str(error)) from error
        except LookupException as error:
            raise LookupFailure('UNKNOWN_FRAME', str(error)) from error
        except TransformException as error:
            raise LookupFailure('TIME_UNAVAILABLE', str(error)) from error
        t, q = transform.transform.translation, transform.transform.rotation
        return RigidTransform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def main(args=None):
    rclpy.init(args=args)
    node = ValidatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
