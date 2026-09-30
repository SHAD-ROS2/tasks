"""Practice example: one point, one lookup. Deliberately small, no homework policy."""
import json
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .contracts import RigidTransform
from .geometry import apply_transform


class PracticeLookup(Node):
    def __init__(self, choose_time=None):
        super().__init__('practice_lookup')
        self.declare_parameter('target_frame', 'rover/world')
        self.declare_parameter('mode', 'measurement')
        self.choose_time = choose_time
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(PointStamped, 'probe', self.receive, 10)

    def receive(self, msg):
        mode = self.get_parameter('mode').value
        target = self.get_parameter('target_frame').value
        # Exercise: predict the outcome before toggling this parameter.
        try:
            requested_time = (self.choose_time(msg, mode) if self.choose_time else
                              (Time() if mode == 'latest' else Time.from_msg(msg.header.stamp)))
        except NotImplementedError as error:
            self.get_logger().warning(str(error))
            return
        try:
            tf = self.buffer.lookup_transform(target, msg.header.frame_id, requested_time)
        except TransformException as error:
            self.get_logger().warning(str(error))
            return
        t, q = tf.transform.translation, tf.transform.rotation
        result = apply_transform(RigidTransform((t.x,t.y,t.z), (q.x,q.y,q.z,q.w)),
                                 (msg.point.x, msg.point.y, msg.point.z))
        self.get_logger().info(json.dumps({'mode': mode, 'point_world': result,
            'measurement_sec': msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            'expected_world': [2.0, .5, .4]}))


def main(args=None):
    rclpy.init(args=args)
    node = PracticeLookup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
