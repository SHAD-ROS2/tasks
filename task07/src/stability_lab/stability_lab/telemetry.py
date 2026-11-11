"""Bridge simulator ground truth to the floating root TF and publish run config."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class StateBridge(Node):
    def __init__(self):
        super().__init__('state_bridge')
        self.cfg = json.loads(self.declare_parameter('config_json','{}').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, '/g1/lab_config', qos)
        self.pub.publish(String(data=json.dumps(self.cfg)))
        self.tf = TransformBroadcaster(self)
        if self.cfg['base'] == 'free':
            self.create_subscription(Odometry, '/g1/odometry', self.on_odom, 10)
        self.create_timer(1.0, lambda: self.pub.publish(String(data=json.dumps(self.cfg))))

    def on_odom(self, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'g1/pelvis'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)


def main():
    rclpy.init()
    node = StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
