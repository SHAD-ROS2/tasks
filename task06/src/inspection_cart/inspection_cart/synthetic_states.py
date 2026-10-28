"""Synthetic kinematic input, never a measured state or a controller."""
import math


JOINT_NAMES = ["left_wheel_joint", "right_wheel_joint", "turret_joint"]


def positions_at_tick(tick):
    """Exactly repeatable sample values: 20 Hz, 10 s sweep period, ±1.2 rad."""
    return [0.0, 0.0, 1.2 * math.sin(2.0 * math.pi * tick / 200.0)]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class SyntheticStates(Node):
        def __init__(self):
            super().__init__("synthetic_joint_states")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("synthetic source requires wall ROS time; replay uses the bag")
            self.publisher = self.create_publisher(JointState, "/joint_states", 10)
            self.tick = 0
            self.timer = self.create_timer(0.05, self.publish_state)
            self.get_logger().info(
                "event=source_started synthetic=true rate_hz=20 period_s=10 amplitude_rad=1.2"
            )

        def publish_state(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINT_NAMES)
            msg.position = positions_at_tick(self.tick)
            self.publisher.publish(msg)
            if self.tick % 100 == 0:
                self.get_logger().info(
                    f"event=sample synthetic=true tick={self.tick} turret_rad={msg.position[2]:.6f}"
                )
            self.tick += 1

    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
