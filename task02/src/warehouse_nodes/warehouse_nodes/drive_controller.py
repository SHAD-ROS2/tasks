from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import MotionCommand

from .core import FORWARD, limit_speed


class DriveController(Node):
    def __init__(self) -> None:
        super().__init__("drive_controller")
        self.declare_parameter("max_speed_mps", 0.50)

        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self._subscription = self.create_subscription(
            MotionCommand,
            "control/motion",
            self._on_motion,
            10,
        )

    def _on_motion(self, message: MotionCommand) -> None:
        output = Twist()
        if message.mode == FORWARD:
            output.linear.x = limit_speed(message.speed_mps, self._max_speed_mps)
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
