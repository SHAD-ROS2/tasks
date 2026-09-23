from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import MotionCommand, ObstacleState

from .core import ObstacleDecision, plan_motion


class MissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner")
        self.declare_parameter("cruise_speed_mps", 0.35)

        self._cruise_speed_mps = float(self.get_parameter("cruise_speed_mps").value)
        self._publisher = self.create_publisher(
            MotionCommand,
            "control/motion",
            10,
        )
        self._subscription = self.create_subscription(
            ObstacleState,
            "perception/obstacle",
            self._on_obstacle,
            10,
        )

    def _on_obstacle(self, message: ObstacleState) -> None:
        decision = plan_motion(
            ObstacleDecision(
                sequence=message.sequence,
                blocked=message.blocked,
                distance_m=message.distance_m,
            ),
            self._cruise_speed_mps,
        )
        output = MotionCommand()
        output.sequence = decision.sequence
        output.mode = decision.mode
        output.speed_mps = decision.speed_mps
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
