from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import ObstacleState, RangeReading

from .core import RangeSample, detect_obstacle


class ObstacleFilter(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_filter")
        self.declare_parameter("stop_distance_m", 0.60)

        self._stop_distance_m = float(self.get_parameter("stop_distance_m").value)
        self._publisher = self.create_publisher(
            ObstacleState,
            "perception/obstacle",
            10,
        )
        self._subscription = self.create_subscription(
            RangeReading,
            "range/readings",
            self._on_reading,
            10,
        )

    def _on_reading(self, message: RangeReading) -> None:
        decision = detect_obstacle(
            RangeSample(sequence=message.sequence, distance_m=message.distance_m),
            self._stop_distance_m,
        )
        output = ObstacleState()
        output.sequence = decision.sequence
        output.blocked = decision.blocked
        output.distance_m = decision.distance_m
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObstacleFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
