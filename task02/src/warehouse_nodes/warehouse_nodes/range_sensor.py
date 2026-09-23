from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import RangeReading

from .core import CyclicReadings
from .qos import sensor_data_qos


class RangeSensor(Node):
    def __init__(self) -> None:
        super().__init__("range_sensor")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("readings_m", [2.0, 1.2, 0.45, 0.30, 1.5])

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        readings_m = self.get_parameter("readings_m").value
        self._program = CyclicReadings(readings_m)
        self._publisher = self.create_publisher(
            RangeReading,
            "range/readings",
            sensor_data_qos(),
        )
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_reading)
        self.get_logger().info(f"publishing deterministic ranges at {publish_rate_hz:.2f} Hz")

    def _publish_reading(self) -> None:
        sample = self._program.next()
        message = RangeReading()
        message.sequence = sample.sequence
        message.distance_m = sample.distance_m
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RangeSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
