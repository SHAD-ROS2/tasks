from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import RangeReading, SafetyState

from .core import must_stop
from .qos import safety_status_qos, sensor_data_qos


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("stop_distance_m", 0.50)

        self._stop_distance_m = float(self.get_parameter("stop_distance_m").value)
        self._last_state: bool | None = None
        self._publisher = self.create_publisher(
            SafetyState,
            "/safety/state",
            safety_status_qos(),
        )
        self._subscription = self.create_subscription(
            RangeReading,
            "range/readings",
            self._on_reading,
            sensor_data_qos(),
        )

    def _on_reading(self, message: RangeReading) -> None:
        stopped = must_stop(message.distance_m, self._stop_distance_m)
        if stopped == self._last_state:
            return

        self._last_state = stopped
        output = SafetyState()
        output.sequence = message.sequence
        output.stopped = stopped
        output.source_namespace = self.get_namespace()
        self._publisher.publish(output)
        self.get_logger().info(
            f"safety stopped={stopped} source={output.source_namespace}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
