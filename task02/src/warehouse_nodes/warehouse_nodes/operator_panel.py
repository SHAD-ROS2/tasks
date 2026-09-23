from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from warehouse_interfaces.msg import SafetyState

from .qos import panel_status_qos


class OperatorPanel(Node):
    def __init__(self) -> None:
        super().__init__("operator_panel")
        self._subscription = self.create_subscription(
            SafetyState,
            "/safety/state",
            self._on_safety_state,
            panel_status_qos(),
        )

    def _on_safety_state(self, message: SafetyState) -> None:
        own_namespace = self.get_namespace()
        if message.source_namespace != own_namespace:
            self.get_logger().warning(
                "CROSS_TALK "
                f"expected={own_namespace} received={message.source_namespace}"
            )
            return
        self.get_logger().info(
            "PANEL_READY "
            f"namespace={own_namespace} stopped={message.stopped} sequence={message.sequence}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OperatorPanel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
