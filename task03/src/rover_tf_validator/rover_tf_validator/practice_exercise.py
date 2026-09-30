"""Practice checkpoint: choose the query time, then compare your result with the demo."""
import rclpy
from rclpy.time import Time
from .practice_lookup import PracticeLookup


def choose_time(msg, mode):
    # TODO (two lines): use msg.header.stamp for measurement mode;
    # use TF2's special latest query only when mode == 'latest'.
    # Before editing, predict which mode should recover the stationary landmark.
    raise NotImplementedError('Practice checkpoint: complete choose_time in practice_exercise.py')


def main(args=None):
    rclpy.init(args=args)
    node = PracticeLookup(choose_time=choose_time)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
