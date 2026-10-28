#!/usr/bin/env python3
"""Bounded installed-package smoke test in a sourced ROS Jazzy shell.

Uses its own launch and tears down only its process group. Run in an otherwise
empty ROS domain. This tests the actual published TF, not the pure FK oracle.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    Path("evidence").mkdir(exist_ok=True)
    rclpy.init()
    node = Node("l06_installed_package_check")
    states = []
    subscription = node.create_subscription(JointState, "/joint_states", states.append, 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    with open("evidence/ros-smoke-launch.log", "w") as log:
        launch = subprocess.Popen([
            "ros2", "launch", "inspection_cart", "view.launch.py",
            "rviz:=false", "state_source:=synthetic", "model:=clean",
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12.0
            transform = None
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError("launch exited; see evidence/ros-smoke-launch.log")
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    transform = buffer.lookup_transform("base_link", "probe_tip", Time())
                except TransformException:
                    continue
                if len(states) >= 30:
                    break
            assert len(states) >= 30, f"only {len(states)} states in 12 wall seconds"
            expected_names = ["left_wheel_joint", "right_wheel_joint", "turret_joint"]
            assert all(message.name == expected_names for message in states)
            stamps = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in states]
            assert all(later > earlier for earlier, later in zip(stamps, stamps[1:]))
            assert len({round(m.position[2], 3) for m in states}) > 5
            assert transform is not None
            point = transform.transform.translation
            assert abs(point.z - 0.34) < 1e-6, f"wrong tip height: {point.z}"
            assert abs(math.hypot(point.x - 0.07, point.y) - 0.30) < 1e-6
            print(f"PASS: installed launch, {len(states)} changing states, increasing stamps, live TF arc")
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    launch.wait(timeout=4)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
