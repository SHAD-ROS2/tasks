#!/usr/bin/env python3
"""Optional bounded instructor check: record inputs, replay clean and broken TF.

Requires a sourced ROS Jazzy install and an otherwise empty ROS domain. Creates
a new bag directory in evidence and refuses to overwrite an existing one.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=7)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)


def wait_ready(node, process, condition, description, timeout=15):
    """Observe actual readiness, bounded by wall time rather than ROS clock."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"process exited while waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return
    raise TimeoutError(f"not ready in {timeout} wall seconds: {description}")


def record_inputs(start, bag):
    node = Node("l06_live_readiness")
    states = []
    sub = node.create_subscription(JointState, "/joint_states", states.append, 10)
    live = start(["ros2", "launch", "inspection_cart", "view.launch.py",
                  "model:=broken", "state_source:=synthetic", "rviz:=false"])
    recorder = None
    try:
        wait_ready(node, live, lambda: len(states) >= 3, "three live JointState samples")
        recorder = start(["ros2", "bag", "record", "--storage", "sqlite3",
                          "-o", str(bag), "--topics", "/joint_states"])

        def recording_ready():
            subscriptions = node.get_subscriptions_info_by_topic("/joint_states")
            return (bool(list(bag.glob("*.db3")))
                    and any(info.node_name == "rosbag2_recorder" for info in subscriptions))

        wait_ready(node, recorder, recording_ready, "recorder storage and JointState subscription")
        # Explicit observation window; this is not a startup-readiness delay.
        observe_until = time.monotonic() + 4.0
        while time.monotonic() < observe_until:
            if live.poll() is not None or recorder.poll() is not None:
                raise RuntimeError("source or recorder exited during the observation window")
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if recorder is not None:
            stop(recorder)
        stop(live)
        node.destroy_subscription(sub)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag-dir", type=Path, default=Path("evidence/verified-axis-input"))
    args = parser.parse_args()
    evidence = Path("evidence")
    evidence.mkdir(exist_ok=True)
    bag = args.bag_dir
    if bag.exists():
        raise FileExistsError(f"choose a fresh evidence directory; refusing to overwrite {bag}")
    with open(evidence / "bag-roundtrip.log", "w") as log:
        def start(arguments):
            return subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)

        rclpy.init()
        try:
            record_inputs(start, bag)
            subprocess.run(["ros2", "bag", "info", str(bag)], check=True)
        except BaseException:
            rclpy.shutdown()
            raise
        try:
            for variant in ("clean", "broken"):
                node = Node("l06_bag_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
                states = []
                sub = node.create_subscription(JointState, "/joint_states", states.append, 10)
                buffer = Buffer()
                listener = TransformListener(buffer, node)
                launch = start(["ros2", "launch", "inspection_cart", "view.launch.py",
                                f"model:={variant}", "state_source:=none",
                                "use_sim_time:=true", "rviz:=false"])
                player = None
                heights, authorities = [], []
                try:
                    def rsp_ready():
                        subscriptions = node.get_subscriptions_info_by_topic("/joint_states")
                        return (any(info.node_name == "robot_state_publisher" for info in subscriptions)
                                and buffer.can_transform("base_link", "rear_support", Time()))

                    wait_ready(node, launch, rsp_ready, "RSP JointState subscription and static TF")
                    player = start(["ros2", "bag", "play", str(bag), "--clock",
                                    "--disable-keyboard-controls"])
                    deadline = time.monotonic() + 18
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        try:
                            transform = buffer.lookup_transform("base_link", "probe_tip", Time())
                            heights.append(transform.transform.translation.z)
                        except TransformException:
                            pass
                        if len(states) > 10 and player.poll() is None:
                            authorities.append((len(node.get_publishers_info_by_topic("/joint_states")),
                                                len(node.get_publishers_info_by_topic("/clock"))))
                        if player.poll() is not None:
                            break
                    assert player.poll() == 0, "bag playback did not complete successfully"
                    assert len(states) >= 20, f"replayed only {len(states)} states"
                    assert len(heights) >= 10, "TF was not reconstructed"
                    # Publishers disappear during normal player teardown, before
                    # its CLI process exits. Zero endpoints then is not conflict.
                    assert (1, 1) in authorities, "never observed the active bag authorities"
                    assert all(a <= 1 and b <= 1 for a, b in authorities), set(authorities)
                    deviation = max(abs(height - 0.34) for height in heights)
                    if variant == "clean":
                        assert deviation < 1e-6, deviation
                    else:
                        assert deviation > 0.005, deviation
                    print(f"PASS replay {variant}: states={len(states)}, "
                          f"max_height_error_m={deviation:.6f}, one state/clock publisher")
                finally:
                    if player is not None:
                        stop(player)
                    stop(launch)
                    listener.unregister()
                    node.destroy_subscription(sub)
                    node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
