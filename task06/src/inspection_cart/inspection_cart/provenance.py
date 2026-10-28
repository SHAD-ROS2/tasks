"""Record the exact model bytes alongside a bag; no ROS import is required."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-root", type=Path, default=Path.cwd())
    args = parser.parse_args()
    model = args.model.resolve()
    commit = subprocess.run(
        ["git", "-C", str(args.source_root), "rev-parse", "HEAD"],
        text=True, capture_output=True, check=False,
    )
    status = subprocess.run(
        ["git", "-C", str(args.source_root), "status", "--porcelain"],
        text=True, capture_output=True, check=False,
    )
    result = {
        "schema": "l06-model-provenance-v1",
        "model_file": model.name,
        "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "source_commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "source_dirty": bool(status.stdout.strip()) if status.returncode == 0 else None,
        "state_origin": "synthetic_joint_states; not physics or hardware measurements",
        "recorded_topics": ["/joint_states"],
        "live_time": "system ROS time",
        "replay_time": "rosbag2 --clock; all consumers use_sim_time=true",
        "tf_policy": "reconstructed by exactly one robot_state_publisher; TF is not recorded",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
