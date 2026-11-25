#!/usr/bin/env bash
set -eo pipefail

# Serialize incremental builds if a second terminal runs the lab.
exec 9>/work/.task08.lock
flock 9

source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --base-paths /lab/src --executor sequential \
  --packages-select nav2_map_server nav2_map_io_lab \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source /work/install/local_setup.bash

if [[ "$1" == prepare ]]; then
  exit 0
fi

scratch=$(mktemp -d /tmp/task08.XXXXXX)
trap 'rm -rf -- "$scratch"' EXIT
case "$1" in
  reproduce)
    cd "$scratch"
    ros2 run nav2_map_io_lab map_io_demo "$@"
    ;;
  compatibility)
    cd "$scratch"
    ros2 run nav2_map_io_lab map_io_demo compatibility /lab/fixtures/existing/inspection.yaml
    ;;
  test)
    build_dir=/work/build/nav2_map_server
    if [[ $# == 1 ]]; then
      ctest --test-dir "$build_dir" -R '^test_map_io$' --verbose --no-tests=error
    else
      cd "$build_dir/test/unit"
      ./test_map_io --gtest_filter="$2" --gtest_output="xml:$scratch/result.xml"
      python3 - "$scratch/result.xml" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

report = Path(sys.argv[1])
if not report.is_file() or int(ET.parse(report).getroot().attrib["tests"]) == 0:
    sys.exit("No gtest matched the filter.")
PY
    fi
    ;;
esac
