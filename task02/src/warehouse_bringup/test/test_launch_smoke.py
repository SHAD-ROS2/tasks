from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchDescription


LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def load_launch_description(filename: str) -> LaunchDescription:
    path = LAUNCH_DIR / filename
    module_name = path.name.replace(".", "_")
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    return description


@pytest.mark.parametrize(
    "filename",
    ["warehouse.launch.py", "two_robots.launch.py"],
)
def test_launch_description_can_be_constructed(filename: str) -> None:
    description = load_launch_description(filename)
    assert description.entities
