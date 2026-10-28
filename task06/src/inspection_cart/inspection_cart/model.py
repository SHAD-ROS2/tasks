"""Tiny standard-library URDF forward kinematics oracle for this exercise.

Only fixed/revolute/continuous/prismatic tree joints are supported. This is not
a URDF validator or a replacement for robot_state_publisher/urdfdom.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _xyz(text):
    values = tuple(float(value) for value in text.split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("expected three finite coordinates")
    return values


def _identity():
    return [[float(row == col) for col in range(4)] for row in range(4)]


def _multiply(a, b):
    return [[sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
            for row in range(4)]


def _rotation(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    if length == 0:
        raise ValueError("zero joint axis")
    x, y, z = (value / length for value in axis)
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = _identity()
    rows = [
        [x*x*v+c, x*y*v-z*s, x*z*v+y*s],
        [y*x*v+z*s, y*y*v+c, y*z*v-x*s],
        [z*x*v-y*s, z*y*v+x*s, z*z*v+c],
    ]
    for row in range(3):
        result[row][:3] = rows[row]
    return result


def _origin(joint):
    origin = joint.find("origin")
    xyz = _xyz(origin.get("xyz", "0 0 0")) if origin is not None else (0, 0, 0)
    rpy = _xyz(origin.get("rpy", "0 0 0")) if origin is not None else (0, 0, 0)
    rotation = _multiply(_rotation((0, 0, 1), rpy[2]),
                         _multiply(_rotation((0, 1, 0), rpy[1]), _rotation((1, 0, 0), rpy[0])))
    for row in range(3):
        rotation[row][3] = xyz[row]
    return rotation


def frame_position(model_path, frame, positions=None, root="base_link"):
    """Return frame origin in root coordinates in metres, using named joint angles."""
    document = ET.parse(Path(model_path)).getroot()
    joints = {joint.find("child").get("link"): joint for joint in document.findall("joint")}
    chain, visited = [], set()
    while frame != root:
        if frame in visited or frame not in joints:
            raise ValueError(f"no acyclic path from {root} to {frame}")
        visited.add(frame)
        joint = joints[frame]
        chain.append(joint)
        frame = joint.find("parent").get("link")
    transform = _identity()
    for joint in reversed(chain):
        transform = _multiply(transform, _origin(joint))
        kind = joint.get("type")
        if kind == "fixed":
            continue
        q = float((positions or {}).get(joint.get("name"), 0.0))
        if not math.isfinite(q):
            raise ValueError("joint value must be finite")
        axis_element = joint.find("axis")
        axis = _xyz(axis_element.get("xyz", "1 0 0")) if axis_element is not None else (1, 0, 0)
        if kind in {"revolute", "continuous"}:
            motion = _rotation(axis, q)
        elif kind == "prismatic":
            motion = _identity()
            for row in range(3):
                motion[row][3] = axis[row] * q
        else:
            raise ValueError(f"unsupported joint type {kind}")
        transform = _multiply(transform, motion)
    return tuple(transform[row][3] for row in range(3))
