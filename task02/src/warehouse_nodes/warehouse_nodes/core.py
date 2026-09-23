"""ROS-independent behavior shared by the warehouse nodes.

Keeping the small decision functions independent of rclpy makes their contract
cheap to test while the launch tests concentrate on wiring and middleware.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable


STOP = "STOP"
FORWARD = "FORWARD"


@dataclass(frozen=True)
class RangeSample:
    sequence: int
    distance_m: float


@dataclass(frozen=True)
class ObstacleDecision:
    sequence: int
    blocked: bool
    distance_m: float


@dataclass(frozen=True)
class MotionDecision:
    sequence: int
    mode: str
    speed_mps: float


class CyclicReadings:
    """Return a deterministic, repeating sequence of range samples."""

    def __init__(self, readings_m: Iterable[float]) -> None:
        readings = tuple(float(value) for value in readings_m)
        if not readings:
            raise ValueError("readings_m must contain at least one value")
        if any(not isfinite(value) or value <= 0.0 for value in readings):
            raise ValueError("every range reading must be finite and positive")
        self._readings = readings
        self._sequence = 0

    def next(self) -> RangeSample:
        sample = RangeSample(
            sequence=self._sequence,
            distance_m=self._readings[self._sequence % len(self._readings)],
        )
        self._sequence += 1
        return sample


def detect_obstacle(sample: RangeSample, stop_distance_m: float) -> ObstacleDecision:
    threshold = _positive_finite(stop_distance_m, "stop_distance_m")
    return ObstacleDecision(
        sequence=sample.sequence,
        blocked=sample.distance_m <= threshold,
        distance_m=sample.distance_m,
    )


def plan_motion(obstacle: ObstacleDecision, cruise_speed_mps: float) -> MotionDecision:
    speed = _non_negative_finite(cruise_speed_mps, "cruise_speed_mps")
    if obstacle.blocked:
        return MotionDecision(sequence=obstacle.sequence, mode=STOP, speed_mps=0.0)
    return MotionDecision(sequence=obstacle.sequence, mode=FORWARD, speed_mps=speed)


def limit_speed(requested_mps: float, maximum_mps: float) -> float:
    requested = _non_negative_finite(requested_mps, "requested_mps")
    maximum = _non_negative_finite(maximum_mps, "maximum_mps")
    return min(requested, maximum)


def must_stop(distance_m: float, stop_distance_m: float) -> bool:
    distance = _positive_finite(distance_m, "distance_m")
    threshold = _positive_finite(stop_distance_m, "stop_distance_m")
    return distance <= threshold


def _positive_finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _non_negative_finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return result
