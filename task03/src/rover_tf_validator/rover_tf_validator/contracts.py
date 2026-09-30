"""Small ROS-independent policy contract. These supplied types need no edits."""
from dataclasses import dataclass
from typing import Callable

STATUSES = frozenset({'OK', 'INVALID_INPUT', 'UNKNOWN_FRAME', 'DISCONNECTED',
                      'TIME_UNAVAILABLE', 'NOT_IMPLEMENTED'})
RETRYABLE = frozenset({'UNKNOWN_FRAME', 'DISCONNECTED', 'TIME_UNAVAILABLE'})


@dataclass(frozen=True)
class Observation:
    sensor: str
    source_frame: str
    target_frame: str
    stamp_ns: int
    point: tuple[float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # x, y, z, w


class LookupFailure(Exception):
    def __init__(self, status: str, detail: str):
        if status not in RETRYABLE:
            raise ValueError('lookup failures must be retryable TF categories')
        self.status, self.detail = status, detail
        super().__init__(detail)


Lookup = Callable[[str, str, int], RigidTransform]


def report(observation: Observation, status: str, detail: str = '', point=None):
    """Common report schema; stamp always describes the input measurement."""
    if status not in STATUSES:
        raise ValueError(f'Unsupported status: {status}')
    return {
        'kind': 'observation', 'sensor': observation.sensor,
        'source_frame': observation.source_frame,
        'target_frame': observation.target_frame,
        'stamp_ns': observation.stamp_ns, 'status': status, 'detail': detail,
        'point': list(point) if point is not None else None,
    }
