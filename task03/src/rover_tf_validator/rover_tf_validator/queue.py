"""Supplied bounded retry queue. Its clock is monotonic, never ROS time."""
from collections import deque
from dataclasses import dataclass
from time import monotonic
from .contracts import RETRYABLE, report


@dataclass
class Pending:
    observation: object
    deadline: float
    attempts: int = 0


class RetryQueue:
    def __init__(self, validate, lookup, emit, timeout=.35, capacity=128, now=monotonic):
        if timeout <= 0 or capacity < 1:
            raise ValueError('timeout and capacity must be positive')
        self.validate, self.lookup, self.emit = validate, lookup, emit
        self.timeout, self.capacity, self.now = timeout, capacity, now
        self.pending = deque()

    def submit(self, observation):
        if len(self.pending) >= self.capacity:
            oldest = self.pending.popleft()
            self.emit(report(oldest.observation, 'TIME_UNAVAILABLE', 'retry queue capacity exceeded'))
        self.pending.append(Pending(observation, self.now() + self.timeout))

    def tick(self):
        # Snapshot the count so retries cannot spin indefinitely in one callback.
        for _ in range(len(self.pending)):
            item = self.pending.popleft()
            result = self.validate(item.observation, self.lookup)
            item.attempts += 1
            if result['status'] in RETRYABLE and self.now() < item.deadline:
                self.pending.append(item)
            else:
                result['attempts'] = item.attempts
                result['wait_expired'] = result['status'] in RETRYABLE
                self.emit(result)

    def clear(self, detail='ROS clock jumped backwards; pending observations discarded'):
        while self.pending:
            self.emit(report(self.pending.popleft().observation, 'TIME_UNAVAILABLE', detail))
