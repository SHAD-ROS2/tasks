"""Supplied infrastructure checks pass even before the homework policy exists."""
from rover_tf_validator.contracts import Observation, RigidTransform, report
from rover_tf_validator.geometry import apply_transform
from rover_tf_validator.queue import RetryQueue


def observation():
    return Observation('lidar', 'sensor', 'odom', 2_000_000_000, (1.,0.,0.))


def test_transform_rotates_then_translates():
    q = 2**-.5
    assert all(abs(a-b)<1e-12 for a,b in zip(
        apply_transform(RigidTransform((2.,3.,4.), (0.,0.,q,q)), (1.,0.,0.)), (2.,4.,4.)))


def test_retry_queue_finishes_when_steady_deadline_expires():
    clock, output = [0.], []
    def unavailable(obs, lookup):
        return report(obs, 'TIME_UNAVAILABLE', 'future extrapolation')
    queue = RetryQueue(unavailable, None, output.append, timeout=.5, now=lambda:clock[0])
    queue.submit(observation())
    queue.tick()
    assert not output
    clock[0] = .51  # ROS clock may remain paused throughout.
    queue.tick()
    assert output[0]['status'] == 'TIME_UNAVAILABLE'
    assert output[0]['wait_expired'] is True
    assert not queue.pending


def test_queue_retries_without_losing_measurement_stamp():
    output, ready = [], [False]
    def attempt(obs, lookup):
        return report(obs, 'OK' if ready[0] else 'UNKNOWN_FRAME', point=(1.,2.,3.))
    queue = RetryQueue(attempt, None, output.append)
    queue.submit(observation())
    queue.tick()
    assert not output
    ready[0] = True
    queue.tick()
    assert output[0]['stamp_ns'] == 2_000_000_000
    assert output[0]['attempts'] == 2


def test_queue_capacity_and_clock_reset_are_bounded():
    output = []
    queue = RetryQueue(None, None, output.append, capacity=1)
    queue.submit(observation()); queue.submit(observation())
    assert len(output) == 1 and len(queue.pending) == 1
    queue.clear()
    assert len(output) == 2 and not queue.pending
