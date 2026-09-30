"""Tests of stand calibration, independent from the student's TF implementation."""
import math

import pytest

from rover_fixture.truth import (LANDMARKS, MOUNTS, PRACTICE_LANDMARK,
                                 cart_probe_pose, compose, local_point, rotate, rover_pose)


def world_point(pose, local):
    return tuple(a+b for a, b in zip(rotate(pose.q, local), pose.xyz))


def test_rep103_camera_optical_basis():
    rotation = MOUNTS['camera'].q
    assert rotate(rotation, (1., 0., 0.)) == pytest.approx((0., -1., 0.))
    assert rotate(rotation, (0., 1., 0.)) == pytest.approx((0., 0., -1.))
    assert rotate(rotation, (0., 0., 1.)) == pytest.approx((1., 0., 0.))


@pytest.mark.parametrize('stamp', [10., 10.025, 12.35, 20.])
def test_independent_sensor_observations_recover_known_landmark(stamp):
    for name in LANDMARKS:
        pose = compose(rover_pose(stamp), MOUNTS[name])
        assert world_point(pose, local_point(pose, LANDMARKS[name])) == pytest.approx(LANDMARKS[name])


def test_latest_lookup_has_measurable_error_when_rover_moves():
    at_measurement = compose(rover_pose(12.), MOUNTS['lidar'])
    at_delivery = compose(rover_pose(12.2), MOUNTS['lidar'])
    local = local_point(at_measurement, LANDMARKS['lidar'])
    assert math.dist(world_point(at_delivery, local), LANDMARKS['lidar']) > .02


def test_cart_joint_changes_do_not_rewrite_old_observation():
    old_pose = cart_probe_pose(12., joint_angle=.4)
    new_pose = cart_probe_pose(12.2, joint_angle=.9)
    old_sample = local_point(old_pose, PRACTICE_LANDMARK)
    assert world_point(old_pose, old_sample) == pytest.approx(PRACTICE_LANDMARK)
    assert math.dist(world_point(new_pose, old_sample), PRACTICE_LANDMARK) > .1
