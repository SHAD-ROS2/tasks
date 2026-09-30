"""Public stand specification. Independent of editable description and labels.

There is no physics engine: scans, images and IMU attitude are idealized probes.
Coordinates are metres, rotations radians, quaternion order x y z w.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    xyz: tuple
    q: tuple


def quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, v):
    out = multiply(multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return out[:3]


def compose(a, b):
    offset = rotate(a.q, b.xyz)
    return Pose(tuple(x+y for x, y in zip(a.xyz, offset)), multiply(a.q, b.q))


def local_point(pose, world_point):
    return rotate((-pose.q[0], -pose.q[1], -pose.q[2], pose.q[3]),
                  tuple(a-b for a, b in zip(world_point, pose.xyz)))


LANDMARKS = {'lidar': (3.0, 0.0, 0.36), 'camera': (4.0, 0.4, 0.4),
             'imu': (1.0, 2.0, 0.6)}
MOUNTS = {'lidar': Pose((0.10, 0.0, 0.16), quaternion()),
          'camera': Pose((0.31, 0.0, 0.06), quaternion(-math.pi/2, 0.0, -math.pi/2)),
          'imu': Pose((-0.12, 0.04, 0.04), quaternion(yaw=math.pi/2))}
PRACTICE_LANDMARK = (2.0, 0.5, 0.4)


def rover_pose(stamp, moving=True):
    t = stamp-10.0 if moving else 0.0
    return Pose((0.20*t, 0.15*math.sin(0.4*t), 0.20),
                quaternion(yaw=0.20*math.sin(0.3*t)))


def cart_pose(stamp, moving=True):
    t = stamp-10.0 if moving else 0.0
    return Pose((0.12*t, 0.0, 0.15), quaternion(yaw=0.10*math.sin(t*0.4)))


def axis_angle(axis, angle):
    n = math.sqrt(sum(x*x for x in axis))
    return tuple(x/n*math.sin(angle/2) for x in axis) + (math.cos(angle/2),)


def cart_probe_pose(stamp, moving=True, joint_angle=0.4, mount_x=0.05, axis=(0., 0., 1.)):
    return compose(compose(cart_pose(stamp, moving), Pose((mount_x, 0., 0.10), axis_angle(axis, joint_angle))),
                   Pose((0.20, 0., 0.), quaternion()))
