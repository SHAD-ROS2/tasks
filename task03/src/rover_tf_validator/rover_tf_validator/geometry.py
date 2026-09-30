"""Supplied numerical operation: active rotation followed by translation."""
from .contracts import RigidTransform


def apply_transform(transform: RigidTransform, point):
    """Quaternion must already have been validated as unit length (x,y,z,w)."""
    x, y, z, w = transform.rotation
    px, py, pz = point
    # q * (p, 0) * inverse(q), expanded to avoid an external numeric dependency.
    tx, ty, tz = 2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px)
    rotated = (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx)
    return tuple(a+b for a, b in zip(rotated, transform.translation))
