"""Pose arithmetic for grasp evidence.

Slip is only meaningful in the gripper frame: an object carried perfectly by a
moving gripper has large world motion and zero relative motion.
"""

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]


def _rotation_matrix(quat_wxyz: Sequence[float]):
    w, x, y, z = (float(v) for v in quat_wxyz)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def relative_position(
    object_xyz: Sequence[float],
    gripper_xyz: Sequence[float],
    gripper_quat_wxyz: Sequence[float],
) -> Vec3:
    """Object position expressed in the gripper frame."""
    delta = tuple(float(o) - float(g) for o, g in zip(object_xyz, gripper_xyz))
    rotation = _rotation_matrix(gripper_quat_wxyz)
    # Transpose of the rotation maps world vectors into the gripper frame.
    return tuple(
        sum(rotation[row][col] * delta[row] for row in range(3)) for col in range(3)
    )


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(p) - float(q)) ** 2 for p, q in zip(a, b)))


def elevation(object_z: float, baseline_z: float) -> float:
    return float(object_z) - float(baseline_z)


def shrink_radially(point: Vec3, gain: float) -> Vec3:
    """Pull an optical reading in towards the optical axis by gain*sin(theta).

    A body's optical centroid sits off its own origin by an amount that grows
    with how far off-axis it is seen, along the radial direction. The offsets
    of object and gripper cancel while both are viewed from the same
    direction, and stop cancelling once a calibration bias separates them -
    which is where the residual comes from. Theta is taken from the reading
    itself, so no extrinsic is involved.

    gain is the effective lever in metres; zero returns the point unchanged.
    """
    x, y, z = point
    if not gain or z <= 0.0:
        return point
    lateral = math.hypot(x, y)
    if lateral <= 1e-9:
        return point
    sin_theta = lateral / math.hypot(lateral, z)
    scale = max(0.0, (lateral - gain * sin_theta) / lateral)
    return (x * scale, y * scale, z)
