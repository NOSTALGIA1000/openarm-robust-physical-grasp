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
