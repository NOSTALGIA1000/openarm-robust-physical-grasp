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


Mat2 = Tuple[Tuple[float, float], Tuple[float, float]]


def jacobian_from_probes(dp_a: Tuple[float, float], de_a: Tuple[float, float],
                         dp_b: Tuple[float, float], de_b: Tuple[float, float]) -> Mat2:
    """Image Jacobian from two deliberate probe motions.

    This is what makes a baseline honestly uncalibrated: the mapping from world
    motion to what the camera sees is measured by moving and looking, never
    read off an extrinsic. Two probes along independent directions give the two
    columns.

    dp_* are world (x, y) displacements, de_* the camera-frame (x, y) changes
    they produced. Returns J with de = J . dp.
    """
    (ax, ay), (bx, by) = dp_a, dp_b
    det = ax * by - ay * bx
    if abs(det) < 1e-12:
        raise ValueError("probe directions are parallel")
    # Columns of J solve [de_a de_b] = J [dp_a dp_b]; invert the 2x2 of probes.
    i00, i01 = by / det, -bx / det
    i10, i11 = -ay / det, ax / det
    return (
        (de_a[0] * i00 + de_b[0] * i10, de_a[0] * i01 + de_b[0] * i11),
        (de_a[1] * i00 + de_b[1] * i10, de_a[1] * i01 + de_b[1] * i11),
    )


def broyden_update(jac: Mat2, dp: Tuple[float, float],
                   de: Tuple[float, float]) -> Mat2:
    """Rank-one correction of the Jacobian from one observed move.

    Keeps the estimate current as the arm changes configuration, without
    spending further probe motions on it.
    """
    dpn = dp[0] * dp[0] + dp[1] * dp[1]
    if dpn < 1e-12:
        return jac
    pred = (jac[0][0] * dp[0] + jac[0][1] * dp[1],
            jac[1][0] * dp[0] + jac[1][1] * dp[1])
    r = (de[0] - pred[0], de[1] - pred[1])
    return (
        (jac[0][0] + r[0] * dp[0] / dpn, jac[0][1] + r[0] * dp[1] / dpn),
        (jac[1][0] + r[1] * dp[0] / dpn, jac[1][1] + r[1] * dp[1] / dpn),
    )


def servo_step(jac: Mat2, err: Tuple[float, float], gain: float,
               max_step: float) -> Tuple[float, float]:
    """World displacement that should cancel `gain` of the image-space error.

    Solves J . dp = gain * err. The step is clipped so a poor Jacobian early in
    the loop cannot throw the arm across the table.
    """
    (a, b), (c, d) = jac
    det = a * d - b * c
    if abs(det) < 1e-12:
        raise ValueError("Jacobian estimate is singular")
    ex, ey = gain * err[0], gain * err[1]
    dx = (d * ex - b * ey) / det
    dy = (-c * ex + a * ey) / det
    mag = math.hypot(dx, dy)
    if mag > max_step > 0.0:
        dx, dy = dx * max_step / mag, dy * max_step / mag
    return dx, dy
