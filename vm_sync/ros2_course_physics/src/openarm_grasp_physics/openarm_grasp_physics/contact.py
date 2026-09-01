"""Classification of raw MuJoCo object contacts into a grasp state.

The simulator publishes every contact the object is involved in. Only the two
named left-hand finger geoms count towards a grasp: palm, arm and table
contacts are evidence of collision, not of grasping.
"""

from dataclasses import dataclass
from typing import Sequence

from openarm_grasp_physics.config import GraspConfig


@dataclass(frozen=True)
class ContactSummary:
    left_contact: bool
    right_contact: bool
    bilateral: bool
    left_normal_force: float
    right_normal_force: float


def summarize_contacts(
    geoms: Sequence[str],
    normal_forces: Sequence[float],
    config: GraspConfig,
) -> ContactSummary:
    if len(geoms) != len(normal_forces):
        raise ValueError(
            "geoms and normal_forces must have equal length, got "
            f"{len(geoms)} and {len(normal_forces)}"
        )

    left_force = 0.0
    right_force = 0.0
    for name, force in zip(geoms, normal_forces):
        magnitude = abs(float(force))
        if name == config.left_finger_geom:
            left_force += magnitude
        elif name == config.right_finger_geom:
            right_force += magnitude

    left_contact = left_force >= config.min_normal_force
    right_contact = right_force >= config.min_normal_force

    return ContactSummary(
        left_contact=left_contact,
        right_contact=right_contact,
        bilateral=left_contact and right_contact,
        left_normal_force=left_force,
        right_normal_force=right_force,
    )
