"""Classification of raw MuJoCo object contacts into a grasp state.

The simulator publishes every contact the object is involved in. Only the two
named finger geoms count towards a grasp: palm, arm and table contacts are
evidence of collision, not of grasping.

They are evidence, and until 2026-09-18 they were discarded unread. The
right-arm smoke test that day recorded three trials in which the cube left at
11 m/s during descend with both finger contacts false and zero finger force,
and the record could not say what hit it, because the only contacts written to
samples.csv were the two the classifier cared about. The telemetry had been
publishing the rest all along. They are summarised here now: how many there
were, and the name and force of the strongest, so a link sweeping through the
object is visible in the record instead of showing up as an object that is no
longer there.

The table is normally among them - the object rests on it - so a count of one
is the resting state and the geom name is what distinguishes a collision from
gravity.
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
    # Contacts with the object from anything that is not one of the two
    # fingers, including the table. The name and force are the largest of them.
    other_count: int = 0
    other_geom: str = ""
    other_force: float = 0.0


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
    other_count = 0
    other_geom = ""
    other_force = 0.0
    for name, force in zip(geoms, normal_forces):
        magnitude = abs(float(force))
        if name == config.left_finger_geom:
            left_force += magnitude
        elif name == config.right_finger_geom:
            right_force += magnitude
        else:
            other_count += 1
            if magnitude >= other_force:
                other_force = magnitude
                other_geom = name

    left_contact = left_force >= config.min_normal_force
    right_contact = right_force >= config.min_normal_force

    return ContactSummary(
        left_contact=left_contact,
        right_contact=right_contact,
        bilateral=left_contact and right_contact,
        left_normal_force=left_force,
        right_normal_force=right_force,
        other_count=other_count,
        other_geom=other_geom,
        other_force=other_force,
    )
