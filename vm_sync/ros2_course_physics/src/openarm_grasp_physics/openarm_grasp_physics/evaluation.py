"""Deterministic single-category outcome for one grasp trial.

A trial may exhibit several symptoms at once. The canonical result is the
first category in the fixed precedence order below, so results stay
comparable across runs.
"""

from dataclasses import dataclass
from enum import Enum

from openarm_grasp_physics.config import GraspConfig


class FailureCategory(str, Enum):
    DETECTOR_TIMEOUT = "detector_timeout"
    PLANNING_FAILURE = "planning_failure"
    GRIPPER_COMMAND_FAILURE = "gripper_command_failure"
    NO_CONTACT = "no_contact"
    UNILATERAL_CONTACT = "unilateral_contact"
    CONTACT_LOST_DURING_LIFT = "contact_lost_during_lift"
    INSUFFICIENT_LIFT = "insufficient_lift"
    EXCESSIVE_SLIP = "excessive_slip"
    HOLD_FAILURE = "hold_failure"
    SUCCESS = "success"


@dataclass(frozen=True)
class TrialObservations:
    planning_ok: bool
    gripper_ok: bool
    timed_out: bool
    any_contact: bool
    bilateral_confirmed: bool
    contact_lost_during_lift: bool
    max_elevation_m: float
    hold_completed: bool
    hold_bilateral_ratio: float
    max_drift_m: float
    # A trial that never got a detector fix never reached the planner.
    # Folding it into planning_failure hid a perception-readiness
    # problem inside a kinematics category: of 20 such trials, 5 were
    # actually this. Defaults True so existing callers keep their meaning.
    detector_ok: bool = True


def classify_trial(
    observations: TrialObservations, config: GraspConfig
) -> FailureCategory:
    if not observations.detector_ok:
        return FailureCategory.DETECTOR_TIMEOUT
    if not observations.planning_ok:
        return FailureCategory.PLANNING_FAILURE
    if not observations.gripper_ok:
        return FailureCategory.GRIPPER_COMMAND_FAILURE
    if not observations.any_contact:
        return FailureCategory.NO_CONTACT
    if not observations.bilateral_confirmed:
        return FailureCategory.UNILATERAL_CONTACT
    if observations.contact_lost_during_lift:
        return FailureCategory.CONTACT_LOST_DURING_LIFT
    if observations.max_elevation_m < config.lift_height_m:
        return FailureCategory.INSUFFICIENT_LIFT
    if observations.max_drift_m > config.max_drift_m:
        return FailureCategory.EXCESSIVE_SLIP
    if not observations.hold_completed:
        return FailureCategory.HOLD_FAILURE
    if observations.hold_bilateral_ratio < config.min_bilateral_ratio:
        return FailureCategory.HOLD_FAILURE
    if observations.timed_out:
        return FailureCategory.HOLD_FAILURE
    return FailureCategory.SUCCESS
