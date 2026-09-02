import dataclasses

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import (
    FailureCategory,
    TrialObservations,
    classify_trial,
)

CONFIG = GraspConfig()

PASSING = TrialObservations(
    planning_ok=True,
    gripper_ok=True,
    timed_out=False,
    any_contact=True,
    bilateral_confirmed=True,
    contact_lost_during_lift=False,
    max_elevation_m=0.025,
    hold_completed=True,
    hold_bilateral_ratio=0.95,
    max_drift_m=0.004,
)


def observations(**overrides):
    return dataclasses.replace(PASSING, **overrides)


def test_all_conditions_met_is_success():
    assert classify_trial(PASSING, CONFIG) is FailureCategory.SUCCESS


def test_planning_failure_wins_over_everything():
    result = classify_trial(
        observations(planning_ok=False, any_contact=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.PLANNING_FAILURE


def test_gripper_command_failure_precedes_contact_categories():
    result = classify_trial(observations(gripper_ok=False, any_contact=False), CONFIG)

    assert result is FailureCategory.GRIPPER_COMMAND_FAILURE


def test_no_contact_at_all():
    result = classify_trial(
        observations(any_contact=False, bilateral_confirmed=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.NO_CONTACT


def test_contact_that_never_became_bilateral():
    result = classify_trial(
        observations(bilateral_confirmed=False, max_elevation_m=0.0),
        CONFIG,
    )

    assert result is FailureCategory.UNILATERAL_CONTACT


def test_contact_lost_during_lift():
    result = classify_trial(
        observations(contact_lost_during_lift=True, max_elevation_m=0.005),
        CONFIG,
    )

    assert result is FailureCategory.CONTACT_LOST_DURING_LIFT


def test_lift_below_threshold():
    result = classify_trial(observations(max_elevation_m=0.019), CONFIG)

    assert result is FailureCategory.INSUFFICIENT_LIFT


def test_lift_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(max_elevation_m=0.020), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_drift_above_threshold():
    result = classify_trial(observations(max_drift_m=0.0101), CONFIG)

    assert result is FailureCategory.EXCESSIVE_SLIP


def test_drift_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(max_drift_m=0.010), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_hold_not_completed():
    result = classify_trial(observations(hold_completed=False), CONFIG)

    assert result is FailureCategory.HOLD_FAILURE


def test_hold_bilateral_ratio_below_threshold():
    result = classify_trial(observations(hold_bilateral_ratio=0.79), CONFIG)

    assert result is FailureCategory.HOLD_FAILURE


def test_hold_bilateral_ratio_exactly_at_threshold_is_accepted():
    result = classify_trial(observations(hold_bilateral_ratio=0.80), CONFIG)

    assert result is FailureCategory.SUCCESS


def test_timeout_is_reported_as_planning_failure_when_planning_failed():
    result = classify_trial(observations(planning_ok=False, timed_out=True), CONFIG)

    assert result is FailureCategory.PLANNING_FAILURE


def test_monitoring_timeout_alone_blocks_success():
    result = classify_trial(observations(timed_out=True), CONFIG)

    assert result is not FailureCategory.SUCCESS
