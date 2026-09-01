import pytest

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.contact import ContactSummary, summarize_contacts

CONFIG = GraspConfig()
LEFT = CONFIG.left_finger_geom
RIGHT = CONFIG.right_finger_geom


def test_no_contacts_is_no_grasp():
    summary = summarize_contacts([], [], CONFIG)

    assert summary == ContactSummary(False, False, False, 0.0, 0.0)


def test_one_finger_is_unilateral():
    summary = summarize_contacts([LEFT], [3.0], CONFIG)

    assert summary.left_contact is True
    assert summary.right_contact is False
    assert summary.bilateral is False


def test_both_fingers_is_bilateral():
    summary = summarize_contacts([LEFT, RIGHT], [3.0, 2.0], CONFIG)

    assert summary.bilateral is True
    assert summary.left_normal_force == 3.0
    assert summary.right_normal_force == 2.0


def test_table_and_palm_contacts_are_ignored():
    summary = summarize_contacts(
        ["table_top", "openarm_left_hand_collision", LEFT],
        [12.0, 8.0, 3.0],
        CONFIG,
    )

    assert summary.left_contact is True
    assert summary.right_contact is False
    assert summary.right_normal_force == 0.0


def test_multiple_contacts_on_one_finger_are_summed():
    summary = summarize_contacts([LEFT, LEFT, RIGHT], [1.0, 2.5, 4.0], CONFIG)

    assert summary.left_normal_force == 3.5
    assert summary.bilateral is True


def test_force_below_threshold_is_not_contact():
    summary = summarize_contacts([LEFT, RIGHT], [0.4, 4.0], CONFIG)

    assert summary.left_contact is False
    assert summary.bilateral is False
    assert summary.left_normal_force == 0.4


def test_force_exactly_at_threshold_counts():
    summary = summarize_contacts([LEFT, RIGHT], [0.5, 0.5], CONFIG)

    assert summary.bilateral is True


def test_negative_normal_force_is_treated_as_magnitude():
    summary = summarize_contacts([LEFT, RIGHT], [-3.0, 3.0], CONFIG)

    assert summary.left_normal_force == 3.0
    assert summary.bilateral is True


def test_mismatched_array_lengths_are_rejected():
    with pytest.raises(ValueError):
        summarize_contacts([LEFT, RIGHT], [1.0], CONFIG)
