"""Static guarantees about the physics-mode MuJoCo model.

These are the checks that stop a physical-grasp claim from silently
regressing into the software-attached demo.
"""
import os
import xml.etree.ElementTree as ET

import pytest

from openarm_grasp_physics.config import GraspConfig

MODEL_DIR = os.path.expanduser(
    "~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config/model"
)
SCENE = os.path.join(MODEL_DIR, "table_scene_physics.xml")
ROBOT = os.path.join(MODEL_DIR, "openarm_bimanual_physics.xml")


@pytest.fixture(scope="module")
def scene_root():
    return ET.parse(SCENE).getroot()


@pytest.fixture(scope="module")
def robot_root():
    return ET.parse(ROBOT).getroot()


def test_object_is_a_single_free_body(scene_root):
    bodies = [b for b in scene_root.iter("body") if b.get("name") == "physics_object"]
    assert len(bodies) == 1
    assert len(bodies[0].findall("freejoint")) == 1

    geoms = [g for g in bodies[0].findall("geom")]
    assert [g.get("name") for g in geoms] == [GraspConfig().object_geom]


def test_scene_has_no_object_slide_joints_or_actuators(scene_root):
    joint_names = {j.get("name") for j in scene_root.iter("joint")}
    assert joint_names.isdisjoint({"cube_x", "cube_y", "cube_z"})

    actuator_joints = {a.get("joint") for a in scene_root.iter("position")}
    assert actuator_joints.isdisjoint({"cube_x", "cube_y", "cube_z"})


def test_physics_timestep_is_unchanged(scene_root):
    options = list(scene_root.iter("option"))
    assert len(options) == 1
    assert options[0].get("timestep") == "0.002"


def test_finger_coupling_uses_equal_coordinates(robot_root):
    couplings = [
        j for j in robot_root.iter("joint")
        if j.get("joint1") is not None and "finger" in j.get("joint1")
    ]
    assert len(couplings) == 2
    for coupling in couplings:
        assert coupling.get("polycoef") == "0 1 0 0 0"


def test_finger_position_actuators_are_absent(robot_root):
    actuator_blocks = list(robot_root.iter("actuator"))
    assert len(actuator_blocks) == 1
    finger_actuators = [
        a for a in actuator_blocks[0]
        if a.get("joint") is not None and "finger" in a.get("joint")
    ]
    assert finger_actuators == []
