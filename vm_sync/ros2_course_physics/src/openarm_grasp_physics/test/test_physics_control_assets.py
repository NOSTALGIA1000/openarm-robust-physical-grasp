"""Static guarantees about the physics-mode URDF, SRDF and controller config."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.expanduser("~/ros2_ws_course_repro/ros2_ws/src/openarm_moveit_config")
URDF = os.path.join(PKG, "model", "table_scene_physics.urdf")
SRDF = os.path.join(PKG, "config", "openarm_physics.srdf")
CONTROLLERS = os.path.join(PKG, "launch", "physics_controllers.yaml")

CUBE_JOINTS = {"cube_x", "cube_y", "cube_z"}


@pytest.fixture(scope="module")
def urdf_root():
    return ET.parse(URDF).getroot()


@pytest.fixture(scope="module")
def srdf_root():
    return ET.parse(SRDF).getroot()


@pytest.fixture(scope="module")
def controllers():
    with open(CONTROLLERS) as handle:
        return yaml.safe_load(handle)


def test_urdf_has_no_object_joints_links_or_interfaces(urdf_root):
    joint_names = {j.get("name") for j in urdf_root.iter("joint")}
    assert joint_names.isdisjoint(CUBE_JOINTS)

    link_names = {link.get("name") for link in urdf_root.iter("link")}
    assert link_names.isdisjoint({"cube_link_1", "cube_link_2", "red_cube_link"})


def test_left_finger_uses_bounded_pid_force(urdf_root):
    control = urdf_root.find("ros2_control")
    finger = [
        j for j in control.findall("joint")
        if j.get("name") == "openarm_left_finger_joint1"
    ]
    assert len(finger) == 1

    interfaces = {c.get("name") for c in finger[0].findall("command_interface")}
    assert interfaces == {"position_pid"}

    params = {p.get("name"): p.text for p in finger[0].iter("param")}
    assert "position_kp" in params
    assert float(params["position_max_effort"]) == 25.0


def test_arm_joints_use_bounded_pid_force():
    """Stage 3 moved the arm off direct qpos writes.

    With plain position control the joints are teleported, so the reaction from
    a 15 N grip pushes the unactuated arm around between control updates and the
    grasp is lost mid-lift. Bounded PID force lets actuator effort resist that
    reaction, which is what makes the lift work.
    """
    root = ET.parse(URDF).getroot()
    control = root.find("ros2_control")
    limits = {1: 40.0, 2: 40.0, 3: 27.0, 4: 27.0, 5: 7.0, 6: 7.0, 7: 7.0}
    for i, limit in limits.items():
        joint = [
            j for j in control.findall("joint")
            if j.get("name") == "openarm_left_joint%d" % i
        ]
        assert len(joint) == 1, "openarm_left_joint%d missing" % i
        interfaces = {c.get("name") for c in joint[0].findall("command_interface")}
        assert interfaces == {"position_pid"}, (
            "joint%d should be force-controlled, got %s" % (i, interfaces))
        params = {p.get("name"): p.text for p in joint[0].iter("param")}
        assert float(params["position_max_effort"]) == limit


def test_srdf_declares_no_joint_absent_from_the_moveit_model(srdf_root):
    joint_names = {j.get("name") for j in srdf_root.iter("joint")}
    assert joint_names.isdisjoint(CUBE_JOINTS)
    assert joint_names.isdisjoint({"marker_x", "marker_y", "marker_z"})

    group_names = {g.get("name") for g in srdf_root.iter("group")}
    assert "cube_group" not in group_names
    assert "marker_group" not in group_names

    collision_links = set()
    for entry in srdf_root.iter("disable_collisions"):
        collision_links.add(entry.get("link1"))
        collision_links.add(entry.get("link2"))
    assert "red_cube_link" not in collision_links


def test_controller_config_has_no_object_controller(controllers):
    manager = controllers["controller_manager"]["ros__parameters"]
    assert "cube_controller" not in manager
    assert "cube_controller" not in controllers
    assert manager["update_rate"] == 100
