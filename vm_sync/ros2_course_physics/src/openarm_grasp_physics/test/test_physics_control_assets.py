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
    assert float(params["position_max_effort"]) == 20.0


def test_arm_joints_keep_the_plain_position_interface(urdf_root):
    control = urdf_root.find("ros2_control")
    arm = [
        j for j in control.findall("joint")
        if j.get("name") == "openarm_left_joint1"
    ]
    interfaces = {c.get("name") for c in arm[0].findall("command_interface")}
    assert interfaces == {"position"}


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
