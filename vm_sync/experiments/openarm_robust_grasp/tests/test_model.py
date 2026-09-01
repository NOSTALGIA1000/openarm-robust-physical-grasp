from pathlib import Path

from openarm_robust_grasp.model import load_scene


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")
CUBE_LIFT_SCENE = Path(
    "/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml"
)


def test_load_scene_resolves_named_indices():
    ctx = load_scene(str(SCENE))

    assert ctx.model.nu == 16
    assert "tablecam" in ctx.cameras
    assert "bottle" in ctx.objects
    assert ctx.left.ee_body_name == "openarm_left_ee_base_link"
    assert len(ctx.left.joint_qpos_ids) == 7
    assert len(ctx.left.actuator_ids) == 7
    assert ctx.left.gripper_actuator_id >= 0


def test_load_scene_uses_home_keyframe_when_available():
    ctx = load_scene(str(SCENE))

    assert ctx.data.qpos[ctx.left.joint_qpos_ids[3]] > 1.0


def test_load_experiment_cube_lift_scene():
    ctx = load_scene(str(CUBE_LIFT_SCENE))

    assert "orange_cube" in ctx.objects
    assert ctx.model.nq > 20
    assert ctx.model.body_mass[ctx.objects["orange_cube"]] < 0.05
