from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class ArmIndex:
    prefix: str
    ee_body_name: str
    ee_body_id: int
    joint_names: tuple[str, ...]
    joint_qpos_ids: np.ndarray
    joint_dof_ids: np.ndarray
    actuator_names: tuple[str, ...]
    actuator_ids: np.ndarray
    gripper_actuator_name: str
    gripper_actuator_id: int
    lower_limits: np.ndarray
    upper_limits: np.ndarray


@dataclass
class SceneContext:
    scene_path: str
    model: mujoco.MjModel
    data: mujoco.MjData
    left: ArmIndex
    right: ArmIndex
    cameras: dict[str, int]
    objects: dict[str, int]


def _required_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    obj_id = mujoco.mj_name2id(model, obj_type, name)
    if obj_id < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return obj_id


def _arm_index(model: mujoco.MjModel, prefix: str) -> ArmIndex:
    joint_names = tuple(f"openarm_{prefix}_joint{i}" for i in range(1, 8))
    actuator_names = tuple(f"{prefix}_joint{i}_ctrl" for i in range(1, 8))
    ee_body_name = f"openarm_{prefix}_ee_base_link"
    gripper_actuator_name = f"{prefix}_finger1_ctrl"

    joint_ids = np.array(
        [_required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names],
        dtype=int,
    )
    actuator_ids = np.array(
        [
            _required_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in actuator_names
        ],
        dtype=int,
    )

    joint_qpos_ids = model.jnt_qposadr[joint_ids].astype(int).copy()
    joint_dof_ids = model.jnt_dofadr[joint_ids].astype(int).copy()

    return ArmIndex(
        prefix=prefix,
        ee_body_name=ee_body_name,
        ee_body_id=_required_id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name),
        joint_names=joint_names,
        joint_qpos_ids=joint_qpos_ids,
        joint_dof_ids=joint_dof_ids,
        actuator_names=actuator_names,
        actuator_ids=actuator_ids,
        gripper_actuator_name=gripper_actuator_name,
        gripper_actuator_id=_required_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, gripper_actuator_name
        ),
        lower_limits=model.jnt_range[joint_ids, 0].copy(),
        upper_limits=model.jnt_range[joint_ids, 1].copy(),
    )


def _name_map(model: mujoco.MjModel, obj_type: mujoco.mjtObj, count: int) -> dict[str, int]:
    names: dict[str, int] = {}
    for obj_id in range(count):
        name = mujoco.mj_id2name(model, obj_type, obj_id)
        if name:
            names[name] = obj_id
    return names


def load_scene(scene_path: str) -> SceneContext:
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
        for actuator_id in range(model.nu):
            joint_id = model.actuator_trnid[actuator_id, 0]
            if joint_id >= 0:
                qpos_id = model.jnt_qposadr[joint_id]
                data.ctrl[actuator_id] = data.qpos[qpos_id]

    mujoco.mj_forward(model, data)

    bodies = _name_map(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)
    object_names = ("bottle", "bottle_cap", "peg", "socket", "orange_cube")
    objects = {name: bodies[name] for name in object_names if name in bodies}

    return SceneContext(
        scene_path=scene_path,
        model=model,
        data=data,
        left=_arm_index(model, "left"),
        right=_arm_index(model, "right"),
        cameras=_name_map(model, mujoco.mjtObj.mjOBJ_CAMERA, model.ncam),
        objects=objects,
    )
