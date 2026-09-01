"""Declared Stage 1 grasp configuration.

Every value here is a research parameter: changing one changes what
"successful grasp" means. They live in a single frozen dataclass so a
snapshot can be written into every trial record.
"""

from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class GraspConfig:
    # MuJoCo geom names used for contact classification.
    object_geom: str = "physics_object_geom"
    left_finger_geom: str = "openarm_left_left_finger_collision"
    right_finger_geom: str = "openarm_left_right_finger_collision"

    # Contact acceptance.
    min_normal_force: float = 0.5
    confirm_window_s: float = 0.30

    # Grasp acceptance.
    lift_height_m: float = 0.020
    hold_duration_s: float = 2.0
    min_bilateral_ratio: float = 0.80
    max_drift_m: float = 0.010

    # Finger joint travel in metres. 0.0 is closed, 0.06 is fully open.
    open_position: float = 0.055
    close_position: float = 0.0
    open_tolerance: float = 0.005

    # Motion offsets in metres relative to the observed object surface.
    hover_offset_m: float = 0.08
    grasp_z_offset_m: float = 0.0

    # Timeouts, all in simulation seconds.
    settle_duration_s: float = 1.0
    contact_confirm_timeout_s: float = 5.0
    gripper_action_timeout_s: float = 10.0
    plan_timeout_s: float = 30.0

    def snapshot(self) -> dict:
        """Plain dict for embedding in trial records."""
        return asdict(self)
