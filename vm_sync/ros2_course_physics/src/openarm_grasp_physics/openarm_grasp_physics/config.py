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
    open_position: float = 0.030
    close_position: float = 0.006
    open_tolerance: float = 0.005

    # Motion offsets in metres relative to the observed object surface.
    hover_offset_m: float = 0.08
    grasp_z_offset_m: float = -0.02

    # The gripper's jaw centre is not coincident with the commanded TCP frame.
    # Measured from the model (constant across finger opening): the jaw centre
    # sits at (0, +0.006, +0.005) m in TCP local coordinates. Without this
    # compensation the object ends up ~6 mm toward one finger, so that finger
    # always contacts first and knocks it out of the jaw before the other
    # closes.
    jaw_offset_x: float = 0.0
    jaw_offset_y: float = 0.0
    jaw_offset_z: float = 0.005

    # MoveIt plans in joint space, so one long descend lets the TCP bow away
    # from vertical and the fingers sweep laterally through the object. That
    # was measured to spin the cube ~12 deg about z before the gripper closed,
    # after which closing strikes its corners and cams it out. Descending in
    # short increments keeps the path near-vertical.
    descend_steps: int = 1

    # Stage 1 isolates grasp mechanics from perception, so a detector
    # observation is a nice-to-have trigger, not a precondition. The course
    # detector is tuned for the original object and does not always see a
    # smaller one; blocking on it would inject a perception limitation into a
    # mechanics experiment. Stage 2 re-introduces perception deliberately.
    detector_wait_s: float = 20.0

    # Timeouts, all in simulation seconds.
    settle_duration_s: float = 1.0
    contact_confirm_timeout_s: float = 5.0
    gripper_action_timeout_s: float = 10.0
    plan_timeout_s: float = 30.0

    def snapshot(self) -> dict:
        """Plain dict for embedding in trial records."""
        return asdict(self)
