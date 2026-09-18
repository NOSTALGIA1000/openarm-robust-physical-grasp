"""Declared Stage 1 grasp configuration.

Every value here is a research parameter: changing one changes what
"successful grasp" means. They live in a single frozen dataclass so a
snapshot can be written into every trial record.
"""

from dataclasses import asdict, dataclass


def arm_names(arm):
    """Every name that depends on which arm is being driven.

    The two arms are not mirror images of each other. Their BASE poses are
    mirrored - the left is Rx(-90) at y=+0.031, the right Rx(+90) at y=-0.031 -
    but every link below the base is identical, so the right arm is a left arm
    mounted at a mirrored base rather than a mirrored arm. The same joint
    angles put the two hands at different places, and a pose one arm reaches
    says nothing about whether the other reaches it. Reachability is measured,
    not assumed; see docs/results for the right-arm smoke test.

    What this function does NOT touch: the detector, which finds the gripper by
    depth band and a point hint and never names an arm, and the object, which
    is a free body in the scene.
    """
    if arm not in ("left", "right"):
        raise ValueError("arm must be 'left' or 'right', got %r" % (arm,))
    return {
        "group": "%s_arm" % arm,
        "ee_link": "openarm_%s_hand_tcp" % arm,
        "gripper_action": "/%s_hand_controller/gripper_cmd" % arm,
        "joint_prefix": "openarm_%s_joint" % arm,
        "gripper_body": "openarm_%s_hand" % arm,
        # Finger geoms are named <arm>_left_finger and <arm>_right_finger:
        # the second word is the finger's side within the hand, not the arm.
        "left_finger_geom": "openarm_%s_left_finger_collision" % arm,
        "right_finger_geom": "openarm_%s_right_finger_collision" % arm,
        "branch_joint": "openarm_%s_joint5" % arm,
    }


@dataclass(frozen=True)
class GraspConfig:
    # Which arm drives the trial. Recorded in every trial snapshot because a
    # batch run on one arm says nothing about the other.
    arm: str = "left"

    # MuJoCo geom names used for contact classification.
    object_geom: str = "physics_object_geom"
    left_finger_geom: str = "openarm_left_left_finger_collision"
    right_finger_geom: str = "openarm_left_right_finger_collision"

    # Contact acceptance.
    min_normal_force: float = 0.5
    confirm_window_s: float = 0.30

    # Grasp acceptance.
    # Acceptance threshold: how far the object must actually rise.
    lift_height_m: float = 0.020
    # How far the gripper is commanded to rise, which is a separate question.
    # These were one constant, so the commanded lift could not be raised
    # without also raising the bar it was judged against. That coupling hid a
    # size effect: at a 30mm command the object slips in the jaws, 4.5mm for a
    # 50mm cube but 8.5 to 11.5mm for a 30mm one, so the smaller cube landed at
    # 18.4mm mean against the same 20mm threshold and 25 of its trials were
    # recorded as insufficient_lift - every one of them between 15.5 and
    # 19.95mm. The default reproduces the old lift_height_m + 0.01 exactly.
    lift_command_m: float = 0.030
    hold_duration_s: float = 2.0
    min_bilateral_ratio: float = 0.80
    max_drift_m: float = 0.010

    # Finger joint travel in metres. 0.0 is closed, 0.06 is fully open.
    #
    # Each finger travels this far, so the pre-grasp gap is twice it: 60mm
    # against a 30mm object leaves 15mm of clearance per side. That number is
    # the self-centering budget the tolerance results keep running into - a
    # residual inside +/-15mm gets absorbed as the jaws close, which is why
    # cutting the residual from 12mm to 2mm changed no success rate.
    open_position: float = 0.030
    # Commanded finger position when closing, so the closed gap is twice it.
    # Against a 30mm object that is 12mm of gap, i.e. 18mm of squeeze - the
    # quantity the contact force follows. It has to scale with the object or
    # grip force gets confounded with clearance: the lab's 50mm cubes would
    # otherwise be squeezed 38mm instead of 18mm. (side - 0.018) / 2 keeps the
    # squeeze fixed and reproduces this value at 30mm.
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
    # was measured to spin the object ~12 deg about z before the gripper closed,
    # after which closing strikes its corners and cams it out. Descending in
    # short increments keeps the path near-vertical.
    descend_steps: int = 1

    # Stage 1 isolates grasp mechanics from perception, so a detector
    # observation is a nice-to-have trigger, not a precondition. The course
    # detector is tuned for the original object and does not always see a
    # smaller one; blocking on it would inject a perception limitation into a
    # mechanics experiment. Stage 2 re-introduces perception deliberately.
    detector_wait_s: float = 20.0

    # --- Vision in the loop ---
    # Use the detector's own estimate as the grasp target instead of simulator
    # ground truth. Measured detector error (10 poses, one launch via
    # /reset_object): lateral 2.1 mm mean / 0.6 sd, depth +27.5 mm mean /
    # 0.8 sd. The depth term is a constant geometric bias, not noise: the
    # detector takes the centroid of the largest depth contour and so measures
    # the object's top face while the target needs its centre. Subtracting the
    # measured constant is calibration; the residual 0.5-0.8 mm is what a real
    # system cannot remove.
    use_detector_target: bool = False
    detector_z_bias: float = 0.0275

    # Re-measure the object from the hover pose and re-target before descending.
    # Requires the detector to run with lock_first_detection:=false. This is the
    # naive form of closed-loop correction: re-observing the object alone
    # through the *same* miscalibrated extrinsic returns the same biased answer,
    # so it cannot correct hand-eye error. See the step 3 result document.
    pregrasp_redetect: bool = False
    redetect_timeout_s: float = 5.0

    # How many gripper readings to collect before descending, keeping the
    # deepest. One reproduces the single-reading behaviour every tolerance
    # batch through stage19 was measured under, and is the default so that new
    # batches stay comparable with them. Best-of-N was tried against the depth
    # criterion and does not help: the good and bad readings split per trial,
    # not per frame, so all N samples in a launch come from the same group
    # (see the 2026-09-06 root-cause document). The knob remains so that
    # experiment can be re-run.
    measure_samples: int = 1

    # Radial correction for the optical readings, in metres of effective lever.
    #
    # A body's optical centroid sits off its own frame origin by an amount that
    # grows with how far off the optical axis it is seen. The two terms cancel
    # while object and gripper are viewed from the same direction - which is
    # the case at hover with no calibration error, where the residual measures
    # +1.6mm - and stop cancelling once a bias separates them: at 192mm the
    # gripper's reading runs 12.3mm long and the residual reaches -11.1mm.
    #
    # Measured over 18 trials at three biases, the overshoot is 47.8 +- 4.6mm
    # times the sine of the off-axis angle. That angle comes out of the camera
    # reading itself and never touches the extrinsic, which is what makes the
    # term computable on a system that is never calibrated. Zero keeps the
    # behaviour every earlier batch was measured under.
    view_gain_gripper: float = 0.0
    view_gain_object: float = 0.0

    # Endpoint closed-loop baseline: iterative image-space servoing with the
    # Jacobian estimated on line, which is the classical way this problem is
    # solved (Hutchinson/Hager/Corke 1996 for the taxonomy, Hosoda & Asada 1994
    # and Piepmeier 1999 for the estimation, Horaud/Dornaika/Espiau 1998 for
    # grasping by alignment). It exists so the one-shot scheme has something to
    # be compared against: both read the same two optical measurements, and
    # neither touches the extrinsic, but this one spends probe motions to learn
    # the mapping and then iterates, where the other adds the difference to
    # forward kinematics once.
    #
    # Mutually exclusive with relative_correction.
    servo_baseline: bool = False
    servo_max_iters: int = 6
    servo_gain: float = 0.7
    servo_tol_m: float = 0.004
    servo_probe_m: float = 0.02
    servo_max_step_m: float = 0.08

    # Relative (differential) visual correction. Measures object and gripper
    # optically in the camera frame and adds their difference to the gripper
    # pose known from kinematics:
    #     target = gripper_kin + R * (object_cam - gripper_cam)
    # A translation error in the camera->world extrinsic enters both optical
    # terms identically and cancels in the subtraction, which plain
    # re-detection cannot do. What remains is a fixed geometric offset: the
    # camera sees each body's upper surface rather than its frame origin. That
    # offset does not scale with calibration error, so it is calibrated once
    # and subtracted, exactly like detector_z_bias.
    relative_correction: bool = False
    rel_offset_x: float = 0.0
    rel_offset_y: float = 0.0
    # Return to hover over the corrected point before descending, so the lateral
    # move happens at a safe height instead of as a diagonal sweep past the
    # object with the jaws open.
    rehover_after_correction: bool = True
    # Yaw freedom allowed when planning the hover pose. Default keeps
    # the historical 0.01 rad; larger values trade tool-axis rotation,
    # which a symmetric parallel jaw does not care about, for reachable
    # IK solutions near the workspace edge.
    hover_yaw_tolerance: float = 0.01
    # Pin one joint at the hover pose to select a single IK branch.
    # The residual bimodality tracked since step 7 turned out to be two
    # joint solutions for the same Cartesian pose, joint5 sitting near
    # -0.08 in one and +0.15 in the other.
    lock_ik_branch: bool = False
    branch_joint: str = "openarm_left_joint5"
    branch_centre: float = -0.083
    branch_tolerance: float = 0.08
    # Simulation seconds to hold still before taking the paired
    # kinematic and optical readings, so both see the same pose.
    settle_before_measure_s: float = 0.0

    # --- Stage 2: hand-eye calibration error and pre-grasp visual correction ---
    # A miscalibrated camera-to-base transform biases every observation of the
    # object by the same constant vector, so an open-loop grasp inherits it in
    # full. A pre-grasp correction re-measures the object RELATIVE to the
    # gripper, which cancels that common-mode bias and leaves only sensor noise.
    # Modelling the correction as exact cancellation is an idealisation: a real
    # servo loop also carries gain, latency and residual-calibration error.
    calib_bias_x: float = 0.0
    calib_bias_y: float = 0.0
    vision_noise_std: float = 0.001
    visual_correction: bool = False

    # Timeouts, all in simulation seconds.
    settle_duration_s: float = 1.0
    contact_confirm_timeout_s: float = 5.0
    gripper_action_timeout_s: float = 10.0
    plan_timeout_s: float = 30.0

    def snapshot(self) -> dict:
        """Plain dict for embedding in trial records."""
        return asdict(self)
