# Adapted from run_simulation_with_detector.launch.py in the course package
# openarm_moveit_config (declared author yhang, declared <license>BSD</license>;
# see NOTICE and THIRD_PARTY.md section 2). 26 of that file's 71 unique
# effective lines are still present: launch boilerplate, the standard
# ros2_control pattern of loading controllers on process exit, and about nine
# variable names.

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ["MUJOCO_GL"] = "osmesa"


def generate_launch_description():
    # Which arm the trial drives. Read from the environment for the same reason
    # CAM_BIAS_X/Y are: the sweeps set it per run and a launch argument would
    # have to be threaded through every one of them. Anything other than left
    # or right is refused here rather than producing a node that subscribes to
    # geoms that do not exist and reports every trial as no_contact.
    grasp_arm = os.environ.get("GRASP_ARM", "left")
    if grasp_arm not in ("left", "right"):
        raise ValueError(
            "GRASP_ARM must be 'left' or 'right', got %r" % (grasp_arm,))

    launch_dir = os.path.dirname(__file__)
    model_dir = os.path.join(launch_dir, "..", "model")

    # Scene file is selectable so a batch can randomise the object's initial
    # pose per trial. Repeating one fixed pose N times measures noise, not
    # generalisation.
    scene_name = os.environ.get("PHYSICS_SCENE", "table_scene_physics.xml")
    mujoco_xml_file = os.path.join(model_dir, scene_name)
    urdf_file = os.path.join(model_dir, "table_scene_physics.urdf")
    controller_config = os.path.join(launch_dir, "physics_controllers.yaml")
    detector_script = os.path.join(launch_dir, "apple_detector.py")
    table_script = os.path.join(launch_dir, "spawn_table_scene.py")

    with open(urdf_file, "r") as handle:
        robot_desc = handle.read()

    viewer_render_period = LaunchConfiguration("viewer_render_period", default="0.0166666667")

    node_mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True},
            {"mujoco_model_path": mujoco_xml_file},
            controller_config,
            {"viewer_render_period": viewer_render_period},
            {"camera_period": 0.1666666667},
            {"grasp_telemetry.object_geom": "physics_object_geom"},
            {"grasp_telemetry.gripper_body":
                "openarm_%s_hand" % grasp_arm},
            # The jaws meet midway between these two, which is the only
            # reference an offset can be measured from and still mean the
            # same thing in a different approach pose. gripper_body above
            # is the link origin and does not.
            #
            # The second word names the finger's side within the hand, not the
            # arm, so the right arm's pair is openarm_right_left_finger and
            # openarm_right_right_finger.
            {"grasp_telemetry.left_finger_geom":
                "openarm_%s_left_finger_collision" % grasp_arm},
            {"grasp_telemetry.right_finger_geom":
                "openarm_%s_right_finger_collision" % grasp_arm},
            {"grasp_telemetry.publish_every_n_steps": 10},
        ],
    )

    node_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    # Hand-eye calibration error, modeled where it actually lives: a translation
    # error in the camera->world extrinsic, not a synthetic offset added after
    # the fact to ground truth. Because this transform's rotation is identity,
    # the bias composes as a plain vector add and reaches the detector's
    # world-frame output 1:1, so CAM_BIAS_X/Y can reuse Stage 2's exact sweep
    # grid for a directly comparable curve driven by a real detector instead
    # of a synthetic Gaussian.
    cam_bias_x = float(os.environ.get("CAM_BIAS_X", "0.0"))
    cam_bias_y = float(os.environ.get("CAM_BIAS_Y", "0.0"))
    tf_world_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            str(0.5 + cam_bias_x), str(0.0 + cam_bias_y), "1.2",
            "0", "0", "0", "1", "world", "camera_base_link",
        ],
        output="screen",
    )

    tf_cam_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "1", "0", "0", "0", "camera_base_link", "camera_optical_frame"],
        output="screen",
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_left_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_arm_controller"],
        output="screen",
    )
    load_right_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_arm_controller"],
        output="screen",
    )
    load_left_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_hand_controller"],
        output="screen",
    )
    load_right_hand = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_hand_controller"],
        output="screen",
    )

    # Unlatching lets the detector keep re-measuring, which pre-grasp
    # re-detection needs. Default stays latched: that is the course demo's
    # behaviour and every earlier experiment ran with it.
    detector_unlatched = os.environ.get("DETECTOR_UNLATCHED", "0") == "1"
    detector_cam_frame = os.environ.get("DETECTOR_CAM_FRAME", "0") == "1"
    detector_cmd = [
        "python3", detector_script, "--ros-args",
        "-p", "use_sim_time:=true",
        "-p", "lock_first_detection:={}".format(
            "false" if detector_unlatched else "true"),
        "-p", "publish_camera_frame:={}".format(
            "true" if detector_cam_frame else "false"),
    ]
    # The window schedule has to be settable per launch: deciding whether
    # rel_offset tracks the viewing angle needs the same trials run with the
    # converging cascade and with a single fixed window, and "30" alone gives
    # one pass at the TF-projected centre, which is the fixed-ROI behaviour.
    roi_cascade = os.environ.get("DETECTOR_ROI_CASCADE", "").strip()
    if roi_cascade:
        detector_cmd += ["-p", "roi_cascade:=[{}]".format(roi_cascade)]
    # The object band's far edge has to follow the object's height, or a short
    # one falls past it and never produces a blob.
    object_depth_hi = os.environ.get("DETECTOR_OBJECT_DEPTH_HI", "").strip()
    if object_depth_hi:
        detector_cmd += ["-p", "object_depth_hi:={}".format(object_depth_hi)]
    # Sensor noise on the rendered depth. MuJoCo's depth is exact; real depth
    # is not, and every threshold in the detector was tuned on the exact kind.
    depth_noise = os.environ.get("DETECTOR_DEPTH_NOISE_STD", "").strip()
    if depth_noise:
        detector_cmd += ["-p", "depth_noise_std:={}".format(depth_noise)]
    depth_noise_seed = os.environ.get("DETECTOR_DEPTH_NOISE_SEED", "").strip()
    if depth_noise_seed:
        detector_cmd += ["-p", "depth_noise_seed:={}".format(depth_noise_seed)]
    # Has to be set together with a scene whose camera carries the same value;
    # PHYSICS_SCENE selects that scene.
    fovy = os.environ.get("DETECTOR_FOVY", "").strip()
    if fovy:
        detector_cmd += ["-p", "fovy_deg:={}".format(float(fovy))]
    node_detector = ExecuteProcess(cmd=detector_cmd, output="screen")

    spawn_table = ExecuteProcess(
        cmd=["python3", table_script, "--ros-args", "-p", "use_sim_time:=true"],
        output="screen",
    )

    launch_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group_physics.launch.py"))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viewer_render_period",
                default_value="0.0166666667",
                description="Simulated seconds between viewer redraws.",
            ),
            node_mujoco,
            node_rsp,
            tf_world_to_cam,
            tf_cam_to_optical,
            node_detector,
            launch_moveit,
            spawn_table,
            load_jsb,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_jsb,
                    on_exit=[load_left_arm, load_right_arm, load_left_hand, load_right_hand],
                )
            ),
        ]
    )
